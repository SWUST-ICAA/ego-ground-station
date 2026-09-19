"""Per-host SSH transport. No flight command is retried after a network failure."""
import json
import os
from pathlib import Path
import shlex
import time
import uuid
import paramiko

ROOT=Path(__file__).resolve().parents[1]
CONTAINER='fast-drone-250'
ENTRY='/fast_drone_ws/deploy/entrypoint.sh'
AGENT='/tmp/ground_station/agent.py'


class Remote:
    def __init__(self, config):
        self.config=config
        self.client=None

    def close(self):
        if self.client:self.client.close()
        self.client=None

    def connect(self):
        if self.client and self.client.get_transport() and self.client.get_transport().is_active():return
        self.close()
        client=paramiko.SSHClient()
        keys=Path.home()/'.config/fast-drone-ground-station/known_hosts'
        keys.parent.mkdir(parents=True,exist_ok=True)
        keys.touch(exist_ok=True)
        client.load_host_keys(str(keys))
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(self.config['host'],username=self.config['user'],password=self.config.get('password') or None,
                       timeout=5,banner_timeout=5,auth_timeout=5,allow_agent=True,look_for_keys=True)
        client.get_transport().set_keepalive(10)
        self.client=client

    def shell(self, command, data=None, timeout=25):
        self.connect()
        i,o,e=self.client.exec_command(command,timeout=timeout)
        try:
            if data is not None:i.write(data);i.flush()
            i.channel.shutdown_write()
            out=o.read().decode();err=e.read().decode();rc=o.channel.recv_exit_status()
            if rc:raise RuntimeError((err or out or '远程命令失败')[-1200:])
            return out
        finally:i.close();o.close();e.close()

    def rpc(self, command, **params):
        payload=dict(command=command,**params)
        if command!='status':payload['id']=uuid.uuid4().hex
        try:
            out=self.shell('docker exec -i '+CONTAINER+' '+ENTRY+' python3 '+AGENT+' request',json.dumps(payload,allow_nan=False),timeout=25)
            result=json.loads(out)
        except Exception as e:
            if command not in {'status','prepare'}:
                raise RuntimeError('命令结果未确认，请刷新状态；不会自动重发：'+str(e)) from e
            raise
        if not result.get('ok'):raise RuntimeError(result.get('error','操作失败'))
        return result

    def status(self):
        running=self.shell("docker inspect -f '{{.State.Running}}' "+CONTAINER).strip()
        if running!='true':return dict(program=False,connected=False,ready=False,mission=dict(phase='STOPPED'),reasons=['机上程序已停止'])
        try:
            return dict(self.rpc('status')['status'],program=True)
        except RuntimeError as e:
            if 'No such file' in str(e) or 'Connection refused' in str(e):
                return dict(program=True,connected=False,ready=False,mission=dict(phase='NO_AGENT'),reasons=['点击启动程序以加载观测代理'])
            raise

    def start_program(self):
        try:running=self.shell("docker inspect -f '{{.State.Running}}' "+CONTAINER).strip()
        except RuntimeError as e:
            if 'No such object' not in str(e):raise
            directory='/home/'+self.config['user']+'/Fast-Drone-250'
            self.shell('cd '+shlex.quote(directory)+' && bash deploy/run.sh '+str(int(self.config['id'])))
            running='true'
        if running!='true':self.shell('docker start '+CONTAINER)
        try:
            current=self.rpc('status')['status']
        except Exception as e:
            if not any(t in str(e) for t in ['No such file','Connection refused']):raise
        else:
            if current.get('stopping'):raise RuntimeError('上次停止已锁定代理，请执行关闭程序后重新启动')
            if 'flight-v2' not in current.get('capabilities',[]):raise RuntimeError('机载代理需更新坐标与跳点功能；着陆且未解锁时先关闭程序，再重新启动')
            return current
        destination='/home/'+self.config['user']+'/.fast-drone-ground-station'
        self.shell('mkdir -p '+shlex.quote(destination))
        sftp=self.client.open_sftp()
        try:
            for name in ('agent.py','mission.py','geo.py','fence.py'):
                sftp.put(str(ROOT/'onboard'/name),destination+'/'+name)
        finally:sftp.close()
        self.shell('docker exec '+CONTAINER+' mkdir -p /tmp/ground_station')
        for name in ('agent.py','mission.py','geo.py','fence.py'):
            self.shell('docker cp '+shlex.quote(destination+'/'+name)+' '+CONTAINER+':/tmp/ground_station/'+name)
        self.shell('docker exec -d '+CONTAINER+' '+ENTRY+' python3 '+AGENT+' serve --aircraft '+str(int(self.config['id'])))
        for attempt in range(15):
            time.sleep(.4)
            try:return self.rpc('status')['status']
            except Exception:
                if attempt==14:raise

    def stop_program(self):
        # Container shutdown never bypasses the onboard, fresh landed/unarmed check.
        self.rpc('stop')
        self.shell('docker stop -t 10 '+CONTAINER)
        return dict(program=False,connected=False,ready=False,mission=dict(phase='STOPPED'))

    def execute(self, command, **kwargs):
        if command=='start_program':return self.start_program()
        if command=='stop_program':return self.stop_program()
        return self.rpc(command,**kwargs)['status']

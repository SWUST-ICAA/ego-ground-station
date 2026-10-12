import json
import sys
from pathlib import Path

ROOT=Path('/home/sakura/HJJ/ego-ground-station')
sys.path.insert(0,str(ROOT))
from ground_station.transport import Remote


def test_empty_agent_reply_is_reported_as_no_agent():
    remote=Remote({'id':4,'host':'example','user':'icaa','container':'fast-drone-250'})
    remote.shell=lambda command: 'true'
    def empty_reply(command, **params):
        raise json.JSONDecodeError('empty agent reply','',0)
    remote.rpc=empty_reply
    state=remote.status()
    assert state['program'] is True
    assert state['mission']['phase']=='NO_AGENT'
    assert '启动程序' in state['reasons'][0]

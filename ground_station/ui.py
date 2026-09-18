"""Native PyQt ground station. Each worker owns exactly one SSH connection."""
import json
import os
from pathlib import Path
import queue
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets as W
from .transport import Remote
from .demo import Demo

PHASES={'IDLE':'待命','ARMING':'解锁中','TAKEOFF':'起飞中','OUTBOUND':'前往航点','RETURNING':'规划返航',
        'LAND_REQUESTED':'请求降落','LANDING':'降落中','COMPLETE':'任务完成','ERROR':'异常','MANUAL':'外部接管',
        'STOPPED':'程序已停止','NO_AGENT':'待启动观测代理'}


class Worker(QtCore.QThread):
    state=QtCore.pyqtSignal(int,dict)
    result=QtCore.pyqtSignal(int,str,bool,str)
    def __init__(self,config,demo):
        super().__init__();self.n=config['id'];self.backend=Demo(config) if demo else Remote(config)
        self.commands=queue.Queue();self.halt=threading.Event();self.pending=False
    def submit(self,command,params=None):
        if self.pending:return False
        self.pending=True;self.commands.put((command,params or {}));return True
    def run(self):
        last_poll=0
        while not self.halt.is_set():
            try:command,params=self.commands.get(timeout=.1)
            except queue.Empty:command=None
            if command:
                try:
                    state=self.backend.execute(command,**params)
                    self.state.emit(self.n,state);self.result.emit(self.n,command,True,'完成')
                except Exception as e:self.result.emit(self.n,command,False,str(e))
                finally:self.pending=False
                last_poll=0
            if time.monotonic()-last_poll>.7 and not self.halt.is_set():
                try:
                    state=self.backend.status();state['online']=True
                    self.state.emit(self.n,state)
                except Exception as e:
                    self.backend.close()
                    self.state.emit(self.n,dict(online=False,error=str(e)))
                last_poll=time.monotonic()
        self.backend.close()


class LocalPlot(W.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumSize(400,290);self.state={};self.points=[];self.trace=[];self.aircraft=1
    def change(self,n,state,points,trace):
        self.aircraft=n;self.state=state;self.points=points;self.trace=trace;self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#111d2c'))
        p.setPen(QtGui.QColor('#c5d5e7'));p.drawText(20,27,f'{self.aircraft} 号机 · 独立局部坐标 / 米')
        local=[(v['a'],v['b']) for v in self.points if v['kind']=='local']
        actual=self.state.get('mission',{}).get('points',[])
        targets=[(a[0],a[1]) for a in actual] or local
        pos=self.state.get('position');home=self.state.get('mission',{}).get('home')
        coords=self.trace+targets+([(pos[0],pos[1])] if pos else [])+([(home[0],home[1])] if home else [])+[(0,0)]
        extent=max(5,max(abs(v) for xy in coords for v in xy)+2)
        scale=min(self.width()-80,self.height()-90)/(2*extent)
        cx,cy=self.width()/2,(self.height()+30)/2
        def xy(a,b):return QtCore.QPointF(cx+a*scale,cy-b*scale)
        p.setPen(QtGui.QPen(QtGui.QColor('#233247'),1))
        step=max(1,round(extent/5))
        for v in range(-int(extent),int(extent)+1):
            if v%step==0:
                p.drawLine(xy(v,-extent),xy(v,extent));p.drawLine(xy(-extent,v),xy(extent,v))
        p.setPen(QtGui.QPen(QtGui.QColor('#58708d'),1));p.drawLine(xy(-extent,0),xy(extent,0));p.drawLine(xy(0,-extent),xy(0,extent))
        p.drawText(int(cx+extent*scale)-30,int(cy)-6,'+X');p.drawText(int(cx)+7,int(cy-extent*scale)+14,'+Y')
        p.drawText(20,self.height()-15,f'网格 {step}m   各机坐标不可直接比较')
        if len(self.trace)>1:
            p.setPen(QtGui.QPen(QtGui.QColor('#36c5cc'),2));p.drawPolyline(QtGui.QPolygonF([xy(*a) for a in self.trace]))
        p.setPen(QtGui.QPen(QtGui.QColor('#f6b655'),2));p.setBrush(QtCore.Qt.NoBrush)
        for i,pt in enumerate(targets):
            q=xy(*pt);p.drawEllipse(q,6,6);p.drawText(q+QtCore.QPointF(9,-8),str(i+1))
        if home:
            q=xy(home[0],home[1]);p.setPen(QtGui.QColor('#6be2a5'));p.drawRect(QtCore.QRectF(q.x()-6,q.y()-6,12,12));p.drawText(q+QtCore.QPointF(9,15),'起飞点')
        if pos:
            q=xy(pos[0],pos[1]);p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#35d0db'));p.drawEllipse(q,7,7)
        p.end()


class Window(W.QMainWindow):
    def __init__(self,config,path,demo=False):
        super().__init__();self.config=config;self.path=Path(path);self.demo=demo
        self.states={};self.traces={a['id']:[] for a in config['aircraft']};self.current=config['aircraft'][0]['id']
        self.workers={};self.event_seen={};self.loading=False;self.received={}
        self.setWindowTitle('Fast Drone · 独立单机地面站'+(' [模拟演示]' if demo else ''))
        self.resize(1380,940)
        central=W.QWidget();self.setCentralWidget(central);layout=W.QVBoxLayout(central);layout.setContentsMargins(24,20,24,18);layout.setSpacing(14)
        top=W.QHBoxLayout();title=W.QLabel('FAST DRONE  /  地面站');title.setObjectName('title');top.addWidget(title);top.addStretch()
        badge=W.QLabel('● 模拟演示 · 不连接飞机' if demo else '● 实机 · 1 / 2 / 3 独立运行');badge.setObjectName('badge');top.addWidget(badge);layout.addLayout(top)
        desc=W.QLabel('上电 → 启动程序 → 设置各机航点 → 一键执行 → 规划返航 → 自动降落');desc.setObjectName('muted');layout.addWidget(desc)
        self.table=W.QTableWidget(len(config['aircraft']),9)
        self.table.setHorizontalHeaderLabels(['选择','飞机 / SSH','程序 / 链路','本机定位','GNSS','电量','飞行模式 / 解锁','局部位置 x / y / z','任务'])
        self.table.verticalHeader().hide();self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.ResizeToContents);self.table.horizontalHeader().setSectionResizeMode(7,W.QHeaderView.Stretch)
        self.table.setMaximumHeight(195)
        for row,a in enumerate(config['aircraft']):
            item=W.QTableWidgetItem();item.setFlags(QtCore.Qt.ItemIsEnabled|QtCore.Qt.ItemIsUserCheckable);item.setCheckState(QtCore.Qt.Checked);self.table.setItem(row,0,item)
            for col in range(1,9):self.table.setItem(row,col,W.QTableWidgetItem('—'))
            self.table.item(row,1).setText(f"{a['id']} 号  {a['host']}");self.table.setRowHeight(row,45)
            worker=Worker(a,demo);worker.state.connect(self.on_state);worker.result.connect(self.on_result);self.workers[a['id']]=worker
        layout.addWidget(self.table)
        tools=W.QHBoxLayout();self.buttons={}
        for name,label,style in [('start_program','启动选中程序',''),('stop_program','关闭选中程序',''),('start','一键起飞并执行','primary'),('return','选中飞机返航',''),('land','选中飞机就地降落','danger')]:
            button=W.QPushButton(label);button.setObjectName(style);button.clicked.connect(lambda checked=False,c=name:self.batch(c));tools.addWidget(button);self.buttons[name]=button
        layout.addLayout(tools)
        splitter=W.QSplitter();layout.addWidget(splitter,1)
        left=W.QWidget();ll=W.QVBoxLayout(left);ll.setContentsMargins(0,0,12,0)
        header=W.QHBoxLayout();header.addWidget(W.QLabel('航点设置'));self.selector=W.QComboBox()
        for a in config['aircraft']:self.selector.addItem(f"{a['id']} 号机",a['id'])
        self.selector.currentIndexChanged.connect(self.switch);header.addWidget(self.selector);header.addStretch();ll.addLayout(header)
        self.notice=W.QLabel();self.notice.setWordWrap(True);ll.addWidget(self.notice)
        self.waypoints=W.QTableWidget(0,3);self.waypoints.setHorizontalHeaderLabels(['坐标类型','X / 纬度','Y / 经度']);self.waypoints.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch);ll.addWidget(self.waypoints)
        self.waypoints.itemChanged.connect(self.save_edits)
        controls=W.QHBoxLayout()
        add=W.QPushButton('+ 米制点');add.clicked.connect(lambda:self.add_point('local'));controls.addWidget(add)
        self.add_geo=W.QPushButton('+ 经纬度点');self.add_geo.clicked.connect(lambda:self.add_point('geo'));controls.addWidget(self.add_geo)
        delete=W.QPushButton('删除选中点');delete.clicked.connect(self.delete_point);controls.addWidget(delete)
        ll.addLayout(controls)
        text=W.QLabel('米制：本机 map 绝对坐标，单位 m。经纬度：WGS84 十进制度。\n航点按顺序执行，最后自动返回起飞点并降落。高度固定 1.5m。');text.setWordWrap(True);text.setObjectName('muted');ll.addWidget(text)
        splitter.addWidget(left)
        right=W.QWidget();rl=W.QVBoxLayout(right);rl.setContentsMargins(10,0,0,0)
        self.plot=LocalPlot();rl.addWidget(self.plot,1);self.details=W.QLabel('等待遥测');self.details.setWordWrap(True);rl.addWidget(self.details);splitter.addWidget(right);splitter.setSizes([500,760])
        self.log=W.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumHeight(130);self.log.document().setMaximumBlockCount(300);layout.addWidget(self.log)
        self.setStyleSheet('''QMainWindow,QWidget{background:#0b1421;color:#dce7f5;font-family:"Noto Sans CJK SC","DejaVu Sans";font-size:13px} QLabel#title{font-size:23px;font-weight:700} QLabel#badge{color:#39d0ca;background:#142b35;border-radius:6px;padding:9px} QLabel#muted{color:#8297af} QPushButton{background:#1b2a40;border:1px solid #2d425c;border-radius:6px;padding:10px 14px} QPushButton:hover{background:#263b55} QPushButton:disabled{color:#586777;background:#142031} QPushButton#primary{background:#1b938f;color:white;font-weight:bold} QPushButton#danger{background:#69353c;color:#ffdbdf} QTableWidget,QPlainTextEdit{background:#101d2c;alternate-background-color:#152436;border:1px solid #25374b;border-radius:5px;gridline-color:#25374b;selection-background-color:#214e63} QHeaderView::section{background:#17283c;color:#9bb2cc;padding:8px;border:0} QComboBox{background:#1b2a40;padding:6px;border:1px solid #30465f;border-radius:4px} QLineEdit{background:#142236} QSplitter::handle{background:#25374b;width:1px}''')
        self.load_points();self.write_log('模拟演示：2 号机无 GNSS，可用米制航点。' if demo else '启动程序不会解锁；任务在各机独立执行。')
        for worker in self.workers.values():worker.start()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh_detail);self.timer.start(500)

    def write_log(self,text):self.log.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)
    def selected(self):return [a['id'] for row,a in enumerate(self.config['aircraft']) if self.table.item(row,0).checkState()==QtCore.Qt.Checked]
    def points(self,n):return self.config.setdefault('waypoints',{}).setdefault(str(n),[])
    def persist(self):
        temporary=self.path.with_suffix('.tmp')
        fd=os.open(str(temporary),os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
        with os.fdopen(fd,'w') as f:json.dump(self.config,f,ensure_ascii=False,indent=2)
        os.replace(temporary,self.path);os.chmod(self.path,0o600)
    def save_edits(self,*args):
        if self.loading:return
        result=[]
        try:
            import math
            for row in range(self.waypoints.rowCount()):
                a=float(self.waypoints.item(row,1).text());b=float(self.waypoints.item(row,2).text())
                if not math.isfinite(a+b):raise ValueError()
                result.append(dict(kind=self.waypoints.item(row,0).data(QtCore.Qt.UserRole),a=a,b=b))
        except (ValueError,AttributeError):
            self.notice.setText('坐标输入无效，尚未保存；请填写有限数值。');return False
        self.config['waypoints'][str(self.current)]=result;self.persist();return True
    def load_points(self):
        self.loading=True;self.waypoints.setRowCount(0)
        for point in self.points(self.current):
            row=self.waypoints.rowCount();self.waypoints.insertRow(row)
            item=W.QTableWidgetItem('米制 map' if point['kind']=='local' else '经纬度 WGS84');item.setData(QtCore.Qt.UserRole,point['kind']);item.setFlags(QtCore.Qt.ItemIsEnabled|QtCore.Qt.ItemIsSelectable);self.waypoints.setItem(row,0,item)
            self.waypoints.setItem(row,1,W.QTableWidgetItem(str(point['a'])));self.waypoints.setItem(row,2,W.QTableWidgetItem(str(point['b'])))
        self.loading=False;self.refresh_detail()
    def switch(self):self.current=self.selector.currentData();self.load_points()
    def add_point(self,kind):
        if self.save_edits() is False:return
        state=self.states.get(self.current,{})
        if kind=='geo':
            gps=state.get('gps')
            if not state.get('geo_ready') or not gps:return
            a,b=gps['latitude'],gps['longitude']
        else:
            pos=state.get('position') or [0,0,0];a,b=round(pos[0]+2,2),round(pos[1],2)
        self.points(self.current).append(dict(kind=kind,a=a,b=b));self.persist();self.load_points()
    def delete_point(self):
        row=self.waypoints.currentRow()
        if row>=0:self.points(self.current).pop(row);self.persist();self.load_points()
    def batch(self,command):
        if self.save_edits() is False:
            W.QMessageBox.warning(self,'坐标无效','请修正当前表格中的坐标后再操作。');return
        ids=self.selected()
        if not ids:return
        if any(self.workers[n].pending for n in ids):
            W.QMessageBox.information(self,'操作处理中','选中的飞机尚有操作未完成，请等待状态更新。');return
        if command=='start':
            errors=[]
            for n in ids:
                s=self.states.get(n,{})
                if time.monotonic()-self.received.get(n,0)>3 or not s.get('online',False) or not s.get('ready'):errors.append(f'{n} 号：状态未就绪')
                if not self.points(n):errors.append(f'{n} 号：未设置航点')
                if any(p['kind']=='geo' for p in self.points(n)) and not s.get('geo_ready'):errors.append(f'{n} 号：经纬度不可用，改用米制点')
            if errors:W.QMessageBox.warning(self,'不能执行任务','\n'.join(errors));return
        if command in {'start','return','land','stop_program'}:
            detail={'start':'请求解锁并起飞，按各机航点飞行，然后规划返航并自动降落。','return':'取消后续航点，通过规划器返回各自起飞点并降落。','land':'在当前位置请求 AUTO.LAND，取消当前任务。','stop_program':'仅关闭已着陆且未解锁的飞机程序。'}[command]
            if W.QMessageBox.question(self,'确认操作',f"飞机：{', '.join(map(str,ids))}\n{detail}",W.QMessageBox.Yes|W.QMessageBox.No,W.QMessageBox.No)!=W.QMessageBox.Yes:return
        for n in ids:
            params=dict(waypoints=list(self.points(n))) if command=='start' else {}
            self.workers[n].submit(command,params)
            self.write_log(f'{n} 号 · 已提交 {command}；各机独立接受/拒绝，非同步起飞保证')
    def on_result(self,n,command,ok,text):
        self.write_log(f'{n} 号 · {command} '+('成功' if ok else '失败')+'：'+text)
        if not ok:W.QMessageBox.warning(self,f'{n} 号操作失败',text)
    def on_state(self,n,state):
        if 'online' not in state:state['online']=True
        if not state.get('online'):
            state=dict(self.states.get(n,{}),**state,ready=False,fresh=False,geo_ready=False)
        self.states[n]=state;self.received[n]=time.monotonic()
        row=next(i for i,a in enumerate(self.config['aircraft']) if a['id']==n)
        p=state.get('position');mode=state.get('mode','—');phase=state.get('mission',{}).get('phase','—')
        values=['在线' if state.get('online') else 'SSH 离线','就绪' if state.get('ready') else '未就绪',
                '经纬度可用' if state.get('geo_ready') else '仅米制目标',
                f"{state['battery']:.0f}%" if state.get('battery') is not None else '—',
                mode+(' / 已解锁' if state.get('armed') else ' / 未解锁' if state.get('fresh') else ' / 未知'),
                ' / '.join(f'{v:.2f}' for v in p) if p else '—',PHASES.get(phase,phase)]
        if state.get('program') is False:values[0]='程序已停止'
        elif phase=='NO_AGENT':values[0]='待加载代理'
        for col,value in enumerate(values,2):
            self.table.item(row,col).setText(value)
            self.table.item(row,col).setToolTip(state.get('error','')+'\n'+' / '.join(state.get('reasons',[])))
        if p:
            trace=self.traces[n]
            if not trace or abs(trace[-1][0]-p[0])+abs(trace[-1][1]-p[1])>.04:trace.append((p[0],p[1]))
            self.traces[n]=trace[-1500:]
        events=state.get('events',[])
        last=self.event_seen.get(n)
        if events and events[-1]!=last:
            self.write_log(f"{n} 号 · {events[-1]['text']}");self.event_seen[n]=events[-1]
        self.refresh_detail()
    def refresh_detail(self):
        s=self.states.get(self.current,{});fresh=time.monotonic()-self.received.get(self.current,0)<3
        geo=bool(fresh and s.get('online') and s.get('geo_ready'));self.add_geo.setEnabled(geo)
        self.notice.setText('经纬度和米制坐标均可设置。' if geo else '经纬度参考不可用：仍可设置米制点；飞行取决于本机定位是否就绪。')
        m=s.get('mission',{});gps=s.get('gps');parts=[]
        if not fresh:parts.append('遥测未更新，请等待连接')
        parts.extend(s.get('reasons',[]))
        if s.get('error'):parts.append(s['error'][-220:])
        if gps:parts.append(f"GNSS：{gps['latitude']:.7f}, {gps['longitude']:.7f}")
        parts.append(f"任务：{PHASES.get(m.get('phase'),m.get('phase','—'))}  |  航点 {min(m.get('index',0)+1,len(m.get('points',[])))}/{len(m.get('points',[]))}")
        if m.get('reason'):parts.append(m['reason'])
        self.details.setText('\n'.join(parts));self.plot.change(self.current,s,self.points(self.current),self.traces[self.current])
    def closeEvent(self,event):
        active=[n for n,s in self.states.items() if s.get('mission',{}).get('active')]
        if active and W.QMessageBox.question(self,'机上任务仍在执行','关闭界面不会停止机上任务，飞机仍会按任务返航降落。确认关闭？',W.QMessageBox.Yes|W.QMessageBox.No,W.QMessageBox.No)!=W.QMessageBox.Yes:event.ignore();return
        self.timer.stop()
        for worker in self.workers.values():worker.halt.set();worker.backend.close()
        for worker in self.workers.values():
            if not worker.wait(3000):event.ignore();self.timer.start();return
        event.accept()

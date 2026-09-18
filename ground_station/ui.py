"""Native PyQt ground station. Each worker owns exactly one SSH connection."""
import json
import os
from pathlib import Path
import queue
import threading
import time
from PyQt5 import QtCore, QtWidgets as W
from .transport import Remote
from .demo import Demo
from .view_widgets import AllCheckBox, MapPanel
from .waypoint_editor import WaypointEditor

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


class Window(W.QMainWindow):
    def __init__(self,config,path,demo=False):
        super().__init__();self.config=config;self.path=Path(path);self.demo=demo
        self.states={};self.traces={a['id']:[] for a in config['aircraft']};self.current=config['aircraft'][0]['id']
        self.workers={};self.event_seen={};self.loading=False;self.received={}
        self.setWindowTitle('Fast Drone · 独立单机地面站'+(' [模拟演示]' if demo else ''))
        screen=W.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1680,int(screen.width()*.9)),min(1080,int(screen.height()*.9)))
        self.setMinimumSize(720,540)
        central=W.QWidget();self.setCentralWidget(central);layout=W.QVBoxLayout(central);layout.setContentsMargins(16,12,16,12);layout.setSpacing(10)
        top=W.QHBoxLayout();title=W.QLabel('FAST DRONE  /  地面站');title.setObjectName('title');top.addWidget(title);top.addStretch()
        badge=W.QLabel('● 模拟演示 · 不连接飞机' if demo else '● 实机 · '+str(len(config['aircraft']))+' 架独立运行');badge.setObjectName('badge');top.addWidget(badge);layout.addLayout(top)
        desc=W.QLabel('上电 → 启动程序 → 设置各机航点 → 一键执行 → 规划返航 → 自动降落');desc.setObjectName('muted');desc.setWordWrap(True);layout.addWidget(desc)
        self.main_splitter=W.QSplitter(QtCore.Qt.Vertical);self.main_splitter.setChildrenCollapsible(False)
        self.sections_adjusted=False;self.main_splitter.splitterMoved.connect(self.mark_sections_adjusted)
        layout.addWidget(self.main_splitter,1)
        status_panel=W.QWidget();status_layout=W.QVBoxLayout(status_panel);status_layout.setContentsMargins(0,0,0,0);status_layout.setSpacing(6)
        selection=W.QHBoxLayout();self.select_all=AllCheckBox('全选');self.select_all.setTristate(True)
        self.select_all.stateChanged.connect(self.set_all_selected);selection.addWidget(self.select_all)
        self.selection_count=W.QLabel();self.selection_count.setObjectName('muted');selection.addWidget(self.selection_count);selection.addStretch();status_layout.addLayout(selection)
        self.checkboxes={}
        self.table=W.QTableWidget(len(config['aircraft']),9)
        self.table.setHorizontalHeaderLabels(['选择','飞机 / SSH','程序 / 链路','本机定位','GNSS','电量','飞行模式 / 解锁','局部位置 x / y / z','任务'])
        self.table.verticalHeader().hide();self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.ResizeToContents);self.table.horizontalHeader().setSectionResizeMode(7,W.QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.Fixed);self.table.setColumnWidth(0,48)
        self.table.setMinimumHeight(85);self.table.setSizePolicy(W.QSizePolicy.Expanding,W.QSizePolicy.Expanding)
        for row,a in enumerate(config['aircraft']):
            box=W.QCheckBox();box.setChecked(True);box.setAccessibleName(f"选择 {a['id']} 号机")
            box.stateChanged.connect(self.update_selection);self.checkboxes[a['id']]=box
            cell=W.QWidget();cell_layout=W.QHBoxLayout(cell);cell_layout.setContentsMargins(0,0,0,0);cell_layout.setAlignment(QtCore.Qt.AlignCenter);cell_layout.addWidget(box)
            self.table.setCellWidget(row,0,cell)
            for col in range(1,9):self.table.setItem(row,col,W.QTableWidgetItem('—'))
            self.table.item(row,1).setText(f"{a['id']} 号  {a['host']}");self.table.setRowHeight(row,32)
            worker=Worker(a,demo);worker.state.connect(self.on_state);worker.result.connect(self.on_result);self.workers[a['id']]=worker
        status_layout.addWidget(self.table,1);self.update_selection()
        self.toolbar=W.QWidget();self.toolbar_grid=W.QGridLayout(self.toolbar);self.toolbar_grid.setContentsMargins(0,0,0,0);self.toolbar_grid.setSpacing(8);self.toolbar_columns=0;self.buttons={}
        for name,label,style in [('start_program','启动选中程序',''),('stop_program','关闭选中程序',''),('start','一键起飞并执行','primary'),('return','选中飞机返航',''),('land','选中飞机就地降落','danger')]:
            button=W.QPushButton(label);button.setObjectName(style);button.clicked.connect(lambda checked=False,c=name:self.batch(c));self.buttons[name]=button
        status_layout.addWidget(self.toolbar);self.main_splitter.addWidget(status_panel)
        splitter=W.QSplitter();splitter.setChildrenCollapsible(False);self.main_splitter.addWidget(splitter)
        left=W.QWidget();ll=W.QVBoxLayout(left);ll.setContentsMargins(0,0,10,0)
        header=W.QHBoxLayout();header.addWidget(W.QLabel('航点设置'));self.selector=W.QComboBox()
        for a in config['aircraft']:self.selector.addItem(f"{a['id']} 号机",a['id'])
        self.selector.currentIndexChanged.connect(self.switch);header.addWidget(self.selector);header.addStretch();ll.addLayout(header)
        self.notice=W.QLabel();self.notice.setWordWrap(True);ll.addWidget(self.notice)
        self.waypoint_editor=WaypointEditor();ll.addWidget(self.waypoint_editor,1)
        self.waypoint_editor.pointsChanged.connect(self.save_points)
        self.waypoint_editor.logMessage.connect(self.write_log)
        text=W.QLabel('米制：本机 map 绝对坐标，单位 m。经纬度：WGS84 十进制度。\n航点按顺序执行，最后自动返回起飞点并降落。高度固定 1.5m。');text.setWordWrap(True);text.setObjectName('muted');ll.addWidget(text)
        self.details=W.QLabel('等待遥测');self.details.setWordWrap(True);ll.addWidget(self.details)
        editor=W.QScrollArea();editor.setWidgetResizable(True);editor.setFrameShape(W.QFrame.NoFrame);editor.setWidget(left);editor.setMinimumSize(350,150)
        splitter.addWidget(editor)
        right=W.QWidget();rl=W.QVBoxLayout(right);rl.setContentsMargins(4,0,0,0)
        map_title=W.QLabel('全部飞机 · 独立局部坐标（各机坐标不可直接比较）');map_title.setWordWrap(True);rl.addWidget(map_title)
        self.map_panel=MapPanel([a['id'] for a in config['aircraft']]);rl.addWidget(self.map_panel,1)
        for n,plot in self.map_panel.plots.items():plot.clicked.connect(self.select_aircraft)
        self.table.cellClicked.connect(lambda row,col:self.select_aircraft(self.config['aircraft'][row]['id']) if col else None)
        splitter.addWidget(right);splitter.setStretchFactor(0,0);splitter.setStretchFactor(1,1);splitter.setSizes([460,1060])
        self.log=W.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMinimumHeight(50);self.log.document().setMaximumBlockCount(300);self.main_splitter.addWidget(self.log)
        self.main_splitter.setSizes([335,590,90]);self.main_splitter.setStretchFactor(0,1);self.main_splitter.setStretchFactor(1,3);self.main_splitter.setStretchFactor(2,0)
        self.setStyleSheet('''QMainWindow,QWidget{background:#0b1421;color:#dce7f5;font-family:"Noto Sans CJK SC","DejaVu Sans";font-size:13px} QLabel#title{font-size:21px;font-weight:700} QLabel#badge{color:#39d0ca;background:#142b35;border-radius:6px;padding:9px} QLabel#muted{color:#8297af} QPushButton{background:#1b2a40;border:1px solid #2d425c;border-radius:6px;padding:10px 14px} QPushButton:hover{background:#263b55} QPushButton:disabled{color:#586777;background:#142031} QPushButton#primary{background:#1b938f;color:white;font-weight:bold} QPushButton#danger{background:#69353c;color:#ffdbdf} QTableWidget,QPlainTextEdit{background:#101d2c;alternate-background-color:#152436;border:1px solid #25374b;border-radius:5px;gridline-color:#25374b;selection-background-color:#214e63} QHeaderView::section{background:#17283c;color:#9bb2cc;padding:8px;border:0} QComboBox{background:#1b2a40;padding:6px;border:1px solid #30465f;border-radius:4px} QLineEdit{background:#142236} QCheckBox{spacing:7px} QCheckBox::indicator{width:16px;height:16px} QSplitter::handle{background:#30465f;width:5px;height:5px} QScrollArea{border:0}''')
        self.adapt_toolbar();self.load_points();self.write_log('模拟演示：2 号机无 GNSS，可用米制航点。' if demo else '启动程序不会解锁；任务在各机独立执行。')
        for worker in self.workers.values():worker.start()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh_detail);self.timer.start(500)

    def write_log(self,text):self.log.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)
    def selected(self):return [n for n,box in self.checkboxes.items() if box.isChecked()]
    def set_all_selected(self,state):
        for box in self.checkboxes.values():
            blocker=QtCore.QSignalBlocker(box);box.setChecked(state==QtCore.Qt.Checked);del blocker
        self.update_selection()
    def update_selection(self,*args):
        count=len(self.selected());total=len(self.checkboxes)
        state=QtCore.Qt.Checked if count==total else QtCore.Qt.PartiallyChecked if count else QtCore.Qt.Unchecked
        blocker=QtCore.QSignalBlocker(self.select_all);self.select_all.setCheckState(state);del blocker
        self.selection_count.setText(f'已选 {count} / {total} 架')
    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'toolbar_grid'):
            self.adapt_toolbar();QtCore.QTimer.singleShot(0,self.fit_sections)
    def mark_sections_adjusted(self,*args):self.sections_adjusted=True
    def fit_sections(self):
        if self.sections_adjusted:return
        height=self.main_splitter.height()
        desired=self.table.rowCount()*32+self.table.horizontalHeader().height()+self.toolbar.sizeHint().height()+self.select_all.sizeHint().height()+16
        top=min(desired,int(height*.4));bottom=min(90,max(50,height//12))
        self.main_splitter.setSizes([top,max(150,height-top-bottom-10),bottom])
    def adapt_toolbar(self):
        columns=5 if self.width()>=1150 else 3
        if columns==self.toolbar_columns:return
        while self.toolbar_grid.count():self.toolbar_grid.takeAt(0)
        for col in range(max(columns,self.toolbar_columns)):self.toolbar_grid.setColumnStretch(col,0)
        for i,button in enumerate(self.buttons.values()):self.toolbar_grid.addWidget(button,i//columns,i%columns)
        for col in range(columns):self.toolbar_grid.setColumnStretch(col,1)
        self.toolbar_columns=columns
    def points(self,n):return self.config.setdefault('waypoints',{}).setdefault(str(n),[])
    def persist(self):
        temporary=self.path.with_suffix('.tmp')
        fd=os.open(str(temporary),os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
        with os.fdopen(fd,'w') as f:json.dump(self.config,f,ensure_ascii=False,indent=2)
        os.replace(temporary,self.path);os.chmod(self.path,0o600)
    def save_points(self,points):
        self.config.setdefault('waypoints',{})[str(self.current)]=points;self.persist();self.refresh_detail()
    def load_points(self):
        self.waypoint_editor.set_aircraft(self.current,self.points(self.current));self.refresh_detail()
    def switch(self):
        self.current=self.selector.currentData();self.load_points()
    def select_aircraft(self,n):
        self.selector.setCurrentIndex(self.selector.findData(n))
    def batch(self,command):
        ids=self.selected()
        if not ids:self.write_log('未选择飞机，未提交操作。');return
        if any(self.workers[n].pending for n in ids):
            self.write_log('选中的飞机尚有操作未完成，请等待状态更新。');return
        if command=='start':
            errors=[]
            for n in ids:
                s=self.states.get(n,{})
                if time.monotonic()-self.received.get(n,0)>3 or not s.get('online',False) or not s.get('ready'):errors.append(f'{n} 号：状态未就绪')
                if not self.points(n):errors.append(f'{n} 号：未设置航点')
                if any(p['kind']=='geo' for p in self.points(n)) and not s.get('geo_ready'):errors.append(f'{n} 号：经纬度不可用，改用米制点')
            if errors:self.write_log('不能执行任务：'+'；'.join(errors));return
        if command in {'start','return','land','stop_program'}:
            detail={'start':'请求解锁并起飞，按各机航点飞行，然后规划返航并自动降落。','return':'取消后续航点，通过规划器返回各自起飞点并降落。','land':'在当前位置请求 AUTO.LAND，取消当前任务。','stop_program':'仅关闭已着陆且未解锁的飞机程序。'}[command]
            self.write_log(f"飞机 {ids}：{detail}")
        for n in ids:
            params=dict(waypoints=list(self.points(n))) if command=='start' else {}
            self.workers[n].submit(command,params)
            self.write_log(f'{n} 号 · 已提交 {command}；各机独立接受/拒绝，非同步起飞保证')
    def on_result(self,n,command,ok,text):
        self.write_log(f'{n} 号 · {command} '+('成功' if ok else '失败')+'：'+text)
    def on_state(self,n,state):
        if 'online' not in state:state['online']=True
        if not state.get('online'):
            state=dict(self.states.get(n,{}),**state,ready=False,fresh=False,geo_ready=False)
        if state.get('error') and state.get('error')!=self.states.get(n,{}).get('error'):
            self.write_log(f"{n} 号 · {state['error']}")
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
    def geo_available(self,n):
        state=self.states.get(n,{})
        return bool(time.monotonic()-self.received.get(n,0)<3 and state.get('online')
                    and state.get('geo_ready') and state.get('gps'))
    def refresh_detail(self):
        for row,a in enumerate(self.config['aircraft']):
            n=a['id']
            self.table.item(row,4).setText('经纬度可用' if self.geo_available(n) else '仅米制目标 · 持续检测')
        s=self.states.get(self.current,{});fresh=time.monotonic()-self.received.get(self.current,0)<3
        geo=self.geo_available(self.current);self.waypoint_editor.set_status(s,fresh,geo)
        self.notice.setText('经纬度和米制坐标均可设置。' if geo else '持续检测 GNSS 与坐标参考，可用后自动开放经纬度点；当前可设置米制点。')
        m=s.get('mission',{});gps=s.get('gps');parts=[]
        if not fresh:parts.append('遥测未更新，请等待连接')
        parts.extend(s.get('reasons',[]))
        if s.get('error'):parts.append(s['error'][-220:])
        if gps:parts.append(f"GNSS：{gps['latitude']:.7f}, {gps['longitude']:.7f}")
        parts.append(f"任务：{PHASES.get(m.get('phase'),m.get('phase','—'))}  |  航点 {min(m.get('index',0)+1,len(m.get('points',[])))}/{len(m.get('points',[]))}")
        if m.get('reason'):parts.append(m['reason'])
        self.details.setText('\n'.join(parts))
        for n,plot in self.map_panel.plots.items():
            plot.selected=n==self.current;plot.change(self.states.get(n,{}),self.points(n),self.traces[n])
    def closeEvent(self,event):
        active=[n for n,s in self.states.items() if s.get('mission',{}).get('active')]
        if active:self.write_log(f'关闭界面；飞机 {active} 的机上任务仍会继续执行。')
        self.timer.stop()
        for worker in self.workers.values():worker.halt.set();worker.backend.close()
        for worker in self.workers.values():
            if not worker.wait(3000):event.ignore();self.timer.start();return
        event.accept()

"""Native PyQt ground station. Each worker owns exactly one SSH connection."""
import json
import os
from collections import deque
from pathlib import Path
import queue
import threading
import time
from PyQt5 import QtCore, QtWidgets as W
from .transport import Remote
from .demo import Demo
from .view_widgets import AllCheckBox, MapPanel, SelectCheckBox, StatusTable
from .planner_view import PlannerView
from .waypoint_editor import WaypointEditor
from .site_widget import SiteWidget
from .site import build_plan
from .frame import reference, from_map, display

PHASES={'IDLE':'待命','ARMING':'解锁中','TAKEOFF':'起飞中','OUTBOUND':'前往航点','RETURNING':'规划返航',
        'LAND_REQUESTED':'请求降落','LANDING':'降落中','COMPLETE':'任务完成','ERROR':'异常','MANUAL':'外部接管',
        'STOPPED':'程序已停止','NO_AGENT':'待启动观测代理'}


class Worker(QtCore.QThread):
    state=QtCore.pyqtSignal(int,dict)
    result=QtCore.pyqtSignal(int,str,bool,str)
    observation=QtCore.pyqtSignal(int,dict)
    def __init__(self,config,demo):
        super().__init__();self.n=config['id'];self.backend=Demo(config) if demo else Remote(config)
        self.commands=queue.Queue();self.halt=threading.Event();self.pending=False;self.observe_enabled=False
    def submit(self,command,params=None):
        if self.pending or not self.isRunning():return False
        self.pending=True;self.commands.put((command,params or {}));return True
    def run(self):
        last_poll=0;last_observe=0;last_stream_try=0;last_observe_stream_try=0;latest={}
        status_stream=None;observe_stream=None
        try:
            while not self.halt.is_set():
                try:command,params=self.commands.get(timeout=.025)
                except queue.Empty:command=None
                if command:
                    try:
                        state=self.backend.execute(command,**params)
                        state.setdefault('program',command!='stop_program')
                        state['online']=True;latest=state
                        self.state.emit(self.n,state);self.result.emit(self.n,command,True,'完成')
                    except Exception as e:self.result.emit(self.n,command,False,str(e))
                    finally:self.pending=False
                    if command in {'start_program','stop_program'}:
                        if status_stream:status_stream.close();status_stream=None
                        if observe_stream:observe_stream.close();observe_stream=None
                        last_stream_try=0
                    last_poll=0
                if (isinstance(self.backend,Remote) and status_stream is None and
                        (not latest or latest.get('program')) and time.monotonic()-last_stream_try>=2):
                    last_stream_try=time.monotonic()
                    try:
                        status_stream=self.backend.open_stream('status',.2)
                        last_poll=time.monotonic()
                    except Exception:pass
                if status_stream:
                    try:
                        replies=status_stream.poll()
                        for reply in replies:
                            if not reply.get('ok'):raise RuntimeError(reply.get('error','状态流异常'))
                            state=dict(reply['status'],program=True,online=True)
                            latest=state;self.state.emit(self.n,state)
                            last_poll=time.monotonic()
                    except Exception:
                        status_stream.close();status_stream=None
                if status_stream and time.monotonic()-last_poll>1.5:
                    status_stream.close();status_stream=None
                if status_stream is None and time.monotonic()-last_poll>.7 and not self.halt.is_set():
                    try:
                        state=self.backend.status();state['online']=True
                        latest=state;self.state.emit(self.n,state)
                    except Exception as e:
                        self.backend.close()
                        latest['online']=False
                        self.state.emit(self.n,dict(online=False,error=str(e)))
                        last_poll=time.monotonic()+1.3
                    else:last_poll=time.monotonic()
                observing=(self.observe_enabled and latest.get('online') and latest.get('program') and
                           'planner-view-v1' in latest.get('capabilities',[]))
                if not observing and observe_stream:
                    observe_stream.close();observe_stream=None
                if (observing and isinstance(self.backend,Remote) and observe_stream is None and
                        time.monotonic()-last_observe_stream_try>=2):
                    last_observe_stream_try=time.monotonic()
                    try:observe_stream=self.backend.open_stream('observe',.4)
                    except Exception:pass
                if observe_stream:
                    try:
                        for reply in observe_stream.poll():
                            if not reply.get('ok'):raise RuntimeError(reply.get('error','规划观察流异常'))
                            self.observation.emit(self.n,reply['observation'])
                            last_observe=time.monotonic()
                    except Exception as e:
                        observe_stream.close();observe_stream=None
                        self.observation.emit(self.n,dict(error=str(e)))
                elif observing and not isinstance(self.backend,Remote) and time.monotonic()-last_observe>.4:
                    try:self.observation.emit(self.n,self.backend.observe())
                    except Exception as e:self.observation.emit(self.n,dict(error=str(e)))
                    last_observe=time.monotonic()
        finally:
            if status_stream:status_stream.close()
            if observe_stream:observe_stream.close()
            self.backend.close()


class Window(W.QMainWindow):
    def __init__(self,config,path,demo=False):
        super().__init__();self.config=config;self.path=Path(path);self.demo=demo
        self.states={};self.traces={a['id']:[] for a in config['aircraft']};self.current=config['aircraft'][0]['id']
        self.workers={};self.event_seen={};self.loading=False;self.received={};self.rate_samples={};self.plans={};self.plan_keys={};self.frames={}
        self.setWindowTitle('Fast Drone · 独立单机地面站'+(' [模拟演示]' if demo else ''))
        screen=W.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1680,int(screen.width()*.9)),min(1080,int(screen.height()*.9)))
        self.setMinimumSize(720,540)
        central=W.QWidget();self.setCentralWidget(central);layout=W.QVBoxLayout(central);layout.setContentsMargins(16,12,16,12);layout.setSpacing(10)
        top=W.QHBoxLayout();title=W.QLabel('智控凌云队飞行地面站');title.setObjectName('title');top.addWidget(title);top.addStretch();layout.addLayout(top)
        self.main_splitter=W.QSplitter(QtCore.Qt.Vertical);self.main_splitter.setChildrenCollapsible(False)
        self.sections_adjusted=False;self.main_splitter.splitterMoved.connect(self.mark_sections_adjusted)
        layout.addWidget(self.main_splitter,1)
        status_panel=W.QWidget();status_layout=W.QVBoxLayout(status_panel);status_layout.setContentsMargins(0,0,0,0);status_layout.setSpacing(6)
        selection=W.QHBoxLayout();self.select_all=AllCheckBox('全选');self.select_all.setTristate(True)
        self.select_all.stateChanged.connect(self.set_all_selected);selection.addWidget(self.select_all)
        self.selection_count=W.QLabel();self.selection_count.setObjectName('muted');selection.addWidget(self.selection_count);selection.addStretch();status_layout.addLayout(selection)
        self.checkboxes={};self.connect_buttons={}
        self.table=StatusTable(len(config['aircraft']))
        self.table.setHorizontalHeaderLabels(['选择','飞机 / SSH','程序 / 链路','本机定位','GNSS','电量','飞行模式 / 解锁','局部位置 x / y / z','任务','SSH 连接'])
        self.table.verticalHeader().hide();self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.table.setMinimumHeight(85);self.table.setSizePolicy(W.QSizePolicy.Expanding,W.QSizePolicy.Expanding)
        for row,a in enumerate(config['aircraft']):
            box=SelectCheckBox();box.setChecked(True);box.setAccessibleName(f"选择 {a['id']} 号机")
            box.stateChanged.connect(self.update_selection);self.checkboxes[a['id']]=box
            cell=W.QWidget();cell_layout=W.QHBoxLayout(cell);cell_layout.setContentsMargins(0,0,0,0);cell_layout.setAlignment(QtCore.Qt.AlignCenter);cell_layout.addWidget(box)
            self.table.setCellWidget(row,0,cell)
            for col in range(1,9):
                item=W.QTableWidgetItem('—');item.setTextAlignment(QtCore.Qt.AlignCenter if col!=1 else QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter);self.table.setItem(row,col,item)
            self.table.item(row,1).setText(f"{a['id']} 号  {a['host']}");self.table.setRowHeight(row,44)
            worker=Worker(a,demo);worker.state.connect(self.on_state);worker.result.connect(self.on_result);self.workers[a['id']]=worker
            worker.observation.connect(self.on_observation)
            n=a['id'];button=W.QPushButton('连接 SSH');button.setStyleSheet('padding:6px 10px')
            button.clicked.connect(lambda checked=False,n=n:self.connect_aircraft(n));self.connect_buttons[n]=button;self.table.setCellWidget(row,9,button)
            self.table.item(row,2).setText('未连接')
            worker.finished.connect(lambda n=n:self.connection_finished(n))
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
        editor_tabs=W.QTabWidget();ll.addWidget(editor_tabs,1)
        self.site_widget=SiteWidget(config.get('site'));editor_tabs.addTab(self.site_widget,'场地 / 航线')
        self.site_widget.changed.connect(self.site_changed);self.site_widget.search.connect(self.search_routes)
        self.notice=W.QLabel();self.notice.setWordWrap(True);ll.addWidget(self.notice)
        self.waypoint_editor=WaypointEditor();editor_tabs.insertTab(0,self.waypoint_editor,'目标点');editor_tabs.setCurrentIndex(0)
        self.waypoint_editor.pointsChanged.connect(self.save_points)
        self.waypoint_editor.logMessage.connect(self.write_log)
        text=W.QLabel('米制：搜索时位置为原点、机头为前，X 向右、Y 向前；飞行中方向固定。经纬度：WGS84。\n移动或转动飞机后请重新搜索航线。最后返回起飞点并降落，高度 1.5m。');text.setWordWrap(True);text.setObjectName('muted');ll.addWidget(text)
        self.details=W.QLabel('等待遥测');self.details.setWordWrap(True);ll.addWidget(self.details)
        editor=W.QScrollArea();editor.setWidgetResizable(True);editor.setFrameShape(W.QFrame.NoFrame);editor.setWidget(left);editor.setMinimumSize(350,150)
        splitter.addWidget(editor)
        right=W.QWidget();rl=W.QVBoxLayout(right);rl.setContentsMargins(4,0,0,0)
        self.view_tabs=W.QTabWidget();rl.addWidget(self.view_tabs,1)
        self.map_panel=MapPanel([a['id'] for a in config['aircraft']]);self.view_tabs.addTab(self.map_panel,'航线总览')
        self.planner_view=PlannerView();self.planner_view.show_aircraft(self.current)
        self.view_tabs.addTab(self.planner_view,'局部规划观察')
        self.view_tabs.currentChanged.connect(self.update_observer)
        for n,plot in self.map_panel.plots.items():plot.clicked.connect(self.select_aircraft)
        self.table.cellClicked.connect(lambda row,col:self.select_aircraft(self.config['aircraft'][row]['id']) if col else None)
        splitter.addWidget(right);splitter.setStretchFactor(0,0);splitter.setStretchFactor(1,1);splitter.setSizes([460,1060])
        self.log=W.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMinimumHeight(50);self.log.document().setMaximumBlockCount(300);self.main_splitter.addWidget(self.log)
        self.main_splitter.setSizes([335,590,90]);self.main_splitter.setStretchFactor(0,1);self.main_splitter.setStretchFactor(1,3);self.main_splitter.setStretchFactor(2,0)
        self.setStyleSheet('''QMainWindow,QWidget{background:#0b1421;color:#dce7f5;font-family:"Noto Sans CJK SC","DejaVu Sans";font-size:13px} QLabel#title{font-size:21px;font-weight:700} QLabel#badge{color:#39d0ca;background:#142b35;border-radius:6px;padding:9px} QLabel#muted{color:#8297af} QPushButton{background:#1b2a40;border:1px solid #2d425c;border-radius:6px;padding:10px 14px} QPushButton:hover{background:#263b55} QPushButton:disabled{color:#586777;background:#142031} QPushButton#primary{background:#1b938f;color:white;font-weight:bold} QPushButton#danger{background:#69353c;color:#ffdbdf} QTableWidget,QPlainTextEdit{background:#101d2c;alternate-background-color:#152436;border:1px solid #25374b;border-radius:5px;gridline-color:#25374b;selection-background-color:#214e63} QHeaderView::section{background:#17283c;color:#9bb2cc;padding:8px;border:0} QComboBox{background:#1b2a40;padding:6px;border:1px solid #30465f;border-radius:4px} QLineEdit{background:#142236} QCheckBox{spacing:7px} QCheckBox::indicator{width:16px;height:16px} QSplitter::handle{background:#30465f;width:5px;height:5px} QScrollArea{border:0}''')
        self.adapt_toolbar();self.load_points();self.write_log('模拟演示：2 号机无 GNSS，可用米制航点。' if demo else '启动程序不会解锁；任务在各机独立执行。')
        self.write_log('请点击各机的「连接 SSH」；仅连接你需要操作的飞机。')
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh_detail);self.timer.start(200)

    def connect_aircraft(self,n):
        worker=self.workers[n]
        if worker.isRunning():return
        worker.halt.clear();self.connect_buttons[n].setEnabled(False);self.connect_buttons[n].setText('连接中…')
        self.write_log(f'{n} 号 · 开始连接 SSH');worker.start()
    def connection_finished(self,n):
        self.connect_buttons[n].setText('重试连接');self.connect_buttons[n].setEnabled(True)
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
        desired=self.table.rowCount()*44+self.table.horizontalHeader().height()+self.toolbar.sizeHint().height()+self.select_all.sizeHint().height()+16
        top=min(desired,int(height*.48));bottom=min(90,max(50,height//12))
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
        self.plans.pop(self.current,None);self.plan_keys.pop(self.current,None)
        self.config.setdefault('waypoints',{})[str(self.current)]=points;self.persist();self.refresh_detail()
    def load_points(self):
        self.waypoint_editor.set_aircraft(self.current,self.points(self.current));self.refresh_detail()
    def switch(self):
        self.current=self.selector.currentData();self.load_points();self.update_observer()
    def update_observer(self,*args):
        viewing=self.view_tabs.currentIndex()==1
        self.planner_view.show_aircraft(self.current)
        for n,worker in self.workers.items():worker.observe_enabled=viewing and n==self.current
    def on_observation(self,n,data):
        self.planner_view.change(n,data)
    def select_aircraft(self,n):
        self.selector.setCurrentIndex(self.selector.findData(n))
    def site_changed(self):
        self.plans.clear();self.plan_keys.clear()
        try:self.config['site']=self.site_widget.value();self.persist()
        except ValueError as e:self.write_log(str(e))
        self.refresh_detail()
    def plan_key(self,n):
        return json.dumps([self.site_widget.value(),self.points(n)],sort_keys=True,allow_nan=False)
    def search_routes(self):
        ids=self.selected()
        if not ids:self.write_log('请先勾选要搜索航线的飞机。');return
        for n in ids:
            self.plans.pop(n,None);self.plan_keys.pop(n,None)
            try:
                if time.monotonic()-self.received.get(n,0)>3 or not self.states.get(n,{}).get('online'):raise ValueError('请先连接并获取新鲜定位')
                plan=build_plan(self.site_widget.value(),self.points(n),self.states[n])
                self.frames[n]=plan['flight_plan']['local_frame']
                self.plans[n]=plan;self.plan_keys[n]=self.plan_key(n)
                self.write_log(f"{n} 号 · 往返航线已生成：去程 {len(plan['waypoints'])} 点、返航 {len(plan['flight_plan']['return_points'])} 点；请核对地图后执行。")
            except (ValueError,KeyError,TypeError) as e:self.write_log(f'{n} 号 · 航线搜索失败：{e}')
        self.refresh_detail()
    def batch(self,command):
        ids=self.selected()
        if not ids:self.write_log('未选择飞机，未提交操作。');return
        disconnected=[n for n in ids if not self.workers[n].isRunning() or not self.states.get(n,{}).get('online')]
        if disconnected:
            self.write_log(f'未提交操作：飞机 {disconnected} 尚未连接，请先点击对应的连接 SSH 按钮。');return
        if any(self.workers[n].pending for n in ids):
            self.write_log('选中的飞机尚有操作未完成，请等待状态更新。');return
        if command=='start':
            errors=[]
            for n in ids:
                s=self.states.get(n,{})
                try:
                    if n not in self.plans or self.plan_keys.get(n)!=self.plan_key(n):errors.append(f'{n} 号：请先搜索并预览当前往返航线')
                except ValueError as e:errors.append(str(e))
                if time.monotonic()-self.received.get(n,0)>3 or not s.get('online',False) or not s.get('ready'):errors.append(f'{n} 号：状态未就绪')
                if not self.points(n):errors.append(f'{n} 号：未设置航点')
                if any(p['kind']=='geo' for p in self.points(n)) and not s.get('geo_ready'):errors.append(f'{n} 号：经纬度不可用，改用米制点')
            if errors:self.write_log('不能执行任务：'+'；'.join(errors));return
        if command in {'start','return','land','stop_program'}:
            detail={'start':'请求解锁并起飞，按各机航点飞行，然后规划返航并自动降落。','return':'取消后续航点，通过规划器返回各自起飞点并降落。','land':'在当前位置请求 AUTO.LAND，取消当前任务。','stop_program':'仅关闭已着陆且未解锁的飞机程序。'}[command]
            self.write_log(f"飞机 {ids}：{detail}")
        for n in ids:
            params={k:self.plans[n][k] for k in ('waypoints','flight_plan')} if command=='start' else {}
            self.workers[n].submit(command,params)
            self.write_log(f'{n} 号 · 已提交 {command}；各机独立接受/拒绝，非同步起飞保证')
    def on_result(self,n,command,ok,text):
        self.write_log(f'{n} 号 · {command} '+('成功' if ok else '失败')+'：'+text)
    def on_state(self,n,state):
        if 'online' not in state:state['online']=True
        if state.get('online'):
            self.connect_buttons[n].setText('已连接');self.connect_buttons[n].setEnabled(False)
        if not state.get('online'):
            self.connect_buttons[n].setText('自动重连中…')
            state=dict(self.states.get(n,{}),**state,ready=False,fresh=False,geo_ready=False)
        if state.get('error') and state.get('error')!=self.states.get(n,{}).get('error'):
            self.write_log(f"{n} 号 · {state['error']}")
        old_session=self.states.get(n,{}).get('session_id')
        if state.get('session_id') and old_session and old_session!=state['session_id']:
            self.frames.pop(n,None);self.plans.pop(n,None);self.plan_keys.pop(n,None);self.traces[n]=[]
            self.write_log(f'{n} 号 · 机上程序已重启，请重新搜索航线')
        self.states[n]=state;self.received[n]=time.monotonic()
        samples=self.rate_samples.setdefault(n,deque(maxlen=30))
        if state.get('online') and state.get('program'):
            samples.append(self.received[n])
        else:samples.clear()
        if state.get('program') is False:
            self.frames.pop(n,None);self.plans.pop(n,None);self.plan_keys.pop(n,None);self.traces[n]=[]
        elif state.get('mission',{}).get('active') and state['mission'].get('local_frame'):
            self.frames[n]=state['mission']['local_frame']
        elif n not in self.frames and state.get('fresh') and state.get('ready') and not state.get('armed'):
            try:self.frames[n]=reference(state)
            except ValueError:pass
        row=next(i for i,a in enumerate(self.config['aircraft']) if a['id']==n)
        p=state.get('position');mode=state.get('mode','—');phase=state.get('mission',{}).get('phase','—')
        values=['在线' if state.get('online') else 'SSH 离线','就绪' if state.get('ready') else '未就绪',
                'GNSS有效' if state.get('geo_ready') else 'GNSS无效',
                f"{state['battery']:.0f}%" if state.get('battery') is not None else '—',
                mode+(' / 已解锁' if state.get('armed') else ' / 未解锁' if state.get('fresh') else ' / 未知'),
                ' / '.join(f'{v:.2f}' for v in p) if p else '—',PHASES.get(phase,phase)]
        if state.get('program') is False:values[0]='程序已停止'
        elif phase=='NO_AGENT':values[0]='待加载代理'
        for col,value in enumerate(values,2):
            self.table.item(row,col).setText(value)
            self.table.item(row,col).setToolTip(value+'\n'+state.get('error','')+'\n'+' / '.join(state.get('reasons',[])))
        if p:
            trace=self.traces[n]
            if not trace or abs(trace[-1][0]-p[0])+abs(trace[-1][1]-p[1])>.04:trace.append((p[0],p[1]))
            self.traces[n]=trace[-1500:]
        events=state.get('events',[])
        last=self.event_seen.get(n)
        if events and events[-1]!=last:
            start=next((i+1 for i in range(len(events)-1,-1,-1) if events[i]==last),0)
            for event in events[start:]:self.write_log(f"{n} 号 · {event['text']}")
            self.event_seen[n]=events[-1]
        self.refresh_detail()
    def geo_available(self,n):
        state=self.states.get(n,{})
        return bool(time.monotonic()-self.received.get(n,0)<3 and state.get('online')
                    and state.get('geo_ready') and state.get('gps'))
    def refresh_detail(self):
        self.site_widget.setEnabled(not any(s.get('armed') or s.get('mission',{}).get('active') for s in self.states.values()))
        for row,a in enumerate(self.config['aircraft']):
            n=a['id']
            self.table.item(row,4).setText('GNSS有效' if self.geo_available(n) else 'GNSS无效')
            pos=self.states.get(n,{}).get('position');ref=self.frames.get(n)
            self.table.item(row,7).setText(' / '.join(f'{v:.2f}' for v in from_map(pos,ref)) if pos and ref else '—')
            self.table.item(row,7).setToolTip('X 向右、Y 向前，相对起飞参考点；Z 为机载地图高度')
        s=self.states.get(self.current,{});fresh=time.monotonic()-self.received.get(self.current,0)<3
        geo=self.geo_available(self.current);self.waypoint_editor.set_status(s,fresh,geo)
        self.notice.setText('经纬度和米制坐标均可设置。' if geo else '持续检测 GNSS 与坐标参考，可用后自动开放经纬度点；当前可设置米制点。')
        m=s.get('mission',{});gps=s.get('gps');parts=[]
        parts.extend(s.get('reasons',[]))
        if s.get('error'):parts.append(s['error'][-220:])
        if gps:parts.append(f"GNSS：{gps['latitude']:.7f}, {gps['longitude']:.7f}")
        if m.get('reason'):parts.append(m['reason'])
        samples=self.rate_samples.get(self.current,())
        if len(samples)>1 and samples[-1]-samples[0]>0:
            parts.append(f'地面站遥测刷新：{(len(samples)-1)/(samples[-1]-samples[0]):.1f} Hz')
        if not s.get('online') and s.get('mission',{}).get('active'):
            parts.append('地面站连接已断开；机上任务继续执行，请重新连接以查看状态')
        self.details.setText('\n'.join(parts));self.details.setVisible(bool(parts))
        for n,plot in self.map_panel.plots.items():
            state,points,trace,preview=display(self.states.get(n,{}),self.points(n),self.traces[n],self.plans.get(n,{}).get('preview'),self.frames.get(n))
            plot.preview=preview
            plot.selected=n==self.current;plot.change(state,points,trace)
    def closeEvent(self,event):
        active=[n for n,s in self.states.items() if s.get('mission',{}).get('active')]
        if active:self.write_log(f'关闭界面；飞机 {active} 的机上任务仍会继续执行。')
        self.timer.stop()
        for worker in self.workers.values():worker.observe_enabled=False
        for worker in self.workers.values():worker.halt.set();worker.backend.close()
        for worker in self.workers.values():
            if not worker.wait(3000):event.ignore();self.timer.start();return
        event.accept()

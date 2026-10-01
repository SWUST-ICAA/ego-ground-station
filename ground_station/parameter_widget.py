"""Single-aircraft parameter editor; incoming telemetry never replaces edits."""
from PyQt5 import QtCore, QtWidgets as W
from onboard.flight_parameters import validate, voxel_count

FIELDS = [('map_size_x','地图 X 尺寸','m'),('map_size_y','地图 Y 尺寸','m'),
          ('map_size_z','地图 Z 尺寸','m'),('ground_height','地面高度（地图下沿）','m'),
          ('obstacles_inflation','障碍膨胀距离','m'),('resolution','地图分辨率','m'),
          ('max_acc','最大加速度','m/s²'),('max_vel','最大速度','m/s'),
          ('takeoff_height','起飞 / 目标点高度','m')]


class ParameterWidget(W.QWidget):
    readRequested=QtCore.pyqtSignal()
    applyRequested=QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__();self.aircraft=None;self.loaded=False;self.supported=False;self.program=False
        layout=W.QVBoxLayout(self);layout.setContentsMargins(18,18,18,18)
        self.title=W.QLabel();layout.addWidget(self.title)
        self.info=W.QLabel('请连接飞机后读取参数');self.info.setWordWrap(True);layout.addWidget(self.info)
        form=W.QFormLayout();self.fields={}
        for key,label,unit in FIELDS:
            spin=W.QDoubleSpinBox();spin.setDecimals(3);spin.setRange(-10000 if key=='ground_height' else .001,100000)
            spin.setSingleStep(.1 if key!='resolution' else .02);spin.setSuffix(' '+unit)
            spin.valueChanged.connect(self.describe);spin.setEnabled(False);self.fields[key]=spin;form.addRow(label,spin)
        layout.addLayout(form)
        self.summary=W.QLabel();self.summary.setWordWrap(True);layout.addWidget(self.summary)
        actions=W.QHBoxLayout();self.read_button=W.QPushButton('读取机上参数')
        self.apply_button=W.QPushButton('保存（下次启动生效）');self.apply_button.setObjectName('primary')
        self.read_button.clicked.connect(self.readRequested.emit);self.apply_button.clicked.connect(self.apply)
        actions.addWidget(self.read_button);actions.addWidget(self.apply_button);layout.addLayout(actions)
        note=W.QLabel('高度使用机载地图坐标。起飞、去程及返航目标使用同一高度。\n保存运行中的参数会重启机上程序；必须已着陆、未解锁且任务结束。修改后请重新搜索航线。')
        note.setWordWrap(True);note.setObjectName('muted');layout.addWidget(note);layout.addStretch()
        self.show_aircraft(None)

    def show_aircraft(self,n):
        if self.aircraft==n and n is not None:return
        self.aircraft=n;self.loaded=False;self.supported=False
        self.title.setText(f'{n} 号机 · 参数设置' if n is not None else '参数设置')
        self.info.setText('请连接飞机后读取参数');self.summary.clear()
        for field in self.fields.values():field.setEnabled(False);field.clear()
        self.read_button.setEnabled(False);self.apply_button.setEnabled(False)

    def receive(self,n,data):
        if n!=self.aircraft:return
        self.loaded=False
        for key,field in self.fields.items():
            blocker=QtCore.QSignalBlocker(field);field.setValue(data['values'][key]);del blocker
        self.loaded=True;self.supported=data['supported'];self.program=data['program']
        self.info.setText(data['source']+('' if self.supported else '；此飞机尚未部署参数功能'))
        self.describe()

    def values(self):return {key:field.value() for key,field in self.fields.items()}

    def describe(self,*args):
        if not self.loaded:return
        values=self.values()
        try:
            p=validate(values);size=voxel_count(p)
            self.summary.setText('地图高度 %.3f～%.3f m；%s 个网格，基础缓冲区约 %.0f MB。'%(p['ground_height'],p['ground_height']+p['map_size_z'],format(size,','),size*15/1e6))
        except ValueError as e:self.summary.setText(str(e))

    def set_status(self,state,fresh,pending):
        connected=fresh and state.get('online',False)
        safe=not state.get('program') or (state.get('fresh') and state.get('landed') and not state.get('armed') and not state.get('mission',{}).get('active'))
        self.read_button.setEnabled(connected and not pending)
        for field in self.fields.values():field.setEnabled(self.loaded and self.supported and connected and not pending)
        try:validate(self.values());valid=True
        except ValueError:valid=False
        self.apply_button.setText('保存并应用（重启程序）' if state.get('program') else '保存（下次启动生效）')
        self.apply_button.setEnabled(self.loaded and self.supported and connected and safe and valid and not pending)

    def apply(self):
        try:values=validate(self.values())
        except ValueError as e:self.info.setText(str(e));return
        self.applyRequested.emit(values)

"""Explicit competition/test profile editing; does not send aircraft commands."""
import copy
from PyQt5 import QtCore,QtWidgets as W
from .site import DEFAULT


class SiteWidget(W.QGroupBox):
    changed=QtCore.pyqtSignal()
    search=QtCore.pyqtSignal()
    def __init__(self,profile):
        super().__init__('场地边界与航线')
        self.profile=copy.deepcopy(profile or DEFAULT)
        layout=W.QVBoxLayout(self);layout.setSpacing(6)
        row=W.QHBoxLayout();self.mode=W.QComboBox();self.mode.addItem('测试 · 本机米制场地','test');self.mode.addItem('比赛 · 科目三','competition')
        self.mode.setCurrentIndex(1 if self.profile['mode']=='competition' else 0);row.addWidget(self.mode,1)
        self.margin=W.QDoubleSpinBox();self.margin.setRange(.75,30);self.margin.setDecimals(2);self.margin.setValue(self.profile['margin']);self.margin.setSuffix(' m');self.margin.setPrefix('内缩 ');row.addWidget(self.margin);layout.addLayout(row)
        self.boundary=W.QPlainTextEdit();self.boundary.setMaximumHeight(85);self.boundary.setPlaceholderText('边界顶点依次填写，每行 X,Y（米）；不重复首点')
        self.boundary.setPlainText('\n'.join(f'{x:g},{y:g}' for x,y in self.profile['polygon']));layout.addWidget(self.boundary)
        self.confirm=W.QCheckBox('已向主办方确认边界坐标为 WGS84');self.confirm.setChecked(self.profile.get('datum_confirmed',False));layout.addWidget(self.confirm)
        hint=W.QLabel('测试边界：以搜索时飞机位置为原点、机头为前，X 向右、Y 向前；每行 X,Y，按周界顺序。内缩包含机体与定位/制动余量；默认 3m，搜索额外预留 0.5m。');hint.setWordWrap(True);layout.addWidget(hint)
        self.button=W.QPushButton('搜索并预览勾选飞机的往返航线');self.button.clicked.connect(self.search);layout.addWidget(self.button);layout.addStretch()
        self.mode.currentIndexChanged.connect(self.on_change);self.margin.valueChanged.connect(self.on_change);self.boundary.textChanged.connect(self.on_change);self.confirm.toggled.connect(self.on_change)
        self.update_mode()
    def update_mode(self):
        competition=self.mode.currentData()=='competition';self.boundary.setVisible(not competition);self.confirm.setVisible(competition)
    def on_change(self,*args):self.update_mode();self.changed.emit()
    def value(self):
        polygon=[]
        for line in self.boundary.toPlainText().splitlines():
            if not line.strip():continue
            try:x,y=map(float,line.replace('，',',').split(','))
            except ValueError:raise ValueError('测试边界格式应为每行两个数字：X,Y')
            polygon.append([x,y])
        return dict(mode=self.mode.currentData(),margin=self.margin.value(),polygon=polygon,datum_confirmed=self.confirm.isChecked())

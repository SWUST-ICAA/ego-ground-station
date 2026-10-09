"""Explicit competition/test profile editing; does not send aircraft commands."""
import copy
from PyQt5 import QtCore,QtWidgets as W
from .site import DEFAULT


class SiteWidget(W.QGroupBox):
    changed=QtCore.pyqtSignal()
    search=QtCore.pyqtSignal()
    def __init__(self,profile):
        super().__init__('场地边界与航线')
        self.setStyleSheet('''
            QGroupBox {
                font-weight: 600;
                color: #1F2937;
                border: 1px solid #E5E7EB;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 18px;
                background: #FFFFFF;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px;
                background: #FFFFFF;
            }
            QComboBox {
                padding: 8px 12px;
                border: 1px solid #D1D5DB;
                border-radius: 6px;
                background: #FFFFFF;
                color: #374151;
            }
            QComboBox:focus {
                border: 2px solid #2563EB;
            }
            QDoubleSpinBox {
                padding: 8px 12px;
                border: 1px solid #D1D5DB;
                border-radius: 6px;
                background: #FFFFFF;
                color: #374151;
            }
            QDoubleSpinBox:focus {
                border: 2px solid #2563EB;
            }
            QPlainTextEdit {
                padding: 10px;
                border: 1px solid #D1D5DB;
                border-radius: 6px;
                background: #FFFFFF;
                color: #374151;
                selection-background-color: #BFDBFE;
                selection-color: #111827;
                line-height: 1.5;
            }
            QPlainTextEdit:focus {
                border: 2px solid #2563EB;
            }
            QPushButton {
                padding: 10px 20px;
                background: #2563EB;
                color: #FFFFFF;
                border: none;
                border-radius: 6px;
                font-weight: 500;
            }
            QPushButton:hover {
                background: #1D4ED8;
            }
            QCheckBox {
                color: #374151;
                spacing: 8px;
            }
            QLabel#hint {
                color: #6B7280;
                background: #F9FAFB;
                padding: 12px;
                border-radius: 6px;
                border-left: 3px solid #93C5FD;
                line-height: 1.6;
            }
            QLabel#status {
                background: #F0F9FF;
                color: #0369A1;
                padding: 8px 12px;
                border-radius: 4px;
                font-size: 13px;
            }
        ''')
        self.profile=copy.deepcopy(profile or DEFAULT)
        layout=W.QVBoxLayout(self);layout.setSpacing(12);layout.setContentsMargins(16,24,16,16)

        # 标题行：添加帮助按钮
        header=W.QHBoxLayout();header.setSpacing(8)
        title=W.QLabel('测试边界遍历');title.setStyleSheet('font-weight:600;font-size:14px;color:#1F2937;')
        header.addWidget(title)
        self.help_btn=W.QPushButton('?');self.help_btn.setFixedSize(24,24)
        self.help_btn.setStyleSheet('''
            QPushButton {
                background: #2563EB;
                color: #FFFFFF;
                border: none;
                border-radius: 12px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background: #1D4ED8;
            }
        ''')
        self.help_btn.setToolTip('点击查看详细说明')
        self.help_btn.clicked.connect(self.toggle_help)
        header.addWidget(self.help_btn);header.addStretch()
        layout.addLayout(header)

        # 可折叠的帮助说明
        self.help_panel=W.QLabel('''<b>📖 使用说明</b><br><br>
<b>坐标系统：</b><br>
• 以搜索时飞机位置为原点<br>
• 机头方向为前（Y+），右侧为右（X+）<br>
• 单位：米<br><br>
<b>输入格式：</b><br>
• 每行一个顶点：X,Y<br>
• 按周界顺序填写<br>
• 例如：-20,-20 表示左后方20米<br><br>
<b>内缩余量：</b><br>
• 包含机体定位和制动距离<br>
• 默认值：3m<br>
• 搜索时额外预留：0.5m''')
        self.help_panel.setObjectName('hint')
        self.help_panel.setWordWrap(True)
        self.help_panel.setVisible(False)
        layout.addWidget(self.help_panel)

        row=W.QHBoxLayout();row.setSpacing(12)
        self.mode=W.QComboBox();self.mode.addItem('测试 · 本机米制场地','test');self.mode.addItem('比赛 · 科目三','competition')
        self.mode.setCurrentIndex(1 if self.profile['mode']=='competition' else 0);row.addWidget(self.mode,1)
        self.margin=W.QDoubleSpinBox();self.margin.setRange(.75,30);self.margin.setDecimals(2);self.margin.setValue(self.profile['margin']);self.margin.setSuffix(' m');self.margin.setPrefix('内缩 ');row.addWidget(self.margin);layout.addLayout(row)
        self.boundary=W.QPlainTextEdit();self.boundary.setMaximumHeight(100);self.boundary.setPlaceholderText('每行输入一个顶点坐标，格式：X,Y\n例如：\n-20,-20\n20,-20\n20,20\n-20,20')
        self.boundary.setPlainText('\n'.join(f'{x:g},{y:g}' for x,y in self.profile['polygon']));layout.addWidget(self.boundary)
        self.confirm=W.QCheckBox('已向主办方确认边界坐标为 WGS84');self.confirm.setChecked(self.profile.get('datum_confirmed',False));layout.addWidget(self.confirm)

        # 底部状态栏
        self.status=W.QLabel('💡 输入顶点坐标后点击「搜索并预览」生成航线')
        self.status.setObjectName('status');self.status.setWordWrap(True);layout.addWidget(self.status)

        self.button=W.QPushButton('搜索并预览勾选飞机的往返航线');self.button.clicked.connect(self.search);layout.addWidget(self.button);layout.addStretch()
        self.mode.currentIndexChanged.connect(self.on_change);self.margin.valueChanged.connect(self.on_change);self.boundary.textChanged.connect(self.on_change);self.confirm.toggled.connect(self.on_change)
        self.update_mode()

    def toggle_help(self):
        self.help_panel.setVisible(not self.help_panel.isVisible())

    def update_mode(self):
        competition=self.mode.currentData()=='competition';self.boundary.setVisible(not competition);self.confirm.setVisible(competition)
        # 更新状态栏提示
        if competition:
            self.status.setText('💡 比赛模式：使用科目三标准场地')
        else:
            self.status.setText('💡 输入顶点坐标后点击「搜索并预览」生成航线')

    def on_change(self,*args):self.update_mode();self.changed.emit()

    def value(self):
        polygon=[]
        for line in self.boundary.toPlainText().splitlines():
            if not line.strip():continue
            try:x,y=map(float,line.replace('，',',').split(','))
            except ValueError:
                self.status.setText('❌ 测试边界格式应为每行两个数字：X,Y')
                raise ValueError('测试边界格式应为每行两个数字：X,Y')
            polygon.append([x,y])
        if polygon:
            self.status.setText(f'✅ 已输入 {len(polygon)} 个顶点')
        return dict(mode=self.mode.currentData(),margin=self.margin.value(),polygon=polygon,datum_confirmed=self.confirm.isChecked())

"""Explicit waypoint editing, with independent drafts for each aircraft."""
import math
from PyQt5 import QtCore, QtWidgets as W


class WaypointEditor(W.QWidget):
    pointsChanged=QtCore.pyqtSignal(list)
    logMessage=QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setStyleSheet('''
            QPushButton {
                padding: 8px 16px;
                border: 1px solid #D1D5DB;
                border-radius: 6px;
                background: #FFFFFF;
                color: #374151;
                font-weight: 500;
            }
            QPushButton:hover {
                background: #F9FAFB;
                border-color: #9CA3AF;
            }
            QPushButton:disabled {
                background: #F3F4F6;
                color: #9CA3AF;
                border-color: #E5E7EB;
            }
            QPushButton#primary {
                background: #2563EB;
                color: #FFFFFF;
                border-color: #2563EB;
            }
            QPushButton#primary:hover {
                background: #1D4ED8;
            }
            QPushButton#danger {
                background: #DC2626;
                color: #FFFFFF;
                border-color: #DC2626;
            }
            QPushButton#danger:hover {
                background: #B91C1C;
            }
            QLineEdit {
                padding: 8px 12px;
                border: 1px solid #D1D5DB;
                border-radius: 6px;
                background: #FFFFFF;
                color: #1F2937;
            }
            QLineEdit:focus {
                border: 2px solid #2563EB;
                padding: 7px 11px;
            }
            QTableWidget {
                background: #FFFFFF;
                border: 1px solid #E5E7EB;
                border-radius: 6px;
                gridline-color: #E5E7EB;
            }
            QTableWidget::item {
                padding: 8px;
                color: #374151;
            }
            QTableWidget::item:selected {
                background: #2563EB;
                color: #FFFFFF;
            }
            QHeaderView::section {
                background: #EFF6FF;
                color: #1E40AF;
                padding: 10px 8px;
                border: none;
                border-bottom: 2px solid #BFDBFE;
                font-weight: 600;
            }
            QLabel#hint {
                color: #6B7280;
                background: #F9FAFB;
                padding: 10px 12px;
                border-radius: 6px;
                border-left: 3px solid #93C5FD;
                line-height: 1.6;
            }
            QLabel#status {
                color: #6B7280;
                background: #F9FAFB;
                padding: 8px 12px;
                border-top: 1px solid #E5E7EB;
                font-size: 12px;
            }
            QPushButton#help {
                padding: 4px 8px;
                background: transparent;
                border: 1px solid #D1D5DB;
                border-radius: 4px;
                color: #6B7280;
                font-size: 16px;
                font-weight: bold;
                min-width: 28px;
                max-width: 28px;
                min-height: 28px;
                max-height: 28px;
            }
            QPushButton#help:hover {
                background: #F3F4F6;
                color: #374151;
            }
            QFrame#helpPanel {
                background: #F0F9FF;
                border: 1px solid #BFDBFE;
                border-radius: 6px;
            }
        ''')
        self.aircraft=None;self.points=[];self.drafts={};self.state={};self.geo_ready=False;self.fresh=False
        layout=W.QVBoxLayout(self);layout.setContentsMargins(12,12,12,12);layout.setSpacing(12)

        # Title with help button
        title_row=W.QHBoxLayout();title_row.setSpacing(8)
        title_label=W.QLabel('航点编辑');title_label.setStyleSheet('font-weight:600;font-size:14px;color:#1F2937')
        self.help_btn=W.QPushButton('?');self.help_btn.setObjectName('help');self.help_btn.setToolTip('点击查看使用说明')
        self.help_btn.clicked.connect(self.toggle_help)
        title_row.addWidget(title_label);title_row.addWidget(self.help_btn);title_row.addStretch()
        layout.addLayout(title_row)

        # Collapsible help panel
        self.help_panel=W.QFrame();self.help_panel.setObjectName('helpPanel');self.help_panel.setVisible(False)
        help_layout=W.QVBoxLayout(self.help_panel);help_layout.setSpacing(8);help_layout.setContentsMargins(12,12,12,12)
        help_title=W.QLabel('💡 使用说明');help_title.setStyleSheet('font-weight:600;color:#1E40AF;font-size:13px')
        help_layout.addWidget(help_title)
        help_text=W.QLabel('''<b>坐标系统：</b><br>
• 米制：搜索时以飞机位置为原点，机头为前（Y+），X向右；飞行中方向固定<br>
• 经纬度：WGS84坐标系统<br><br>
<b>输入格式：</b><br>
• 米制：输入相对位置的X、Y坐标（米）<br>
• 经纬度：纬度 -90～90°，经度 -180～180°<br><br>
<b>操作说明：</b><br>
• 点击「添加航点」将新坐标加入列表<br>
• 选中表格中的行后可修改坐标，点击「保存修改」<br>
• 使用「上移」「下移」调整航点顺序<br>
• 点击「删除」移除选中的航点<br><br>
<b>⚠️ 注意事项：</b><br>
• 移动或转动飞机后请重新搜索航线<br>
• 最后返回起飞点并降落，高度以机上参数设置为准''')
        help_text.setWordWrap(True);help_text.setStyleSheet('color:#374151;line-height:1.5;font-size:12px')
        help_layout.addWidget(help_text)
        self.note=W.QLabel();self.note.setWordWrap(True);self.note.setStyleSheet('color:#1E40AF;font-size:12px')
        help_layout.addWidget(self.note)
        layout.addWidget(self.help_panel)
        self.kind=W.QComboBox();self.kind.addItem('米制坐标 · X 右 / Y 前','local');self.kind.addItem('经纬度 · WGS84','geo')
        self.kind.setStyleSheet('QComboBox{padding:8px 12px;border:1px solid #D1D5DB;border-radius:6px;background:#FFFFFF}')
        layout.addWidget(self.kind)
        form=W.QFormLayout();self.a_label=W.QLabel();self.b_label=W.QLabel()
        self.a=W.QLineEdit();self.b=W.QLineEdit()
        for field in (self.a,self.b):field.setMinimumWidth(150);field.setClearButtonEnabled(True)
        form.addRow(self.a_label,self.a);form.addRow(self.b_label,self.b);layout.addLayout(form)
        actions=W.QHBoxLayout();actions.setSpacing(12)
        self.add=W.QPushButton('添加航点');self.add.setObjectName('primary')
        self.apply=W.QPushButton('保存修改')
        actions.addWidget(self.add);actions.addWidget(self.apply);actions.addStretch();layout.addLayout(actions)
        self.add.clicked.connect(lambda:self.commit(False));self.apply.clicked.connect(lambda:self.commit(True))
        # Remove old verbose message, will add status bar at bottom instead
        self.table=W.QTableWidget(0,3);self.table.setHorizontalHeaderLabels(['类型','X / 纬度','Y / 经度'])
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setSelectionMode(W.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers);self.table.setMinimumHeight(150)
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet('QTableWidget{alternate-background-color:#F9FAFB}')
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.ResizeToContents)
        layout.addWidget(self.table,1)
        order=W.QHBoxLayout();order.setSpacing(12)
        self.up=W.QPushButton('上移');self.down=W.QPushButton('下移')
        self.delete=W.QPushButton('删除');self.delete.setObjectName('danger')
        for button in (self.up,self.down):order.addWidget(button)
        order.addStretch();order.addWidget(self.delete)
        layout.addLayout(order)
        self.up.clicked.connect(lambda:self.move(-1));self.down.clicked.connect(lambda:self.move(1));self.delete.clicked.connect(self.remove)
        self.table.itemSelectionChanged.connect(self.select_point);self.kind.currentIndexChanged.connect(self.change_kind)
        self.a.textEdited.connect(self.mark_draft);self.b.textEdited.connect(self.mark_draft)
        # Status bar at bottom
        self.status=W.QLabel('💡 输入坐标后点击「添加航点」，选中行可编辑');self.status.setObjectName('status')
        layout.addWidget(self.status)
        self.change_kind()

    def toggle_help(self):
        self.help_panel.setVisible(not self.help_panel.isVisible())

    def set_note(self,text):
        if self.note.text()!=text:self.note.setText(text)

    def mark_draft(self,*args):self.status.setText('⚠️ 输入尚未保存，请点击「添加航点」或「保存修改」')

    def change_kind(self,*args):
        geo=self.kind.currentData()=='geo'
        self.a_label.setText('纬度 (°)' if geo else 'X 向右 (m)');self.b_label.setText('经度 (°)' if geo else 'Y 向前 (m)')
        self.a.setPlaceholderText('-90 ～ 90' if geo else '相对起飞点，单位米')
        self.b.setPlaceholderText('-180 ～ 180' if geo else '相对起飞点，单位米')
        self.a.clear();self.b.clear();self.update_actions()

    def set_aircraft(self,n,points):
        if self.aircraft is not None:
            self.drafts[self.aircraft]=(self.kind.currentIndex(),self.a.text(),self.b.text(),self.table.currentRow(),self.status.text())
        self.aircraft=n;self.points=[dict(p) for p in points]
        kind,a,b,row,status_msg=self.drafts.get(n,(0,'','',-1,'💡 输入坐标后点击「添加航点」，选中行可编辑'))
        self.render(row)
        self.kind.setCurrentIndex(kind);self.change_kind();self.a.setText(a);self.b.setText(b);self.status.setText(status_msg)
        self.update_actions()

    def set_status(self,state,fresh,geo_ready):
        self.state=state;self.fresh=bool(fresh and state.get('online'));self.geo_ready=geo_ready
        self.update_actions()

    def update_actions(self):
        geo=self.kind.currentData()=='geo';enabled=not geo or self.geo_ready
        row=self.table.currentRow();selected=0<=row<len(self.points)
        self.add.setEnabled(enabled);self.apply.setEnabled(enabled and selected)
        self.up.setEnabled(selected and row>0);self.down.setEnabled(selected and row<len(self.points)-1);self.delete.setEnabled(selected)
        hint='' if enabled else 'GNSS 与坐标参考暂不可用，恢复后自动启用。'
        self.add.setToolTip(hint);self.apply.setToolTip(hint)

    def commit(self,replace):
        kind=self.kind.currentData();row=self.table.currentRow()
        if kind=='geo' and not self.geo_ready:return
        if not replace and len(self.points)>=50:
            self.status.setText('❌ 最多设置 50 个航点');self.logMessage.emit(f'{self.aircraft} 号 · 最多设置 50 个航点。');return
        if replace and not 0<=row<len(self.points):return
        try:
            a,b=float(self.a.text()),float(self.b.text())
            if not all(math.isfinite(v) for v in (a,b)):raise ValueError('请输入有限数值。')
            if kind=='geo' and not (-90<=a<=90 and -180<=b<=180):raise ValueError('纬度应在 ±90°，经度应在 ±180° 内。')
        except ValueError as e:
            msg='坐标无效：'+(str(e) if 'could not convert' not in str(e) else '请完整填写两个数字。')
            self.status.setText('❌ '+msg);self.logMessage.emit(f'{self.aircraft} 号 · '+msg);return
        point=dict(kind=kind,a=a,b=b)
        if kind=='local':point['frame']='right-forward'
        if replace:self.points[row]=point
        else:self.points.append(point);row=len(self.points)-1
        self.publish(row,'✅ 已保存到本机航点列表，执行顺序从上到下')

    def render(self,row=-1):
        blocker=QtCore.QSignalBlocker(self.table);self.table.setRowCount(len(self.points))
        for i,p in enumerate(self.points):
            geo=p['kind']=='geo';digits=7 if geo else 3
            for col,text in enumerate(('经纬度' if geo else '米制' if p.get('frame')=='right-forward' else '旧map·需重存',f"{p['a']:.{digits}f}",f"{p['b']:.{digits}f}")):
                item=W.QTableWidgetItem(text);item.setToolTip(str(p['a' if col==1 else 'b']) if col else text);self.table.setItem(i,col,item)
        self.table.clearSelection();self.table.setCurrentCell(-1,-1)
        if 0<=row<len(self.points):self.table.selectRow(row)
        del blocker
        self.update_actions()

    def select_point(self):
        row=self.table.currentRow()
        if not 0<=row<len(self.points):self.update_actions();return
        p=self.points[row];self.kind.setCurrentIndex(1 if p['kind']=='geo' else 0)
        self.a.setText(str(p['a']));self.b.setText(str(p['b']));self.status.setText(f'📝 正在编辑第 {row+1} 个航点，修改后点击「保存修改」');self.update_actions()

    def publish(self,row,message):
        self.render(row);self.select_point();self.status.setText(message)
        self.pointsChanged.emit([dict(p) for p in self.points]);self.logMessage.emit(f'{self.aircraft} 号 · '+message)

    def move(self,delta):
        row=self.table.currentRow();target=row+delta
        if not (0<=row<len(self.points) and 0<=target<len(self.points)):return
        self.points[row],self.points[target]=self.points[target],self.points[row]
        self.publish(target,'✅ 航点顺序已保存')

    def remove(self):
        row=self.table.currentRow()
        if not 0<=row<len(self.points):return
        self.points.pop(row);self.publish(min(row,len(self.points)-1),'✅ 航点已删除')

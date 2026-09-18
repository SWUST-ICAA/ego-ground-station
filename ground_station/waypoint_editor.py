"""Explicit waypoint editing, with independent drafts for each aircraft."""
import math
from PyQt5 import QtCore, QtWidgets as W


class WaypointEditor(W.QWidget):
    pointsChanged=QtCore.pyqtSignal(list)
    logMessage=QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setStyleSheet('QPushButton{padding:6px 10px} QLineEdit{padding:5px;border:1px solid #30465f;border-radius:4px}')
        self.aircraft=None;self.points=[];self.drafts={};self.state={};self.geo_ready=False;self.fresh=False
        layout=W.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0);layout.setSpacing(8)
        self.kind=W.QComboBox();self.kind.addItem('米制坐标 · 本机 map','local');self.kind.addItem('经纬度 · WGS84','geo')
        mode_row=W.QHBoxLayout();mode_row.addWidget(self.kind,1);layout.addLayout(mode_row)
        form=W.QFormLayout();self.a_label=W.QLabel();self.b_label=W.QLabel()
        self.a=W.QLineEdit();self.b=W.QLineEdit()
        for field in (self.a,self.b):field.setMinimumWidth(150);field.setClearButtonEnabled(True)
        form.addRow(self.a_label,self.a);form.addRow(self.b_label,self.b);layout.addLayout(form)
        self.read=W.QPushButton('读取当前位置');self.read.clicked.connect(self.read_position);mode_row.addWidget(self.read)
        actions=W.QHBoxLayout();self.add=W.QPushButton('添加航点');self.add.setObjectName('primary')
        self.apply=W.QPushButton('保存修改');actions.addWidget(self.add);actions.addWidget(self.apply);layout.addLayout(actions)
        self.add.clicked.connect(lambda:self.commit(False));self.apply.clicked.connect(lambda:self.commit(True))
        self.message=W.QLabel('输入坐标后点击添加；选中列表中的航点可修改。');self.message.setWordWrap(True);layout.addWidget(self.message)
        self.table=W.QTableWidget(0,3);self.table.setHorizontalHeaderLabels(['类型','X / 纬度','Y / 经度'])
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setSelectionMode(W.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers);self.table.setMinimumHeight(110)
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.ResizeToContents)
        layout.addWidget(self.table,1)
        order=W.QHBoxLayout();self.up=W.QPushButton('上移');self.down=W.QPushButton('下移');self.delete=W.QPushButton('删除')
        for button in (self.up,self.down,self.delete):order.addWidget(button)
        layout.addLayout(order)
        self.up.clicked.connect(lambda:self.move(-1));self.down.clicked.connect(lambda:self.move(1));self.delete.clicked.connect(self.remove)
        self.table.itemSelectionChanged.connect(self.select_point);self.kind.currentIndexChanged.connect(self.change_kind)
        self.a.textEdited.connect(self.mark_draft);self.b.textEdited.connect(self.mark_draft)
        self.change_kind()

    def mark_draft(self,*args):self.message.setText('输入尚未写入航点列表，请点击「添加航点」或「保存修改」。')

    def change_kind(self,*args):
        geo=self.kind.currentData()=='geo'
        self.a_label.setText('纬度 (°)' if geo else 'X (m)');self.b_label.setText('经度 (°)' if geo else 'Y (m)')
        self.a.setPlaceholderText('-90 ～ 90' if geo else '-48.7 < X < 48.7')
        self.b.setPlaceholderText('-180 ～ 180' if geo else '-23.7 < Y < 23.7')
        self.a.clear();self.b.clear();self.update_actions()

    def set_aircraft(self,n,points):
        if self.aircraft is not None:
            self.drafts[self.aircraft]=(self.kind.currentIndex(),self.a.text(),self.b.text(),self.table.currentRow(),self.message.text())
        self.aircraft=n;self.points=[dict(p) for p in points]
        kind,a,b,row,message=self.drafts.get(n,(0,'','',-1,'输入坐标后点击添加；选中列表中的航点可修改。'))
        self.render(row)
        self.kind.setCurrentIndex(kind);self.change_kind();self.a.setText(a);self.b.setText(b);self.message.setText(message)
        self.update_actions()

    def set_status(self,state,fresh,geo_ready):
        self.state=state;self.fresh=bool(fresh and state.get('online'));self.geo_ready=geo_ready
        self.update_actions()

    def update_actions(self):
        geo=self.kind.currentData()=='geo';enabled=not geo or self.geo_ready
        row=self.table.currentRow();selected=0<=row<len(self.points)
        self.add.setEnabled(enabled);self.apply.setEnabled(enabled and selected)
        self.read.setEnabled(self.geo_ready if geo else self.fresh and bool(self.state.get('fresh')) and bool(self.state.get('position')))
        self.up.setEnabled(selected and row>0);self.down.setEnabled(selected and row<len(self.points)-1);self.delete.setEnabled(selected)
        hint='' if enabled else 'GNSS 与坐标参考暂不可用，恢复后自动启用。'
        self.add.setToolTip(hint);self.apply.setToolTip(hint)

    def read_position(self):
        self.update_actions()
        if not self.read.isEnabled():return
        if self.kind.currentData()=='geo':
            gps=self.state['gps'];a,b=gps['latitude'],gps['longitude'];digits=7
        else:a,b=self.state['position'][:2];digits=3
        self.a.setText(f'{a:.{digits}f}');self.b.setText(f'{b:.{digits}f}');self.mark_draft()

    def commit(self,replace):
        kind=self.kind.currentData();row=self.table.currentRow()
        if kind=='geo' and not self.geo_ready:return
        if not replace and len(self.points)>=50:
            self.message.setText('最多设置 50 个航点。');self.logMessage.emit(f'{self.aircraft} 号 · 最多设置 50 个航点。');return
        if replace and not 0<=row<len(self.points):return
        try:
            a,b=float(self.a.text()),float(self.b.text())
            if not all(math.isfinite(v) for v in (a,b)):raise ValueError('请输入有限数值。')
            if kind=='local' and not (abs(a)<48.7 and abs(b)<23.7):raise ValueError('米制范围：|X| < 48.7m，|Y| < 23.7m。')
            if kind=='geo' and not (-90<=a<=90 and -180<=b<=180):raise ValueError('纬度应在 ±90°，经度应在 ±180° 内。')
        except ValueError as e:
            self.message.setText('坐标无效：'+(str(e) if 'could not convert' not in str(e) else '请完整填写两个数字。'));self.logMessage.emit(f'{self.aircraft} 号 · '+self.message.text());return
        point=dict(kind=kind,a=a,b=b)
        if replace:self.points[row]=point
        else:self.points.append(point);row=len(self.points)-1
        self.publish(row,'已保存到本机航点列表；执行顺序为从上到下。')

    def render(self,row=-1):
        blocker=QtCore.QSignalBlocker(self.table);self.table.setRowCount(len(self.points))
        for i,p in enumerate(self.points):
            geo=p['kind']=='geo';digits=7 if geo else 3
            for col,text in enumerate(('经纬度' if geo else '米制',f"{p['a']:.{digits}f}",f"{p['b']:.{digits}f}")):
                item=W.QTableWidgetItem(text);item.setToolTip(str(p['a' if col==1 else 'b']) if col else text);self.table.setItem(i,col,item)
        self.table.clearSelection();self.table.setCurrentCell(-1,-1)
        if 0<=row<len(self.points):self.table.selectRow(row)
        del blocker
        self.update_actions()

    def select_point(self):
        row=self.table.currentRow()
        if not 0<=row<len(self.points):self.update_actions();return
        p=self.points[row];self.kind.setCurrentIndex(1 if p['kind']=='geo' else 0)
        self.a.setText(str(p['a']));self.b.setText(str(p['b']));self.message.setText(f'正在编辑第 {row+1} 个航点；修改后点击「保存修改」。');self.update_actions()

    def publish(self,row,message):
        self.render(row);self.select_point();self.message.setText(message)
        self.pointsChanged.emit([dict(p) for p in self.points]);self.logMessage.emit(f'{self.aircraft} 号 · '+message)

    def move(self,delta):
        row=self.table.currentRow();target=row+delta
        if not (0<=row<len(self.points) and 0<=target<len(self.points)):return
        self.points[row],self.points[target]=self.points[target],self.points[row]
        self.publish(target,'航点顺序已保存。')

    def remove(self):
        row=self.table.currentRow()
        if not 0<=row<len(self.points):return
        self.points.pop(row);self.publish(min(row,len(self.points)-1),'航点已删除。')

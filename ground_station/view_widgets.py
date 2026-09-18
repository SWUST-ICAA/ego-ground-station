"""Presentation-only widgets for independent aircraft views."""
import math
from PyQt5 import QtCore, QtGui, QtWidgets as W


class AllCheckBox(W.QCheckBox):
    def nextCheckState(self):
        # Partial is a display state; a click always selects all or clears all.
        self.setCheckState(QtCore.Qt.Unchecked if self.checkState()==QtCore.Qt.Checked else QtCore.Qt.Checked)


class LocalPlot(W.QWidget):
    def __init__(self,aircraft):
        super().__init__();self.setMinimumSize(180,150)
        self.setSizePolicy(W.QSizePolicy.Expanding,W.QSizePolicy.Expanding)
        self.aircraft=aircraft;self.state={};self.points=[];self.trace=[]
    def change(self,state,points,trace):
        self.state=state;self.points=points;self.trace=trace;self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#111d2c'))
        p.setPen(QtGui.QPen(QtGui.QColor('#2d425c'),1));p.drawRect(self.rect().adjusted(0,0,-1,-1))
        p.setPen(QtGui.QColor('#c5d5e7'));p.drawText(12,23,f'{self.aircraft} 号机 · 局部 map / m')
        local=[(v['a'],v['b']) for v in self.points if v['kind']=='local']
        actual=self.state.get('mission',{}).get('points',[])
        targets=[(a[0],a[1]) for a in actual] or local
        pos=self.state.get('position');home=self.state.get('mission',{}).get('home')
        coords=self.trace+targets+([(pos[0],pos[1])] if pos else [])+([(home[0],home[1])] if home else [])+[(0,0)]
        extent=max(5,max(abs(v) for xy in coords for v in xy)+2)
        scale=min(self.width()-52,self.height()-65)/(2*extent)
        cx,cy=self.width()/2,(self.height()+8)/2
        def xy(a,b):return QtCore.QPointF(cx+a*scale,cy-b*scale)
        step=max(1,math.ceil(extent/4))
        p.setPen(QtGui.QPen(QtGui.QColor('#233247'),1))
        for i in range(-4,5):
            v=i*step
            if abs(v)<=extent:
                p.drawLine(xy(v,-extent),xy(v,extent));p.drawLine(xy(-extent,v),xy(extent,v))
        p.setPen(QtGui.QPen(QtGui.QColor('#58708d'),1))
        p.drawLine(xy(-extent,0),xy(extent,0));p.drawLine(xy(0,-extent),xy(0,extent))
        p.drawText(xy(extent,0)+QtCore.QPointF(-17,-5),'+X');p.drawText(xy(0,extent)+QtCore.QPointF(5,11),'+Y')
        p.drawText(12,self.height()-10,f'网格 {step:g}m')
        if len(self.trace)>1:
            p.setPen(QtGui.QPen(QtGui.QColor('#36c5cc'),2));p.drawPolyline(QtGui.QPolygonF([xy(*a) for a in self.trace]))
        p.setPen(QtGui.QPen(QtGui.QColor('#f6b655'),2));p.setBrush(QtCore.Qt.NoBrush)
        for i,pt in enumerate(targets):
            q=xy(*pt);p.drawEllipse(q,5,5);p.drawText(q+QtCore.QPointF(7,-7),str(i+1))
        if home:
            q=xy(home[0],home[1]);p.setPen(QtGui.QColor('#6be2a5'));p.drawRect(QtCore.QRectF(q.x()-5,q.y()-5,10,10))
        if pos:
            q=xy(pos[0],pos[1]);p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#35d0db'));p.drawEllipse(q,5,5)
        else:
            p.setPen(QtGui.QColor('#8297af'));p.drawText(self.rect(),QtCore.Qt.AlignCenter,'等待位置')
        if not self.state.get('online',False):
            p.setPen(QtGui.QColor('#f6b655'));p.drawText(self.width()-66,self.height()-10,'未连接')
        p.end()


class MapPanel(W.QScrollArea):
    def __init__(self,aircraft):
        super().__init__();self.setWidgetResizable(True);self.setFrameShape(W.QFrame.NoFrame)
        self.setMinimumSize(200,150)
        self.body=W.QWidget();self.grid=W.QGridLayout(self.body);self.grid.setContentsMargins(0,0,0,0);self.grid.setSpacing(10)
        self.plots={n:LocalPlot(n) for n in aircraft};self.columns=0
        self.setWidget(self.body);self.viewport().installEventFilter(self)
        self.reflow(self.viewport().width())
    def eventFilter(self,obj,event):
        if obj is self.viewport() and event.type()==QtCore.QEvent.Resize:
            self.reflow(event.size().width())
        return super().eventFilter(obj,event)
    def reflow(self,width):
        columns=min(len(self.plots),4,max(1,(width+10)//210))
        if columns==self.columns:return
        while self.grid.count():self.grid.takeAt(0)
        for col in range(max(self.columns,columns)):self.grid.setColumnStretch(col,0)
        for row in range(self.grid.rowCount()):self.grid.setRowStretch(row,0)
        for i,plot in enumerate(self.plots.values()):
            self.grid.addWidget(plot,i//columns,i%columns);self.grid.setRowStretch(i//columns,1)
        for col in range(columns):self.grid.setColumnStretch(col,1)
        rows=math.ceil(len(self.plots)/columns)
        self.body.setMinimumHeight(rows*150+(rows-1)*10)
        self.columns=columns

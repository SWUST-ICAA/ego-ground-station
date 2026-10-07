"""Embedded, read-only OpenGL view of the planner's native 3D obstacle grid."""
import math
import time
import numpy as np
from OpenGL import GL as G
from PyQt5 import QtCore, QtGui, QtWidgets as W
from onboard.visualization import unpack_voxels


def voxel_mesh(mask,origin,resolution,color):
    """Draw only exposed faces; removing interior faces does not alter occupancy."""
    if not mask.size or not mask.any():return np.empty((0,3),np.float32),np.empty((0,3),np.float32),False
    padded=np.pad(mask,1);faces=[];count=0
    for axis in range(3):
        for direction in (-1,1):
            selection=[slice(1,-1)]*3;selection[axis]=slice(0,-2) if direction<0 else slice(2,None)
            indices=np.argwhere(mask & ~padded[tuple(selection)])
            faces.append((axis,direction,indices));count+=len(indices)
    if count>500000:
        vertices=(np.argwhere(mask)*resolution+origin).astype(np.float32)
        return vertices,np.tile(np.array(color,np.float32),(len(vertices),1)),True
    vertices=[];colors=[]
    for axis,direction,indices in faces:
        if not len(indices):continue
        corners=np.zeros((4,3));corners[:,axis]=direction*.5
        other=[a for a in range(3) if a!=axis]
        corners[:,other]=[[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]
        centers=indices*resolution+origin
        vertices.append((centers[:,None,:]+corners[None,:,:]*resolution).reshape(-1,3))
        shade=(.72,.85,1.)[axis]*(1. if direction>0 else .85)
        colors.append(np.tile(np.array(color)*shade,(len(indices)*4,1)))
    return np.asarray(np.concatenate(vertices),np.float32),np.asarray(np.concatenate(colors),np.float32),False


class CloudCanvas(W.QOpenGLWidget):
    def __init__(self):
        super().__init__()
        fmt=QtGui.QSurfaceFormat();fmt.setVersion(2,1);fmt.setDepthBufferSize(24);self.setFormat(fmt)
        self.aircraft=None;self.data={};self.received=0.;self.layer='voxel_map';self.slice=False
        self.drag=None;self.meshes=[];self.render_error='';self.raw=np.empty((0,3),np.float32)
        self.reset_view();self.setMinimumSize(250,240)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.update);self.timer.start(500)

    def reset_view(self,top=False):
        self.azimuth=math.radians(-35);self.elevation=math.radians(90 if top else 35)
        self.zoom=1.;self.pan=QtCore.QPointF();self.update()

    def mousePressEvent(self,event):
        if event.button() in (QtCore.Qt.LeftButton,QtCore.Qt.RightButton,QtCore.Qt.MiddleButton):self.drag=event.pos();event.accept()

    def mouseMoveEvent(self,event):
        if self.drag is None:return
        delta=event.pos()-self.drag;self.drag=event.pos()
        if event.buttons() & QtCore.Qt.LeftButton:
            self.azimuth+=delta.x()*.008
            self.elevation=max(math.radians(5),min(math.radians(90),self.elevation+delta.y()*.008))
        else:self.pan+=QtCore.QPointF(delta)
        self.update()

    def mouseReleaseEvent(self,event):self.drag=None

    def wheelEvent(self,event):
        self.zoom=max(.3,min(12.,self.zoom*1.15**(event.angleDelta().y()/120)));self.update();event.accept()

    def age(self,value):return None if value is None else value+max(0.,time.monotonic()-self.received)

    def rebuild(self):
        self.meshes=[];self.render_error=''
        try:
            keys=['voxel_map','voxel_inflated'] if self.layer=='both' else [self.layer]
            frames={self.data[k].get('frame') for k in keys if k in self.data}
            if len(frames)>1:raise ValueError('两层坐标系不同，已暂停叠加')
            for key in keys:
                if key not in ('voxel_map','voxel_inflated'):continue
                packet=self.data.get(key)
                if packet is None:
                    if self.data.get(key+'_error'):raise ValueError(self.data[key+'_error'])
                    continue
                mask,origin,res=unpack_voxels(packet)
                if self.slice and mask.size:
                    height=self.data.get('target_height',0.)
                    mask[:,:,np.abs(origin[2]+np.arange(mask.shape[2])*res-height)>.5]=False
                color=(.30,.70,.90) if key=='voxel_map' else (1.,.63,.25)
                vertices,colors,points=voxel_mesh(mask,origin,res,color)
                self.meshes.append(dict(key=key,vertices=vertices,colors=colors,points=points,
                                        count=int(mask.sum()),resolution=res,age=packet.get('age_sec')))
            self.raw=np.asarray(self.data.get('cloud',[]),np.float32).reshape(-1,3)
            self.raw=self.raw[np.isfinite(self.raw).all(axis=1)]
            if self.slice:self.raw=self.raw[np.abs(self.raw[:,2]-self.data.get('target_height',0.))<=.5]
        except (ValueError,KeyError,TypeError) as e:self.render_error=str(e);self.meshes=[]
        self.update()

    def initializeGL(self):G.glClearColor(.045,.075,.115,1.)

    def draw_arrays(self,vertices,colors,mode):
        if not len(vertices):return
        G.glEnableClientState(G.GL_VERTEX_ARRAY);G.glEnableClientState(G.GL_COLOR_ARRAY)
        G.glVertexPointer(3,G.GL_FLOAT,0,vertices);G.glColorPointer(3,G.GL_FLOAT,0,colors)
        G.glDrawArrays(mode,0,len(vertices))
        G.glDisableClientState(G.GL_COLOR_ARRAY);G.glDisableClientState(G.GL_VERTEX_ARRAY)

    def paintGL(self):
        G.glClear(G.GL_COLOR_BUFFER_BIT|G.GL_DEPTH_BUFFER_BIT);G.glEnable(G.GL_DEPTH_TEST)
        G.glDisable(G.GL_LIGHTING);G.glDisable(G.GL_CULL_FACE);G.glDisable(G.GL_BLEND)
        scale=max(20,min(self.width(),self.height()))/14.*self.zoom
        G.glMatrixMode(G.GL_PROJECTION);G.glLoadIdentity()
        G.glOrtho(-self.width()/2/scale,self.width()/2/scale,-self.height()/2/scale,self.height()/2/scale,-1000,1000)
        G.glMatrixMode(G.GL_MODELVIEW);G.glLoadIdentity();G.glTranslatef(self.pan.x()/scale,-self.pan.y()/scale,0)
        ca,sa=math.cos(self.azimuth),math.sin(self.azimuth);ce,se=math.cos(self.elevation),math.sin(self.elevation)
        rotation=np.array([[ca,-sa,0,0],[se*sa,se*ca,ce,0],[-ce*sa,-ce*ca,se,0],[0,0,0,1]],np.float32)
        G.glMultMatrixf(rotation.T.copy())
        position=self.data.get('position');center=position or [0.,0.,0.];G.glTranslatef(*[-v for v in center])
        floor=self.data.get('ground_height',0.)
        G.glColor3f(.16,.24,.33);G.glLineWidth(1);G.glBegin(G.GL_LINES)
        for step in range(-8,9,2):
            for x,y in [(center[0]+step,center[1]-8),(center[0]+step,center[1]+8),
                        (center[0]-8,center[1]+step),(center[0]+8,center[1]+step)]:G.glVertex3f(x,y,floor)
        G.glEnd();count=0
        for mesh in self.meshes:
            age=self.age(mesh['age'])
            if age is None or age>2:continue
            wire=self.layer=='both' and mesh['key']=='voxel_inflated'
            if wire:G.glPolygonMode(G.GL_FRONT_AND_BACK,G.GL_LINE)
            G.glPointSize(max(1.,mesh['resolution']*scale))
            self.draw_arrays(mesh['vertices'],mesh['colors'],G.GL_POINTS if mesh['points'] else G.GL_QUADS)
            G.glPolygonMode(G.GL_FRONT_AND_BACK,G.GL_FILL);count+=mesh['count']
        cloud_age=self.age(self.data.get('cloud_age_sec'))
        if self.layer=='cloud' and cloud_age is not None and cloud_age<=2:
            G.glPointSize(3.)
            low=self.data.get('ground_height',0.);high=max(low+.1,self.data.get('ceiling_height',3.))
            h=np.clip((self.raw[:,2]-low)/(high-low),0,1)
            colors=np.column_stack([h,1.-abs(h-.5),1.-h]).astype(np.float32)
            self.draw_arrays(self.raw,colors,G.GL_POINTS);count=len(self.raw)
        age=self.age(self.data.get('traj_age_sec'))
        if age is not None and age<=2:
            path=np.asarray(self.data.get('trajectory',[]),np.float32).reshape(-1,3)
            G.glLineWidth(3.);self.draw_arrays(path,np.tile(np.array([1.,.3,.45],np.float32),(len(path),1)),G.GL_LINE_STRIP)
        if position:
            G.glPointSize(10.);G.glColor3f(.1,1.,.85);G.glBegin(G.GL_POINTS);G.glVertex3f(*position);G.glEnd()
            G.glLineWidth(2.);G.glBegin(G.GL_LINES)
            for axis,color in enumerate([(1.,.3,.3),(.3,1.,.3),(.3,.5,1.)]):
                end=list(position);end[axis]+=.8;G.glColor3f(*color);G.glVertex3f(*position);G.glVertex3f(*end)
            G.glEnd()
        G.glDisable(G.GL_DEPTH_TEST)
    def status_text(self):
        count=sum(m['count'] for m in self.meshes if self.age(m['age']) is not None and self.age(m['age'])<=2)
        if self.layer=='cloud':count=len(self.raw) if self.age(self.data.get('cloud_age_sec')) is not None and self.age(self.data.get('cloud_age_sec'))<=2 else 0
        def age_text(key):
            age=self.age(self.data.get(key));return '—' if age is None else f'{age:.1f}'
        message=self.data.get('error') or self.render_error or ('所选图层暂无新鲜数据，或显示范围内无障碍' if self.data and not count else '')
        geometry=f"原始栅格 {self.data.get('resolution',0.):.3f} m · 蓝色：障碍 · 橙色：膨胀 · 红线：规划轨迹"
        if any(m['points'] for m in self.meshes):geometry+=' · 高密度：全部格中心点显示'
        if self.slice:geometry=f"显示切片：目标高度 {self.data.get('target_height',0.):.2f} ±0.50 m；规划器障碍不变"
        return f"{self.aircraft or '—'} 号机 · {self.data.get('map_frame') or '—'} · 地图 {age_text('map_age_sec')} s · 雷达 {age_text('cloud_age_sec')} s · 显示 {count:,} 个栅格/点\n{geometry}"+(f"\n{message}" if message else '')



class PlannerView(W.QWidget):
    def __init__(self):
        super().__init__();layout=W.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        controls=W.QHBoxLayout();controls.setContentsMargins(8,4,8,0);self.layers=W.QComboBox()
        for text,key in [('障碍栅格','voxel_map'),('膨胀障碍','voxel_inflated'),('障碍 + 膨胀','both'),('雷达点云','cloud')]:self.layers.addItem(text,key)
        self.slice=W.QCheckBox('仅看目标高度 ±0.5 m');reset=W.QPushButton('三维复位');top=W.QPushButton('俯视')
        for w in (self.layers,self.slice,reset,top):controls.addWidget(w)
        controls.addStretch();layout.addLayout(controls);self.canvas=CloudCanvas();layout.addWidget(self.canvas,1)
        hint=W.QLabel('左键旋转 · 右键平移 · 滚轮缩放 · 跟随机位 · 网格间距 2 m');layout.addWidget(hint)
        self.status=W.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
        self.status_timer=QtCore.QTimer(self);self.status_timer.timeout.connect(lambda:self.status.setText(self.canvas.status_text()));self.status_timer.start(200)
        self.layers.currentIndexChanged.connect(self.options_changed);self.slice.toggled.connect(self.options_changed)
        reset.clicked.connect(lambda:self.canvas.reset_view());top.clicked.connect(lambda:self.canvas.reset_view(True))

    def options_changed(self,*args):
        self.canvas.layer=self.layers.currentData();self.canvas.slice=self.slice.isChecked();self.canvas.rebuild()

    def show_aircraft(self,aircraft):
        if self.canvas.aircraft!=aircraft:
            self.canvas.aircraft=aircraft;self.canvas.data={};self.canvas.received=0.
            self.slice.setChecked(False);self.layers.setCurrentIndex(0);self.canvas.reset_view();self.canvas.rebuild()

    def change(self,aircraft,data):
        if aircraft==self.canvas.aircraft:
            self.canvas.data=data;self.canvas.received=time.monotonic();self.canvas.rebuild()

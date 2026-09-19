"""Read-only top-down view of the selected aircraft's local planner data."""
from PyQt5 import QtCore, QtGui, QtWidgets as W


class PlannerView(W.QWidget):
    def __init__(self):
        super().__init__()
        self.aircraft = None
        self.data = {}
        self.setMinimumSize(250, 220)

    def show_aircraft(self, aircraft):
        if self.aircraft != aircraft:
            self.aircraft, self.data = aircraft, {}
            self.update()

    def change(self, aircraft, data):
        if aircraft == self.aircraft:
            self.data = data
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#111d2c'))
        painter.setPen(QtGui.QColor('#dce7f5'))
        painter.drawText(16, 27, f'{self.aircraft or "—"} 号机 · 局部规划观察（俯视）')
        painter.setPen(QtGui.QColor('#8297af'))
        painter.drawText(16, 48, '膨胀障碍 / 当前规划轨迹 / 机位 · 仅供观察')
        if self.data.get('error'):
            painter.setPen(QtGui.QColor('#f6b655'))
            painter.drawText(self.rect().adjusted(20, 65, -20, -20), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap,
                             '观察数据暂不可用：'+self.data['error'][-180:])
            painter.end()
            return
        box = self.rect().adjusted(25, 63, -25, -55)
        side = max(20, min(box.width(), box.height()))
        cx, cy = box.center().x(), box.center().y()
        scale = side / 16.0  # ±8 m about the live position
        position = self.data.get('position')
        center = position[:2] if position else (0.0, 0.0)

        def screen(point):
            return QtCore.QPointF(cx+(point[0]-center[0])*scale,
                                  cy-(point[1]-center[1])*scale)

        painter.setPen(QtGui.QPen(QtGui.QColor('#24364c'), 1))
        for step in range(-8, 9, 2):
            painter.drawLine(screen((center[0]+step, center[1]-8)), screen((center[0]+step, center[1]+8)))
            painter.drawLine(screen((center[0]-8, center[1]+step)), screen((center[0]+8, center[1]+step)))
        painter.setPen(QtGui.QPen(QtGui.QColor('#4f6780'), 1))
        painter.drawLine(screen((center[0]-8, center[1])), screen((center[0]+8, center[1])))
        painter.drawLine(screen((center[0], center[1]-8)), screen((center[0], center[1]+8)))
        painter.save()
        painter.setClipRect(QtCore.QRectF(cx-side/2, cy-side/2, side, side))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(238, 148, 80, 190))
        for point in self.data.get('inflated', []):
            q = screen(point)
            painter.drawEllipse(q, 2.3, 2.3)
        path = self.data.get('trajectory', [])
        if len(path) > 1:
            painter.setPen(QtGui.QPen(QtGui.QColor('#ef6f79'), 3, QtCore.Qt.SolidLine,
                                      QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            painter.drawPolyline(QtGui.QPolygonF([screen(point) for point in path]))
        if position:
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor('#35d0db'))
            painter.drawEllipse(screen(position), 6, 6)
        painter.restore()
        painter.setPen(QtGui.QColor('#9bb2cc'))
        map_age = self.data.get('map_age_sec')
        traj_age = self.data.get('traj_age_sec')
        painter.drawText(16, self.height()-36,
                         f"地图 {map_age if map_age is not None else '—'} s  ·  轨迹 {traj_age if traj_age is not None else '—'} s"
                         f"  ·  {len(self.data.get('inflated', []))} 个显示点")
        painter.drawText(16, self.height()-15,
                         f"坐标系 {self.data.get('map_frame') or '—'}  ·  中心 ({center[0]:.1f}, {center[1]:.1f}) m"
                         + ('  ·  等待实时数据' if not self.data else ''))
        painter.end()

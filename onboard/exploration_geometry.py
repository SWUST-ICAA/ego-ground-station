"""Conservative grid certificate for Bezier curves, independent of ROS."""
import math


class GridCertificate:
    def __init__(self,msg):
        self.width=int(msg.info.width);self.height=int(msg.info.height)
        self.resolution=float(msg.info.resolution)
        self.x=float(msg.info.origin.position.x);self.y=float(msg.info.origin.position.y)
        q=msg.info.origin.orientation
        if (msg.header.frame_id!='map' or not 0<self.width<=2000 or not 0<self.height<=2000
            or not .05<=self.resolution<=1 or len(msg.data)!=self.width*self.height
            or abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1)>1e-6
            or not math.isfinite(self.x+self.y)):
            raise ValueError('invalid exploration grid')
        self.data=msg.data

    def cell(self,p):return math.floor((p[0]-self.x)/self.resolution),math.floor((p[1]-self.y)/self.resolution)

    def point(self,p):
        x,y=self.cell(p)
        return 0<=x<self.width and 0<=y<self.height and self.data[y*self.width+x]==0

    def box(self,points):
        cells=[self.cell(p) for p in points]
        x0=min(p[0] for p in cells);x1=max(p[0] for p in cells)
        y0=min(p[1] for p in cells);y1=max(p[1] for p in cells)
        if x0<0 or y0<0 or x1>=self.width or y1>=self.height:return False
        return all(self.data[y*self.width+x]==0 for y in range(y0,y1+1) for x in range(x0,x1+1))

    def curve(self,points,zmin,zmax,depth=0):
        if not points or not all(math.isfinite(v) for p in points for v in p):return False
        if min(p[2] for p in points)<zmin or max(p[2] for p in points)>zmax:return False
        if self.box(points):return True
        if depth>=9 or not self.point(points[0]) or not self.point(points[-1]):return False
        left,right=split_curve(points,.5)
        return self.curve(left,zmin,zmax,depth+1) and self.curve(right,zmin,zmax,depth+1)


def split_curve(points,t):
    row=[tuple(p) for p in points];left=[row[0]];right=[row[-1]]
    while len(row)>1:
        row=[tuple((1-t)*a+t*b for a,b in zip(p,q)) for p,q in zip(row,row[1:])]
        left.append(row[0]);right.append(row[-1])
    return left,list(reversed(right))


def remaining_hulls(hulls,durations,elapsed):
    result=[]
    for hull,duration in zip(hulls,durations):
        if elapsed>=duration:elapsed-=duration;continue
        if elapsed>0:hull=split_curve(hull,elapsed/duration)[1]
        result.append(hull);elapsed=0
    return result

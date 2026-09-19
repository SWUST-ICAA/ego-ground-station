"""Dependency-free horizontal polygon clearance and conservative route search."""
import heapq
import math

EPS=1e-7
LIMITS=(-498.7,498.7,-498.7,498.7)


def xy(p):
    if len(p)!=2:raise ValueError('边界点需要两个坐标')
    p=tuple(map(float,p))
    if not all(math.isfinite(v) for v in p):raise ValueError('边界坐标必须为有限数值')
    return p


def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm)) if norm else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def intersects(a,b,c,d):
    ab1,ab2,cd1,cd2=cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)
    if ((ab1>EPS and ab2<-EPS) or (ab1<-EPS and ab2>EPS)) and ((cd1>EPS and cd2<-EPS) or (cd1<-EPS and cd2>EPS)):return True
    return any(abs(v)<=EPS and distance(p,x,y)<=EPS for v,p,x,y in ((ab1,c,a,b),(ab2,d,a,b),(cd1,a,c,d),(cd2,b,c,d)))


def segment_distance(a,b,c,d):
    if intersects(a,b,c,d):return 0.
    return min(distance(a,c,d),distance(b,c,d),distance(c,a,b),distance(d,a,b))


def hull(points):
    pts=sorted(set(tuple(p[:2]) for p in points))
    if len(pts)<3:return pts
    def half(seq):
        out=[]
        for p in seq:
            while len(out)>1 and cross(out[-2],out[-1],p)<=0:out.pop()
            out.append(p)
        return out
    return half(pts)[:-1]+half(reversed(pts))[:-1]


class Fence:
    def __init__(self,polygon,margin):
        self.polygon=[xy(p) for p in polygon];self.margin=float(margin)
        if not 3<=len(self.polygon)<=64 or not math.isfinite(self.margin) or self.margin<.75:
            raise ValueError('边界需要 3～64 点；内缩距离至少为机体半径 0.75m')
        self.edges=list(zip(self.polygon,self.polygon[1:]+self.polygon[:1]))
        if len(set(self.polygon))!=len(self.polygon) or abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in self.edges))<1e-4:raise ValueError('边界重复或面积为零')
        for i,(a,b) in enumerate(self.edges):
            for j,(c,d) in enumerate(self.edges):
                if j<=i+1 or (i==0 and j==len(self.edges)-1):continue
                if intersects(a,b,c,d):raise ValueError('边界不能自相交')

    def contains(self,p,extra=0.):
        x,y=xy(p);inside=False
        for a,b in self.edges:
            if distance((x,y),a,b)<self.margin+extra-EPS:return False
            if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
        return inside and LIMITS[0]<x<LIMITS[1] and LIMITS[2]<y<LIMITS[3]

    def segment(self,a,b,extra=0.):
        a,b=xy(a),xy(b)
        return self.contains(a,extra) and self.contains(b,extra) and all(segment_distance(a,b,c,d)>=self.margin+extra-EPS for c,d in self.edges)

    def control_hull(self,points):
        pts=hull(points)
        return bool(pts) and all(self.segment(a,b) for a,b in zip(pts,pts[1:]+pts[:1]))

    def route(self,start,goal,extra=.5):
        """A* on 0.5–2m cells; every edge and simplification has exact clearance checks."""
        start,goal=xy(start),xy(goal)
        if not self.contains(start,extra) or not self.contains(goal,extra):raise ValueError('起点或目标不在内缩边界/现有规划地图内，或未留出 0.5m 跟踪余量')
        if self.segment(start,goal,extra):return [list(goal)]
        span=max(max(p[i] for p in self.polygon)-min(p[i] for p in self.polygon) for i in (0,1))
        step=max(.5,min(2.,span/200));cache={};parents={};cost={};queue=[]
        def pt(k):return (k[0]*step,k[1]*step)
        def valid(k):
            if k not in cache:cache[k]=self.contains(pt(k),extra)
            return cache[k]
        center=(round(start[0]/step),round(start[1]/step))
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                k=(center[0]+dx,center[1]+dy);p=pt(k)
                if valid(k) and self.segment(start,p,extra):
                    g=math.dist(start,p);cost[k]=g;parents[k]=None;heapq.heappush(queue,(g+math.dist(p,goal),g,k))
        found=None;visited=0
        while queue:
            _,g,k=heapq.heappop(queue)
            if g!=cost[k]:continue
            visited+=1
            if visited>100000:raise ValueError('航线搜索超出限制')
            p=pt(k)
            if math.dist(p,goal)<=2*step and self.segment(p,goal,extra):found=k;break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(k[0]+dx,k[1]+dy);q=pt(nxt);ng=g+step*math.hypot(dx,dy)
                if ng>=cost.get(nxt,float('inf')) or not valid(nxt) or not self.segment(p,q,extra):continue
                cost[nxt]=ng;parents[nxt]=k;heapq.heappush(queue,(ng+math.dist(q,goal),ng,nxt))
        if found is None:raise ValueError('内缩区域不连通或通道过窄，未找到航线')
        path=[goal]
        while found is not None:path.append(pt(found));found=parents[found]
        path.append(start);path.reverse();result=[];i=0
        while i<len(path)-1:
            j=len(path)-1
            while j>i+1 and not self.segment(path[i],path[j],extra):j-=1
            result.append(list(path[j]));i=j
        return result

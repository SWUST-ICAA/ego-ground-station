# 4～7 号机部署记录

日期：2026-09-18。复用与前三架相同的独立单机方案，保留每架原 MID360 配置和 FAST-LIO 外参。

| 飞机 | 飞控 | 机载机以太网 | 雷达 → 机载机 | 静态验证 |
|---|---|---|---|---|
| 4 | 10.41.10.24:14550 | 10.41.10.14:14550 | 192.168.1.158 → 192.168.1.204 | connected / ready，点云非空，未解锁 |
| 5 | 10.41.10.25:14550 | 10.41.10.15:14550 | 192.168.1.160 → 192.168.1.205 | connected / ready，点云非空，未解锁 |
| 6 | 10.41.10.26:14550 | 10.41.10.16:14550 | 192.168.1.184 → 192.168.1.206 | connected / ready，点云非空，未解锁 |
| 7 | 10.41.10.27:14550 | 10.41.10.17:14550 | 192.168.1.191 → 192.168.1.207 | connected / ready，点云非空，未解锁 |

已回读四架参数：flight_type=1、thresh_replan_time=0.2、resolution=0.1、obstacles_inflation=1.3、ground_height=1.3、map_size_z=0.4、max_vel=2、max_acc=6。目标高度和初始 yaw 保持逻辑沿用前三架源码。

四架的 swarm-flight、swarm-fastlio、swarm-mid360、swarm-lidar-recovery 服务均 inactive/disabled；保留的旧容器 restart=no。新 fast-drone-250 容器 restart=no，只运行传感器/规划器及等待命令的地面站代理；无 px4_controller，也无 /mavros/setpoint_raw/local 发布者。

原雷达驱动、FAST-LIO、定位桥核心和 MAVLink 转发二进制的 SHA256 与前三架复用版本一致。原 JSON/YAML 原样保存于 Fast-Drone-250/deploy/aircraft/4～7；运行时只沿用已有启动覆盖项（注册点云发布等）。

本次真机未发布导航目标、未请求解锁/起飞/模式切换；尚未进行实际飞行验收。当前无有效经纬度数据，不影响设置本机米制航点。

地面站已支持 1～7 号机，也允许配置其中的非空子集；校验重复编号及范围。已验证七架离线状态、7 号选择和全部七行显示。

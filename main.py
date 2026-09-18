#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
import sys
from PyQt5 import QtWidgets
from ground_station.ui import Window


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description='Fast Drone 独立单机地面站')
    parser.add_argument('--demo',action='store_true',help='离线模拟，不连接真实飞机')
    parser.add_argument('--config',type=Path,default=root/'config.local.json')
    args=parser.parse_args()
    if not args.config.exists():
        args.config.write_text((root/'config.example.json').read_text());os.chmod(args.config,0o600)
    config=json.loads(args.config.read_text())
    aircraft=config.get('aircraft',[])
    if sorted(a['id'] for a in aircraft)!=[1,2,3]:parser.error('当前仅支持 1、2、3 号机，各一条配置')
    if args.demo:
        # Demo edits must not replace flight configuration.
        args.config=root/'logs/demo-config.json';args.config.parent.mkdir(exist_ok=True)
        config=json.loads((root/'config.example.json').read_text())
        for n in (1,2,3):config['waypoints'][str(n)]=[dict(kind='local',a=2.,b=float(n-1))]
    app=QtWidgets.QApplication(sys.argv[:1]);app.setApplicationName('Fast Drone Ground Station')
    window=Window(config,args.config,args.demo);window.show();return app.exec_()


if __name__=='__main__':sys.exit(main())

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
    ids=[a['id'] for a in aircraft]
    if not ids or len(ids)!=len(set(ids)) or any(type(n) is not int or not 1<=n<=7 for n in ids):
        parser.error('飞机编号必须为 1～7，至少一架且不能重复')
    if args.demo:
        # Demo edits must not replace flight configuration.
        args.config=root/'logs/demo-config.json';args.config.parent.mkdir(exist_ok=True)
        config=json.loads((root/'config.example.json').read_text())
        for n in (a['id'] for a in config['aircraft']):config['waypoints'][str(n)]=[dict(kind='local',a=2.,b=float(n-1))]
    app=QtWidgets.QApplication(sys.argv[:1]);app.setApplicationName('Fast Drone Ground Station')
    window=Window(config,args.config,args.demo);window.show();return app.exec_()


if __name__=='__main__':sys.exit(main())

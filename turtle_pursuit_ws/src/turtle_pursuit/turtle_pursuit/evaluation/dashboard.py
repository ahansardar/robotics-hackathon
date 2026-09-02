import json
import math
import signal
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPalette, QPen, QPixmap
from PyQt5.QtWidgets import QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QProgressBar, QVBoxLayout, QWidget


class DashboardNode(Node):
    def __init__(self):
        super().__init__('match_dashboard')
        self.declare_parameter('scenario','Competition Gauntlet')
        self.latest=None; self.last_message=0.; self.sensors={}; self.images={}; self.scans={}; self.obstacles={}; self.revisions={}; self.arrivals={}
        self.create_subscription(String,'/match/state',self._state,10)
        for topic in ('/catcher/scan','/runner/scan'):
            self.create_subscription(LaserScan,topic,lambda message,t=topic:self._scan(message,t),qos_profile_sensor_data)
        for topic in ('/catcher/camera/color/image_raw','/runner/camera/color/image_raw','/catcher/camera/depth/image_raw','/runner/camera/depth/image_raw'):
            self.create_subscription(Image,topic,lambda message,t=topic:self._image(message,t),qos_profile_sensor_data)
        for role in ('catcher','runner'):
            self.create_subscription(String,f'/match/{role}_obstacles',lambda message,r=role:self._obstacles(message,r),10)
    def _state(self,message):
        try:self.latest=json.loads(message.data); self.last_message=time.monotonic()
        except json.JSONDecodeError:self.get_logger().warning('ignored malformed /match/state JSON')
    def _mark(self,topic):
        now=time.monotonic(); self.sensors[topic]=now
        self.arrivals.setdefault(topic,deque(maxlen=120)).append(now)
        self.revisions[topic]=self.revisions.get(topic,0)+1
    def topic_rate(self,topic):
        samples=self.arrivals.get(topic)
        return (len(samples)-1)/(samples[-1]-samples[0]) if samples is not None and len(samples)>1 and samples[-1]>samples[0] else 0.
    def _scan(self,message,topic): self._mark(topic); self.scans[topic]=message
    def _image(self,message,topic): self._mark(topic); self.images[topic]=message
    def _obstacles(self,message,role):
        try:self.obstacles[role]=json.loads(message.data).get('centers',[])
        except json.JSONDecodeError:pass


class MetricCard(QFrame):
    def __init__(self,title,color='#8fa3bf'):
        super().__init__(); self.setObjectName('card')
        layout=QVBoxLayout(self); layout.setContentsMargins(12,8,12,8)
        heading=QLabel(title.upper()); heading.setStyleSheet(f'color:{color};font-size:11px;font-weight:700;letter-spacing:1px')
        self.value=QLabel('--'); self.value.setStyleSheet('color:#f4f7fb;font-size:25px;font-weight:800')
        layout.addWidget(heading); layout.addWidget(self.value)


class MatchDashboard(QWidget):
    def __init__(self,node):
        super().__init__(); self.node=node; self.previous=('', '', ''); self.setWindowTitle('Turtle Pursuit — Live Gauntlet Telemetry'); self.resize(470,760); self.move(20,150)
        self.setStyleSheet('QWidget{background:#0b111b;color:#e8eef7;font-family:Inter,Arial} QFrame#card{background:#131d2c;border:1px solid #26354b;border-radius:10px} QProgressBar{background:#202b3a;border:0;border-radius:6px;height:12px;text-align:center;color:transparent} QProgressBar::chunk{background:#38bdf8;border-radius:6px}')
        root=QVBoxLayout(self); root.setContentsMargins(18,16,18,16); root.setSpacing(12)
        title=QLabel('PURSUIT GAUNTLET'); title.setStyleSheet('font-size:25px;font-weight:900;color:#f8fafc')
        scenario=QLabel(node.get_parameter('scenario').value); scenario.setWordWrap(True); scenario.setStyleSheet('font-size:13px;color:#93c5fd')
        self.status=QLabel('WAITING FOR ROBOTS'); self.status.setAlignment(Qt.AlignCenter); self.status.setStyleSheet('background:#334155;border-radius:8px;padding:9px;font-weight:800')
        root.addWidget(title); root.addWidget(scenario); root.addWidget(self.status)
        grid=QGridLayout(); grid.setSpacing(9); root.addLayout(grid)
        cards=(('elapsed','ELAPSED','#38bdf8'),('remaining','TIME LEFT','#fbbf24'),('separation','DISTANCE','#c4b5fd'),('minimum','MIN DISTANCE','#fb7185'),('catcher_speed','CATCHER SPEED','#f87171'),('runner_speed','RUNNER SPEED','#60a5fa'),('catcher_path','CATCHER PATH','#fca5a5'),('runner_path','RUNNER PATH','#93c5fd'),('collisions','COLLISIONS','#fbbf24'),('hold','CAPTURE HOLD','#4ade80'))
        self.cards={}
        for i,(key,label,color) in enumerate(cards): self.cards[key]=MetricCard(label,color); grid.addWidget(self.cards[key],i//2,i%2)
        root.addWidget(QLabel('MATCH PROGRESS')); self.match_progress=QProgressBar(); self.match_progress.setRange(0,1000); root.addWidget(self.match_progress)
        root.addWidget(QLabel('CONTINUOUS CAPTURE HOLD')); self.hold_progress=QProgressBar(); self.hold_progress.setRange(0,1000); self.hold_progress.setStyleSheet(self.hold_progress.styleSheet()+' QProgressBar::chunk{background:#22c55e;border-radius:6px}'); root.addWidget(self.hold_progress)
        modes=QHBoxLayout(); self.cmode=QLabel('CATCHER: --'); self.rmode=QLabel('RUNNER: --'); self.cmode.setStyleSheet('color:#f87171;font-weight:700'); self.rmode.setStyleSheet('color:#60a5fa;font-weight:700'); modes.addWidget(self.cmode); modes.addStretch(); modes.addWidget(self.rmode); root.addLayout(modes)
        self.sensor_line=QLabel('SENSORS: waiting'); self.sensor_line.setWordWrap(True); self.sensor_line.setStyleSheet('color:#94a3b8;font-size:12px'); root.addWidget(self.sensor_line)
        self.timer=QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(100)
    @staticmethod
    def _seconds(value): return f'{float(value):.2f} s'
    @staticmethod
    def _meters(value): return f'{float(value):.3f} m'
    def refresh(self):
        if not rclpy.ok():
            QApplication.instance().quit(); return
        data=self.node.latest
        if data is None:return
        state=data.get('state','WAITING'); colors={'RUNNING':'#0369a1','CAPTURED':'#15803d','SURVIVED':'#1d4ed8'}; self.status.setText(state); self.status.setStyleSheet(f'background:{colors.get(state,"#334155")};border-radius:8px;padding:9px;font-weight:900')
        self.cards['elapsed'].value.setText(self._seconds(data.get('elapsed',0))); self.cards['remaining'].value.setText(self._seconds(data.get('remaining',0)))
        self.cards['separation'].value.setText(self._meters(data.get('separation',0))); self.cards['minimum'].value.setText(self._meters(data.get('minimum_separation',0)))
        self.cards['catcher_speed'].value.setText(f'{data.get("catcher_speed",0):.3f} m/s'); self.cards['runner_speed'].value.setText(f'{data.get("runner_speed",0):.3f} m/s')
        self.cards['catcher_path'].value.setText(self._meters(data.get('catcher_path_length',0))); self.cards['runner_path'].value.setText(self._meters(data.get('runner_path_length',0)))
        self.cards['collisions'].value.setText(str(data.get('collisions',0))); self.cards['hold'].value.setText(f'{data.get("hold",0):.2f} / {data.get("hold_required",1):.2f} s')
        duration=max(.001,float(data.get('duration',180))); self.match_progress.setValue(int(min(1.,data.get('elapsed',0)/duration)*1000)); self.hold_progress.setValue(int(data.get('capture_progress',0)*1000))
        self.cmode.setText('CATCHER: '+data.get('catcher_mode','--')); self.rmode.setText('RUNNER: '+data.get('runner_mode','--'))
        now=time.monotonic(); groups={'C-LIDAR':['/catcher/scan'],'R-LIDAR':['/runner/scan'],'C-RGBD':['/catcher/camera/color/image_raw','/catcher/camera/depth/image_raw'],'R-RGBD':['/runner/camera/color/image_raw','/runner/camera/depth/image_raw']}; statuses=[]
        for name,topics in groups.items(): statuses.append(('● '+name) if all(now-self.node.sensors.get(t,0)<1.5 for t in topics) else ('○ '+name))
        self.sensor_line.setText('SENSORS  '+ '   '.join(statuses)); self.sensor_line.setStyleSheet('color:#4ade80;font-size:12px;font-weight:700' if all(s.startswith('●') for s in statuses) else 'color:#fbbf24;font-size:12px;font-weight:700')


class LidarView(QWidget):
    def __init__(self,color):
        super().__init__(); self.scan=None; self.color=QColor(color); self.setMinimumSize(320,220)
    def paintEvent(self,_):
        painter=QPainter(self); painter.fillRect(self.rect(),QColor('#07101c')); painter.setRenderHint(QPainter.Antialiasing)
        cx=self.width()/2; cy=self.height()/2; scale=min(self.width(),self.height())*.43/5.0
        painter.setPen(QPen(QColor('#334155'),1)); painter.drawEllipse(int(cx-5*scale),int(cy-5*scale),int(10*scale),int(10*scale)); painter.drawLine(int(cx),0,int(cx),self.height()); painter.drawLine(0,int(cy),self.width(),int(cy))
        if self.scan is not None:
            painter.setPen(QPen(self.color,2)); stride=max(1,len(self.scan.ranges)//720); count=0
            for index in range(0,len(self.scan.ranges),stride):
                value=self.scan.ranges[index]
                if not math.isfinite(value) or value<=self.scan.range_min or value>=min(5.0,self.scan.range_max):continue
                angle=self.scan.angle_min+index*self.scan.angle_increment
                x=cx-value*scale*math.sin(angle); y=cy-value*scale*math.cos(angle); painter.drawPoint(int(x),int(y)); count+=1
            painter.setPen(QColor('#cbd5e1')); painter.drawText(8,18,f'{count} valid returns · 5 m view')
        painter.setBrush(self.color); painter.setPen(Qt.NoPen); painter.drawEllipse(int(cx-5),int(cy-5),10,10)


def image_to_qimage(message):
    encoding=message.encoding.lower(); data=bytes(message.data)
    if encoding in ('rgb8','bgr8'):
        image=QImage(data,message.width,message.height,message.step,QImage.Format_RGB888).copy()
        return image.rgbSwapped() if encoding=='bgr8' else image
    if encoding in ('rgba8','bgra8'):
        image=QImage(data,message.width,message.height,message.step,QImage.Format_RGBA8888).copy()
        return image.rgbSwapped() if encoding=='bgra8' else image
    if encoding not in ('32fc1','16uc1','mono16'):return QImage()
    item_size=4 if encoding=='32fc1' else 2
    byte_order='>' if message.is_bigendian else '<'
    dtype=np.dtype(byte_order+('f4' if item_size==4 else 'u2'))
    depth=np.ndarray((message.height,message.width),dtype=dtype,buffer=data,strides=(message.step,item_size))
    sample=max(1,max(message.width//320,message.height//220)); depth=depth[::sample,::sample]
    if item_size==2:depth=depth.astype(np.float32)/1000.0
    valid=np.isfinite(depth)&(depth>0); scaled=np.clip(depth,0.,5.)
    pixels=np.ascontiguousarray(np.where(valid,255.*(1.-scaled/5.),0.).astype(np.uint8))
    return QImage(pixels.data,pixels.shape[1],pixels.shape[0],pixels.strides[0],QImage.Format_Grayscale8).copy()


class SensorFeedWindow(QWidget):
    def __init__(self,node):
        super().__init__(); self.node=node; self.setWindowTitle('Turtle Pursuit — Live Sensor Feeds'); self.resize(1040,780); self.move(510,80)
        self.setStyleSheet('QWidget{background:#0b111b;color:#e8eef7;font-family:Inter,Arial} QLabel#feed{background:#07101c;border:1px solid #26354b;border-radius:8px;color:#64748b;font-weight:700}')
        root=QGridLayout(self); self.views={}; self.map_labels={}; self.displayed={}
        for column,(role,color) in enumerate((('catcher','#ef4444'),('runner','#3b82f6'))):
            heading=QLabel(role.upper()+' SENSOR STACK'); heading.setStyleSheet(f'color:{color};font-size:18px;font-weight:900'); root.addWidget(heading,0,column)
            rgb=QLabel('WAITING FOR RGB'); rgb.setObjectName('feed'); rgb.setAlignment(Qt.AlignCenter); rgb.setMinimumSize(320,220); root.addWidget(rgb,1,column); self.views[(role,'rgb')]=rgb
            depth=QLabel('WAITING FOR DEPTH'); depth.setObjectName('feed'); depth.setAlignment(Qt.AlignCenter); depth.setMinimumSize(320,220); root.addWidget(depth,2,column); self.views[(role,'depth')]=depth
            lidar=LidarView(color); root.addWidget(lidar,3,column); self.views[(role,'lidar')]=lidar
            mapped=QLabel('MAPPED OBSTACLES: waiting'); mapped.setWordWrap(True); mapped.setStyleSheet('color:#a7f3d0;font-family:monospace'); root.addWidget(mapped,4,column); self.map_labels[role]=mapped
        self.timer=QTimer(self); self.timer.setTimerType(Qt.PreciseTimer); self.timer.timeout.connect(self.refresh); self.timer.start(16)
    def refresh(self):
        for role in ('catcher','runner'):
            topics={'rgb':f'/{role}/camera/color/image_raw','depth':f'/{role}/camera/depth/image_raw'}
            for kind,topic in topics.items():
                message=self.node.images.get(topic)
                revision=self.node.revisions.get(topic,0)
                if message is not None and revision!=self.displayed.get(topic):
                    image=image_to_qimage(message)
                    if not image.isNull():
                        self.views[(role,kind)].setPixmap(QPixmap.fromImage(image).scaled(self.views[(role,kind)].size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
                        self.displayed[topic]=revision
            lidar_topic=f'/{role}/scan'; revision=self.node.revisions.get(lidar_topic,0)
            if revision!=self.displayed.get(lidar_topic):
                self.views[(role,'lidar')].scan=self.node.scans.get(lidar_topic); self.views[(role,'lidar')].update(); self.displayed[lidar_topic]=revision
            centers=self.node.obstacles.get(role,[]); formatted='  '.join(f'({float(x):+.2f},{float(y):+.2f})' for x,y in centers[:8])
            rgb_rate=self.node.topic_rate(f'/{role}/camera/color/image_raw'); depth_rate=self.node.topic_rate(f'/{role}/camera/depth/image_raw'); lidar_rate=self.node.topic_rate(lidar_topic)
            self.map_labels[role].setText(f'INPUT  RGB {rgb_rate:4.1f} FPS · DEPTH {depth_rate:4.1f} FPS · LIDAR {lidar_rate:4.1f} Hz\nMAPPED OBSTACLES [{len(centers)}]  {formatted or "none"}')


def main(args=None):
    app=QApplication([]); rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO); app.setStyle('Fusion')
    signal.signal(signal.SIGINT,lambda *_:app.quit()); signal.signal(signal.SIGTERM,lambda *_:app.quit())
    palette=QPalette(); palette.setColor(QPalette.Window,QColor('#0b111b')); palette.setColor(QPalette.WindowText,QColor('#e8eef7')); app.setPalette(palette)
    node=DashboardNode(); executor=SingleThreadedExecutor(); executor.add_node(node)
    ros_thread=threading.Thread(target=executor.spin,name='dashboard-ros',daemon=True); ros_thread.start()
    window=MatchDashboard(node); feeds=SensorFeedWindow(node); window.show(); feeds.show()
    try:return app.exec_()
    finally:
        executor.shutdown(timeout_sec=1.0); ros_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__': sys.exit(main())

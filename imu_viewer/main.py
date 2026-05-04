import sys
import json
import serial
import serial.tools.list_ports
import threading
import time
import os
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QGridLayout, QFrame, QTabWidget, QMessageBox,
                             QProgressBar, QScrollArea)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class IMUDataProcessor(QObject):
    data_received = pyqtSignal(dict)
    status_received = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_running = False
        self.thread = None
        
    def connect_serial(self, port, baudrate=115200):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.is_running = True
            self.thread = threading.Thread(target=self._read_serial, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
            
    def disconnect_serial(self):
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
    def send_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write((cmd + '\n').encode('utf-8'))
            
    def _read_serial(self):
        buffer = ""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('{'):
                            try:
                                msg = json.loads(line)
                                if msg.get('type') == 'imu':
                                    self.data_received.emit(msg)
                                elif msg.get('type') == 'status':
                                    self.status_received.emit(msg)
                                elif msg.get('type') == 'accel_sample':
                                    self.status_received.emit(msg)
                            except json.JSONDecodeError:
                                pass
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)

class CalibrationManager:
    def __init__(self):
        self.reset()
        
    def reset(self):
        self.gyro_bias = [0, 0, 0]
        self.accel_samples = {
            'x_pos': [], 'x_neg': [],
            'y_pos': [], 'y_neg': [],
            'z_pos': [], 'z_neg': []
        }
        self.current_face = None
        self.calib_params = None
        
    def add_accel_sample(self, face, ax, ay, az):
        if face in self.accel_samples:
            self.accel_samples[face].append((ax, ay, az))
            
    def calculate_calibration(self):
        params = {
            'accel_bias': [0, 0, 0],
            'accel_scale': [1, 1, 1],
            'accel_ortho': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            'gyro_bias': self.gyro_bias
        }
        
        faces = ['x_pos', 'x_neg', 'y_pos', 'y_neg', 'z_pos', 'z_neg']
        face_data = {}
        
        for face in faces:
            samples = self.accel_samples[face]
            if len(samples) < 10:
                return None
            avg_ax = sum(s[0] for s in samples) / len(samples)
            avg_ay = sum(s[1] for s in samples) / len(samples)
            avg_az = sum(s[2] for s in samples) / len(samples)
            face_data[face] = (avg_ax, avg_ay, avg_az)
        
        params['accel_bias'][0] = (face_data['x_pos'][0] + face_data['x_neg'][0]) / 2
        params['accel_bias'][1] = (face_data['y_pos'][1] + face_data['y_neg'][1]) / 2
        params['accel_bias'][2] = (face_data['z_pos'][2] + face_data['z_neg'][2]) / 2
        
        ideal_g = 1.0
        sx = abs(face_data['x_pos'][0] - face_data['x_neg'][0]) / (2 * ideal_g)
        sy = abs(face_data['y_pos'][1] - face_data['y_neg'][1]) / (2 * ideal_g)
        sz = abs(face_data['z_pos'][2] - face_data['z_neg'][2]) / (2 * ideal_g)
        
        if sx > 0.5 and sy > 0.5 and sz > 0.5:
            params['accel_scale'][0] = 1.0 / sx
            params['accel_scale'][1] = 1.0 / sy
            params['accel_scale'][2] = 1.0 / sz
        
        ax_ay = (face_data['x_pos'][1] - params['accel_bias'][1]) * params['accel_scale'][1]
        ax_az = (face_data['x_pos'][2] - params['accel_bias'][2]) * params['accel_scale'][2]
        ay_ax = (face_data['y_pos'][0] - params['accel_bias'][0]) * params['accel_scale'][0]
        ay_az = (face_data['y_pos'][2] - params['accel_bias'][2]) * params['accel_scale'][2]
        az_ax = (face_data['z_pos'][0] - params['accel_bias'][0]) * params['accel_scale'][0]
        az_ay = (face_data['z_pos'][1] - params['accel_bias'][1]) * params['accel_scale'][1]
        
        params['accel_ortho'][0][1] = ax_ay
        params['accel_ortho'][0][2] = ax_az
        params['accel_ortho'][1][0] = ay_ax
        params['accel_ortho'][1][2] = ay_az
        params['accel_ortho'][2][0] = az_ax
        params['accel_ortho'][2][1] = az_ay
        
        self.calib_params = params
        return params
        
    def save_to_file(self, filepath):
        if self.calib_params:
            data = {
                'accel_bias': self.calib_params['accel_bias'],
                'accel_scale': self.calib_params['accel_scale'],
                'accel_ortho': self.calib_params['accel_ortho'],
                'gyro_bias': self.calib_params['gyro_bias']
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        return False
        
    def load_from_file(self, filepath):
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                data = json.load(f)
            self.calib_params = data
            return True
        return False
        
    def calculate_calibration_from_faces(self, face_data, gyro_bias):
        required_faces = ['x_pos', 'x_neg', 'y_pos', 'y_neg', 'z_pos', 'z_neg']
        for face in required_faces:
            if face not in face_data:
                return None
                
        params = {
            'accel_bias': [0, 0, 0],
            'accel_scale': [1, 1, 1],
            'accel_ortho': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            'gyro_bias': gyro_bias
        }
        
        params['accel_bias'][0] = (face_data['x_pos'][0] + face_data['x_neg'][0]) / 2
        params['accel_bias'][1] = (face_data['y_pos'][1] + face_data['y_neg'][1]) / 2
        params['accel_bias'][2] = (face_data['z_pos'][2] + face_data['z_neg'][2]) / 2
        
        ideal_g = 1.0
        sx = abs(face_data['x_pos'][0] - face_data['x_neg'][0]) / (2 * ideal_g)
        sy = abs(face_data['y_pos'][1] - face_data['y_neg'][1]) / (2 * ideal_g)
        sz = abs(face_data['z_pos'][2] - face_data['z_neg'][2]) / (2 * ideal_g)
        
        if sx > 0.5 and sy > 0.5 and sz > 0.5:
            params['accel_scale'][0] = 1.0 / sx
            params['accel_scale'][1] = 1.0 / sy
            params['accel_scale'][2] = 1.0 / sz
        
        ax_ay = (face_data['x_pos'][1] - params['accel_bias'][1]) * params['accel_scale'][1]
        ax_az = (face_data['x_pos'][2] - params['accel_bias'][2]) * params['accel_scale'][2]
        ay_ax = (face_data['y_pos'][0] - params['accel_bias'][0]) * params['accel_scale'][0]
        ay_az = (face_data['y_pos'][2] - params['accel_bias'][2]) * params['accel_scale'][2]
        az_ax = (face_data['z_pos'][0] - params['accel_bias'][0]) * params['accel_scale'][0]
        az_ay = (face_data['z_pos'][1] - params['accel_bias'][1]) * params['accel_scale'][1]
        
        params['accel_ortho'][0][1] = ax_ay
        params['accel_ortho'][0][2] = ax_az
        params['accel_ortho'][1][0] = ay_ax
        params['accel_ortho'][1][2] = ay_az
        params['accel_ortho'][2][0] = az_ax
        params['accel_ortho'][2][1] = az_ay
        
        self.calib_params = params
        return params

class MetricCard(QFrame):
    def __init__(self, title, color='#007AFF'):
        super().__init__()
        self.setStyleSheet(f"""
            QFrame {{
                background-color: #FFFFFF;
                border-radius: 12px;
                border: 1px solid #E5E5EA;
            }}
            QFrame:hover {{
                border: 1px solid {color};
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        
        self.title_label = QLabel(title)
        self.title_label.setFont(QFont('SF Pro Text', 10, QFont.Light))
        self.title_label.setStyleSheet('color: #8E8E93;')
        self.title_label.setAlignment(Qt.AlignCenter)
        
        self.value_label = QLabel('0.00')
        self.value_label.setFont(QFont('SF Pro Display', 18, QFont.Bold))
        self.value_label.setStyleSheet(f'color: {color};')
        self.value_label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

class TimeSeriesCanvas(FigureCanvas):
    def __init__(self, parent=None, width=6, height=3, dpi=80):
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor='none')
        
        self.ax_accel = self.fig.add_subplot(311)
        self.ax_accel.set_facecolor('none')
        self.ax_accel.set_ylabel('Accel (g)', fontsize=9, color='#8E8E93')
        self.ax_accel.tick_params(colors='#8E8E93')
        for spine in self.ax_accel.spines.values():
            spine.set_color('#E5E5EA')
        self.ax_accel.grid(True, alpha=0.3, color='#E5E5EA')
        
        self.line_accel_x, = self.ax_accel.plot([], [], '-', color='#007AFF', linewidth=1.5, label='X')
        self.line_accel_y, = self.ax_accel.plot([], [], '-', color='#34C759', linewidth=1.5, label='Y')
        self.line_accel_z, = self.ax_accel.plot([], [], '-', color='#FF9500', linewidth=1.5, label='Z')
        self.ax_accel.legend(loc='upper right', fontsize=7, framealpha=0.8)
        
        self.ax_gyro = self.fig.add_subplot(312, sharex=self.ax_accel)
        self.ax_gyro.set_facecolor('none')
        self.ax_gyro.set_ylabel('Gyro (deg/s)', fontsize=9, color='#8E8E93')
        self.ax_gyro.tick_params(colors='#8E8E93')
        for spine in self.ax_gyro.spines.values():
            spine.set_color('#E5E5EA')
        self.ax_gyro.grid(True, alpha=0.3, color='#E5E5EA')
        
        self.line_gyro_x, = self.ax_gyro.plot([], [], '-', color='#007AFF', linewidth=1.5, label='X')
        self.line_gyro_y, = self.ax_gyro.plot([], [], '-', color='#34C759', linewidth=1.5, label='Y')
        self.line_gyro_z, = self.ax_gyro.plot([], [], '-', color='#FF9500', linewidth=1.5, label='Z')
        self.ax_gyro.legend(loc='upper right', fontsize=7, framealpha=0.8)
        
        self.ax_mag = self.fig.add_subplot(313, sharex=self.ax_accel)
        self.ax_mag.set_facecolor('none')
        self.ax_mag.set_ylabel('Mag (uT)', fontsize=9, color='#8E8E93')
        self.ax_mag.set_xlabel('Time (s)', fontsize=9, color='#8E8E93')
        self.ax_mag.tick_params(colors='#8E8E93')
        for spine in self.ax_mag.spines.values():
            spine.set_color('#E5E5EA')
        self.ax_mag.grid(True, alpha=0.3, color='#E5E5EA')
        
        self.line_mag_x, = self.ax_mag.plot([], [], '-', color='#007AFF', linewidth=1.5, label='X')
        self.line_mag_y, = self.ax_mag.plot([], [], '-', color='#34C759', linewidth=1.5, label='Y')
        self.line_mag_z, = self.ax_mag.plot([], [], '-', color='#FF9500', linewidth=1.5, label='Z')
        self.ax_mag.legend(loc='upper right', fontsize=7, framealpha=0.8)
        
        self.fig.tight_layout(pad=2.0)
        super().__init__(self.fig)
        self.setParent(parent)
        
    def update_data(self, times, accel_data, gyro_data, mag_data):
        if times:
            self.line_accel_x.set_data(times, accel_data['x'])
            self.line_accel_y.set_data(times, accel_data['y'])
            self.line_accel_z.set_data(times, accel_data['z'])
            
            self.line_gyro_x.set_data(times, gyro_data['x'])
            self.line_gyro_y.set_data(times, gyro_data['y'])
            self.line_gyro_z.set_data(times, gyro_data['z'])
            
            self.line_mag_x.set_data(times, mag_data['x'])
            self.line_mag_y.set_data(times, mag_data['y'])
            self.line_mag_z.set_data(times, mag_data['z'])
            
            self.ax_accel.set_xlim([times[0], times[-1]])
            self.ax_gyro.set_xlim([times[0], times[-1]])
            self.ax_mag.set_xlim([times[0], times[-1]])
            
            all_accel = accel_data['x'] + accel_data['y'] + accel_data['z']
            all_gyro = gyro_data['x'] + gyro_data['y'] + gyro_data['z']
            all_mag = mag_data['x'] + mag_data['y'] + mag_data['z']
            
            if all_accel:
                self.ax_accel.set_ylim([min(all_accel) - 0.1, max(all_accel) + 0.1])
            if all_gyro:
                self.ax_gyro.set_ylim([min(all_gyro) - 0.1, max(all_gyro) + 0.1])
            if all_mag:
                self.ax_mag.set_ylim([min(all_mag) - 0.1, max(all_mag) + 0.1])
        
        self.draw_idle()

class CalibrationWidget(QWidget):
    def __init__(self, processor, calib_manager):
        super().__init__()
        self.processor = processor
        self.calib_manager = calib_manager
        self.calib_active = False
        self.collected_faces = set()
        self.face_data = {}
        self.gyro_bias = [0, 0, 0]
        
        self.face_names = {
            'x_pos': '+X', 'x_neg': '-X',
            'y_pos': '+Y', 'y_neg': '-Y',
            'z_pos': '+Z', 'z_neg': '-Z'
        }
        
        self.init_ui()
        self.processor.status_received.connect(self.handle_status)
        self.processor.data_received.connect(self.handle_data)
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)
        
        title = QLabel('IMU Calibration')
        title.setFont(QFont('SF Pro Display', 20, QFont.Bold))
        title.setStyleSheet('color: #1C1C1E;')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        self.status_label = QLabel('Click Start to begin calibration')
        self.status_label.setFont(QFont('SF Pro Display', 16, QFont.Bold))
        self.status_label.setStyleSheet('color: #007AFF;')
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        self.desc_label = QLabel('Place IMU in 6 different orientations')
        self.desc_label.setFont(QFont('SF Pro Text', 12))
        self.desc_label.setStyleSheet('color: #8E8E93;')
        self.desc_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.desc_label)
        
        self.progress = QProgressBar()
        self.progress.setMaximum(6)
        self.progress.setValue(0)
        self.progress.setFormat('%v/6 faces collected')
        self.progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid #E5E5EA;
                border-radius: 5px;
                text-align: center;
                height: 25px;
            }
            QProgressBar::chunk {
                background-color: #007AFF;
                border-radius: 5px;
            }
        """)
        layout.addWidget(self.progress)
        
        self.face_grid = QGridLayout()
        self.face_grid.setSpacing(10)
        
        self.face_labels = {}
        faces = [('+X', 0, 0), ('-X', 0, 1), ('+Y', 1, 0), ('-Y', 1, 1), ('+Z', 2, 0), ('-Z', 2, 1)]
        for name, row, col in faces:
            label = QLabel(f'{name}: Not collected')
            label.setFont(QFont('SF Pro Text', 12))
            label.setStyleSheet('color: #8E8E93; background-color: #F2F2F7; border-radius: 8px; padding: 8px;')
            label.setAlignment(Qt.AlignCenter)
            self.face_labels[name] = label
            self.face_grid.addWidget(label, row, col)
        
        layout.addLayout(self.face_grid)
        
        self.ax_label = QLabel('AX: 0.000')
        self.ax_label.setFont(QFont('SF Pro Text', 14))
        self.ax_label.setStyleSheet('color: #007AFF;')
        self.ax_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.ax_label)
        
        self.ay_label = QLabel('AY: 0.000')
        self.ay_label.setFont(QFont('SF Pro Text', 14))
        self.ay_label.setStyleSheet('color: #34C759;')
        self.ay_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.ay_label)
        
        self.az_label = QLabel('AZ: 0.000')
        self.az_label.setFont(QFont('SF Pro Text', 14))
        self.az_label.setStyleSheet('color: #FF9500;')
        self.az_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.az_label)
        
        self.start_btn = QPushButton('Start Calibration')
        self.start_btn.setFixedHeight(40)
        self.start_btn.setFont(QFont('SF Pro Text', 14, QFont.Bold))
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #007AFF;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #0051D5;
            }
        """)
        self.start_btn.clicked.connect(self.start_calibration)
        layout.addWidget(self.start_btn)
        
        self.save_btn = QPushButton('Save Calibration')
        self.save_btn.setFixedHeight(40)
        self.save_btn.setFont(QFont('SF Pro Text', 14, QFont.Bold))
        self.save_btn.setStyleSheet("""
            QPushButton {
                background-color: #34C759;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #2DB84E;
            }
        """)
        self.save_btn.clicked.connect(self.save_calibration)
        layout.addWidget(self.save_btn)
        
        self.load_btn = QPushButton('Load Calibration')
        self.load_btn.setFixedHeight(40)
        self.load_btn.setFont(QFont('SF Pro Text', 14, QFont.Bold))
        self.load_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9500;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #E08600;
            }
        """)
        self.load_btn.clicked.connect(self.load_calibration)
        layout.addWidget(self.load_btn)
        
        layout.addStretch()
        
    def start_calibration(self):
        self.calib_active = True
        self.collected_faces = set()
        self.face_data = {}
        self.gyro_bias = [0, 0, 0]
        self.progress.setValue(0)
        
        for name in self.face_labels:
            self.face_labels[name].setText(f'{name}: Not collected')
            self.face_labels[name].setStyleSheet('color: #8E8E93; background-color: #F2F2F7; border-radius: 8px; padding: 8px;')
        
        self.status_label.setText('Collecting data...')
        self.status_label.setStyleSheet('color: #007AFF;')
        self.desc_label.setText('Place IMU in different orientations')
        self.start_btn.setEnabled(False)
        
        self.processor.send_command('{"type":"calib_start"}')
        
    def handle_status(self, msg):
        if msg.get('msg') == 'calib_started':
            self.status_label.setText('Collecting data...')
        elif msg.get('msg') == 'face_collected':
            face = msg.get('face', '')
            progress = msg.get('progress', 0)
            self.progress.setValue(progress)
            
            if face in self.face_labels:
                self.face_labels[face].setText(f'{face}: Collected')
                self.face_labels[face].setStyleSheet('color: #34C759; background-color: #E8F5E9; border-radius: 8px; padding: 8px;')
        elif msg.get('msg') == 'calib_complete':
            self.calib_active = False
            self.gyro_bias = [msg.get('gx', 0), msg.get('gy', 0), msg.get('gz', 0)]
            
            self.face_data = {
                'x_pos': (msg.get('x_pos_ax', 0), msg.get('x_pos_ay', 0), msg.get('x_pos_az', 0)),
                'x_neg': (msg.get('x_neg_ax', 0), msg.get('x_neg_ay', 0), msg.get('x_neg_az', 0)),
                'y_pos': (msg.get('y_pos_ax', 0), msg.get('y_pos_ay', 0), msg.get('y_pos_az', 0)),
                'y_neg': (msg.get('y_neg_ax', 0), msg.get('y_neg_ay', 0), msg.get('y_neg_az', 0)),
                'z_pos': (msg.get('z_pos_ax', 0), msg.get('z_pos_ay', 0), msg.get('z_pos_az', 0)),
                'z_neg': (msg.get('z_neg_ax', 0), msg.get('z_neg_ay', 0), msg.get('z_neg_az', 0)),
            }
            
            self.calculate_and_apply()
            
    def handle_data(self, data):
        if self.calib_active:
            self.ax_label.setText(f"AX: {data.get('ax', 0):.3f}")
            self.ay_label.setText(f"AY: {data.get('ay', 0):.3f}")
            self.az_label.setText(f"AZ: {data.get('az', 0):.3f}")
            
    def calculate_and_apply(self):
        params = self.calib_manager.calculate_calibration_from_faces(self.face_data, self.gyro_bias)
        if params:
            cmd = '{{"type":"apply_calib","sx":{:.4f},"sy":{:.4f},"sz":{:.4f},"ox":{:.4f},"oy":{:.4f},"oz":{:.4f},"gb_x":{:.4f},"gb_y":{:.4f},"gb_z":{:.4f},"ab_x":{:.4f},"ab_y":{:.4f},"ab_z":{:.4f}}}'.format(
                params['accel_scale'][0], params['accel_scale'][1], params['accel_scale'][2],
                params['accel_ortho'][0][1], params['accel_ortho'][0][2], params['accel_ortho'][1][2],
                params['gyro_bias'][0], params['gyro_bias'][1], params['gyro_bias'][2],
                params['accel_bias'][0], params['accel_bias'][1], params['accel_bias'][2]
            )
            self.processor.send_command(cmd)
            
            self.status_label.setText('Calibration Complete!')
            self.status_label.setStyleSheet('color: #34C759;')
            self.desc_label.setText('IMU is now calibrated')
            self.start_btn.setEnabled(True)
            self.start_btn.setText('Recalibrate')
            
            QMessageBox.information(self, 'Success', 'Calibration completed and applied!')
        else:
            self.status_label.setText('Calibration Failed')
            self.status_label.setStyleSheet('color: #FF3B30;')
            self.desc_label.setText('Please try again')
            self.start_btn.setEnabled(True)
            self.start_btn.setText('Retry')
            
            QMessageBox.warning(self, 'Error', 'Not enough data. Please try again.')
            
    def save_calibration(self):
        from PyQt5.QtWidgets import QFileDialog
        filepath, _ = QFileDialog.getSaveFileName(self, 'Save Calibration', '', 'JSON Files (*.json)')
        if filepath:
            if self.calib_manager.save_to_file(filepath):
                QMessageBox.information(self, 'Success', 'Calibration saved!')
            else:
                QMessageBox.warning(self, 'Error', 'No calibration data to save.')
                
    def load_calibration(self):
        from PyQt5.QtWidgets import QFileDialog
        filepath, _ = QFileDialog.getOpenFileName(self, 'Load Calibration', '', 'JSON Files (*.json)')
        if filepath:
            if self.calib_manager.load_from_file(filepath):
                params = self.calib_manager.calib_params
                cmd = '{{"type":"apply_calib","sx":{:.4f},"sy":{:.4f},"sz":{:.4f},"ox":{:.4f},"oy":{:.4f},"oz":{:.4f},"gb_x":{:.4f},"gb_y":{:.4f},"gb_z":{:.4f},"ab_x":{:.4f},"ab_y":{:.4f},"ab_z":{:.4f}}}'.format(
                    params['accel_scale'][0], params['accel_scale'][1], params['accel_scale'][2],
                    params['accel_ortho'][0][1], params['accel_ortho'][0][2], params['accel_ortho'][1][2],
                    params['gyro_bias'][0], params['gyro_bias'][1], params['gyro_bias'][2],
                    params['accel_bias'][0], params['accel_bias'][1], params['accel_bias'][2]
                )
                self.processor.send_command(cmd)
                QMessageBox.information(self, 'Success', 'Calibration loaded and applied!')
            else:
                QMessageBox.warning(self, 'Error', 'Failed to load calibration file.')

class IMUViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('IMU Viewer')
        self.setMinimumSize(1200, 800)
        self.setStyleSheet("QMainWindow { background-color: #F2F2F7; }")
        
        self.processor = IMUDataProcessor()
        self.processor.data_received.connect(self.update_data)
        self.calib_manager = CalibrationManager()
        
        self.time_window = 5.0
        self.max_samples = 200
        self.time_history = deque(maxlen=self.max_samples)
        self.accel_history = {'x': deque(maxlen=self.max_samples), 
                             'y': deque(maxlen=self.max_samples), 
                             'z': deque(maxlen=self.max_samples)}
        self.gyro_history = {'x': deque(maxlen=self.max_samples), 
                            'y': deque(maxlen=self.max_samples), 
                            'z': deque(maxlen=self.max_samples)}
        self.mag_history = {'x': deque(maxlen=self.max_samples), 
                           'y': deque(maxlen=self.max_samples), 
                           'z': deque(maxlen=self.max_samples)}
        
        self.start_time = None
        self.last_data = None
        
        self.init_ui()
        self.setup_timer()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        header = self.create_header()
        main_layout.addWidget(header)
        
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #E5E5EA;
                border-radius: 12px;
                background-color: #FFFFFF;
            }
            QTabBar::tab {
                background-color: #F2F2F7;
                border: 1px solid #E5E5EA;
                border-bottom: none;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                padding: 8px 16px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #FFFFFF;
                border-bottom: 2px solid #007AFF;
            }
        """)
        
        monitor_tab = self.create_monitor_tab()
        calib_widget = CalibrationWidget(self.processor, self.calib_manager)
        
        self.tabs.addTab(monitor_tab, 'Monitor')
        self.tabs.addTab(calib_widget, 'Calibration')
        
        main_layout.addWidget(self.tabs)
        
    def create_header(self):
        header = QFrame()
        header.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border-radius: 16px;
                border: 1px solid #E5E5EA;
            }
        """)
        header.setFixedHeight(60)
        layout = QHBoxLayout(header)
        layout.setContentsMargins(20, 10, 20, 10)
        
        title = QLabel('IMU Monitor')
        title.setFont(QFont('SF Pro Display', 20, QFont.Bold))
        title.setStyleSheet('color: #1C1C1E;')
        layout.addWidget(title)
        
        layout.addStretch()
        
        port_layout = QHBoxLayout()
        port_label = QLabel('Port:')
        port_label.setFont(QFont('SF Pro Text', 12))
        port_label.setStyleSheet('color: #8E8E93;')
        port_layout.addWidget(port_label)
        
        self.port_combo = QComboBox()
        self.port_combo.setFont(QFont('SF Pro Text', 12))
        self.port_combo.setFixedWidth(120)
        self.refresh_ports()
        port_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton('Refresh')
        refresh_btn.setFixedHeight(32)
        refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #F2F2F7;
                border-radius: 8px;
                color: #007AFF;
            }
            QPushButton:hover {
                background-color: #E5E5EA;
            }
        """)
        refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(refresh_btn)
        
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.setFixedHeight(32)
        self.connect_btn.setFont(QFont('SF Pro Text', 12, QFont.Bold))
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background-color: #007AFF;
                color: white;
                border-radius: 8px;
                padding: 0 16px;
            }
            QPushButton:hover {
                background-color: #0051D5;
            }
        """)
        self.connect_btn.clicked.connect(self.toggle_connection)
        port_layout.addWidget(self.connect_btn)
        
        layout.addLayout(port_layout)
        return header
        
    def create_monitor_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)
        
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        title = QLabel('Real-time Data (5 seconds)')
        title.setFont(QFont('SF Pro Display', 16, QFont.Bold))
        title.setStyleSheet('color: #1C1C1E;')
        left_layout.addWidget(title)
        
        self.timeseries_canvas = TimeSeriesCanvas(self, width=8, height=5, dpi=80)
        left_layout.addWidget(self.timeseries_canvas)
        
        layout.addWidget(left_panel, 2)
        
        right_panel = self.create_data_cards()
        layout.addWidget(right_panel, 1)
        
        return tab
        
    def create_data_cards(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(15)
        
        accel_card = self.create_accel_card()
        layout.addWidget(accel_card)
        
        gyro_card = self.create_gyro_card()
        layout.addWidget(gyro_card)
        
        mag_card = self.create_mag_card()
        layout.addWidget(mag_card)
        
        temp_card = self.create_temp_card()
        layout.addWidget(temp_card)
        
        layout.addStretch()
        return panel
        
    def create_accel_card(self):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border-radius: 16px;
                border: 1px solid #E5E5EA;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(10)
        
        header = QLabel('Accelerometer (g)')
        header.setFont(QFont('SF Pro Display', 14, QFont.Bold))
        header.setStyleSheet('color: #1C1C1E;')
        layout.addWidget(header)
        
        grid = QGridLayout()
        grid.setSpacing(10)
        
        self.accel_x = MetricCard('X', '#007AFF')
        self.accel_y = MetricCard('Y', '#34C759')
        self.accel_z = MetricCard('Z', '#FF9500')
        
        grid.addWidget(self.accel_x, 0, 0)
        grid.addWidget(self.accel_y, 0, 1)
        grid.addWidget(self.accel_z, 0, 2)
        
        layout.addLayout(grid)
        return card
        
    def create_gyro_card(self):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border-radius: 16px;
                border: 1px solid #E5E5EA;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(10)
        
        header = QLabel('Gyroscope (deg/s)')
        header.setFont(QFont('SF Pro Display', 14, QFont.Bold))
        header.setStyleSheet('color: #1C1C1E;')
        layout.addWidget(header)
        
        grid = QGridLayout()
        grid.setSpacing(10)
        
        self.gyro_x = MetricCard('X', '#007AFF')
        self.gyro_y = MetricCard('Y', '#34C759')
        self.gyro_z = MetricCard('Z', '#FF9500')
        
        grid.addWidget(self.gyro_x, 0, 0)
        grid.addWidget(self.gyro_y, 0, 1)
        grid.addWidget(self.gyro_z, 0, 2)
        
        layout.addLayout(grid)
        return card
        
    def create_mag_card(self):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border-radius: 16px;
                border: 1px solid #E5E5EA;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(10)
        
        header = QLabel('Compass (uT)')
        header.setFont(QFont('SF Pro Display', 14, QFont.Bold))
        header.setStyleSheet('color: #1C1C1E;')
        layout.addWidget(header)
        
        grid = QGridLayout()
        grid.setSpacing(10)
        
        self.mag_x = MetricCard('X', '#007AFF')
        self.mag_y = MetricCard('Y', '#34C759')
        self.mag_z = MetricCard('Z', '#FF9500')
        
        grid.addWidget(self.mag_x, 0, 0)
        grid.addWidget(self.mag_y, 0, 1)
        grid.addWidget(self.mag_z, 0, 2)
        
        layout.addLayout(grid)
        return card
        
    def create_temp_card(self):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border-radius: 16px;
                border: 1px solid #E5E5EA;
            }
        """)
        layout = QHBoxLayout(card)
        layout.setContentsMargins(15, 12, 15, 12)
        
        temp_label = QLabel('Temperature')
        temp_label.setFont(QFont('SF Pro Display', 14, QFont.Bold))
        temp_label.setStyleSheet('color: #1C1C1E;')
        layout.addWidget(temp_label)
        
        layout.addStretch()
        
        self.temp_value = QLabel('0.00 C')
        self.temp_value.setFont(QFont('SF Pro Display', 18, QFont.Bold))
        self.temp_value.setStyleSheet('color: #34C759;')
        layout.addWidget(self.temp_value)
        
        return card
        
    def setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)
        
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
            
    def toggle_connection(self):
        if self.processor.is_running:
            self.processor.disconnect_serial()
            self.connect_btn.setText('Connect')
            self.connect_btn.setStyleSheet("""
                QPushButton {
                    background-color: #007AFF;
                    color: white;
                    border-radius: 8px;
                    padding: 0 16px;
                }
                QPushButton:hover {
                    background-color: #0051D5;
                }
            """)
        else:
            port = self.port_combo.currentText()
            if port:
                if self.processor.connect_serial(port):
                    self.connect_btn.setText('Disconnect')
                    self.connect_btn.setStyleSheet("""
                        QPushButton {
                            background-color: #FF3B30;
                            color: white;
                            border-radius: 8px;
                            padding: 0 16px;
                        }
                        QPushButton:hover {
                            background-color: #D62D24;
                        }
                    """)
                    self.start_time = time.time()
                    
    def update_data(self, data):
        self.last_data = data
        
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        ax = data.get('ax', 0)
        ay = data.get('ay', 0)
        az = data.get('az', 0)
        gx = data.get('gx', 0)
        gy = data.get('gy', 0)
        gz = data.get('gz', 0)
        mx = data.get('mx', 0)
        my = data.get('my', 0)
        mz = data.get('mz', 0)
        
        self.time_history.append(current_time)
        self.accel_history['x'].append(ax)
        self.accel_history['y'].append(ay)
        self.accel_history['z'].append(az)
        self.gyro_history['x'].append(gx)
        self.gyro_history['y'].append(gy)
        self.gyro_history['z'].append(gz)
        self.mag_history['x'].append(mx)
        self.mag_history['y'].append(my)
        self.mag_history['z'].append(mz)
        
        calib_widget = self.tabs.widget(1)
        if isinstance(calib_widget, CalibrationWidget) and hasattr(calib_widget, 'handle_data'):
            calib_widget.handle_data(data)
        
    def update_ui(self):
        if self.last_data:
            data = self.last_data
            self.accel_x.value_label.setText(f"{data.get('ax', 0):.3f}")
            self.accel_y.value_label.setText(f"{data.get('ay', 0):.3f}")
            self.accel_z.value_label.setText(f"{data.get('az', 0):.3f}")
            self.gyro_x.value_label.setText(f"{data.get('gx', 0):.3f}")
            self.gyro_y.value_label.setText(f"{data.get('gy', 0):.3f}")
            self.gyro_z.value_label.setText(f"{data.get('gz', 0):.3f}")
            self.mag_x.value_label.setText(f"{data.get('mx', 0):.2f}")
            self.mag_y.value_label.setText(f"{data.get('my', 0):.2f}")
            self.mag_z.value_label.setText(f"{data.get('mz', 0):.2f}")
            self.temp_value.setText(f"{data.get('temp', 0):.1f} C")
            
            if len(self.time_history) > 1:
                times = list(self.time_history)
                start_t = times[-1] - self.time_window
                valid_idx = [i for i, t in enumerate(times) if t >= start_t]
                
                if valid_idx:
                    filtered_times = [times[i] - start_t for i in valid_idx]
                    filtered_accel = {
                        'x': [list(self.accel_history['x'])[i] for i in valid_idx],
                        'y': [list(self.accel_history['y'])[i] for i in valid_idx],
                        'z': [list(self.accel_history['z'])[i] for i in valid_idx]
                    }
                    filtered_gyro = {
                        'x': [list(self.gyro_history['x'])[i] for i in valid_idx],
                        'y': [list(self.gyro_history['y'])[i] for i in valid_idx],
                        'z': [list(self.gyro_history['z'])[i] for i in valid_idx]
                    }
                    filtered_mag = {
                        'x': [list(self.mag_history['x'])[i] for i in valid_idx],
                        'y': [list(self.mag_history['y'])[i] for i in valid_idx],
                        'z': [list(self.mag_history['z'])[i] for i in valid_idx]
                    }
                    self.timeseries_canvas.update_data(filtered_times, filtered_accel, filtered_gyro, filtered_mag)

if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    app.setFont(QFont('SF Pro Text', 12))
    window = IMUViewer()
    window.show()
    sys.exit(app.exec_())

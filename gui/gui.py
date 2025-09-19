import sys
import numpy as np
import asyncio
from collections import deque
import time
import struct
from bleak import BleakScanner, BleakClient
import qasync

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, QFrame)
from PyQt6.QtCore import QTimer, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont, QPalette, QColor

import pyqtgraph as pg

from utils import ArmMotionTracker

# Configuration
CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"
CALIBRATION_TIME = 5
DATA_BUFFER_SIZE = 500

class BLEManager(QObject):
    """Simple BLE manager for handling connections"""
    data_received = pyqtSignal(object)
    connection_status = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.client = None
        self.is_connected = False
        
    def handle_data(self, sender, data):
        """Handle incoming BLE data"""
        try:
            raw_quat_data = struct.unpack('<12d', data)
            self.data_received.emit(raw_quat_data)
        except Exception as e:
            print(f"Error unpacking data: {e}")
    
    async def connect(self):
        """Connect to ESP32 device"""
        try:
            self.connection_status.emit("Scanning...")
            devices = await BleakScanner.discover(timeout=5.0)
            esp32 = None
            
            for device in devices:
                if device.name == "ESP32_Hello":
                    esp32 = device
                    break
            
            if not esp32:
                self.connection_status.emit("ESP32 not found!")
                return False
                
            self.connection_status.emit(f"Connecting to {esp32.address}")
            
            self.client = BleakClient(esp32)
            await self.client.connect()
            await self.client.start_notify(CHAR_UUID, self.handle_data)
            
            self.is_connected = True
            self.connection_status.emit("Connected!")
            return True
            
        except Exception as e:
            self.connection_status.emit(f"Connection error: {str(e)}")
            return False
    
    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.is_connected:
            try:
                await self.client.disconnect()
                self.is_connected = False
                self.connection_status.emit("Disconnected")
            except Exception as e:
                print(f"Disconnect error: {e}")

class MotionTrackingGUI(QMainWindow):
    """Main GUI for real-time motion tracking visualization"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize motion tracker
        self.motion_tracker = ArmMotionTracker(calibration_time=CALIBRATION_TIME, verbose=False)
        self.motion_tracker.set_active_body_parts(upper_arm=True, elbow=True, wrist=False)
        
        # Data buffers for plotting (last 100 points)
        self.data_buffers = {
            'upper_arm_x': deque(maxlen=DATA_BUFFER_SIZE),
            'upper_arm_y': deque(maxlen=DATA_BUFFER_SIZE),
            'upper_arm_z': deque(maxlen=DATA_BUFFER_SIZE),
            'elbow_flexion': deque(maxlen=DATA_BUFFER_SIZE),
            'time': deque(maxlen=DATA_BUFFER_SIZE)
        }
        
        # Current angle values
        self.current_angles = {
            'upper_arm_x': 0.0,
            'upper_arm_y': 0.0,
            'upper_arm_z': 0.0,
            'elbow_flexion': 0.0
        }
        
        self.start_time = time.time()
        
        # Setup UI
        self.setup_ui()
        self.setup_plots()
        
        # Setup BLE manager
        self.ble_manager = BLEManager()
        self.ble_manager.data_received.connect(self.process_motion_data)
        self.ble_manager.connection_status.connect(self.update_connection_status)
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(50)  # Update every 50ms (20 FPS)
    
    def setup_ui(self):
        """Setup the main UI layout"""
        self.setWindowTitle("ARM Motion Tracking - Real-time IMU Visualization")
        self.setGeometry(100, 100, 1400, 800)
        
        # Set dark theme
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; color: white; }
            QLabel { color: white; font-size: 12px; }
            QFrame { background-color: #3c3c3c; border: 1px solid #555; border-radius: 5px; }
            QPushButton { background-color: #4a4a4a; border: 1px solid #666; padding: 5px; border-radius: 3px; }
            QPushButton:hover { background-color: #5a5a5a; }
        """)
        
        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left side: Graphs
        graphs_widget = QWidget()
        graphs_layout = QGridLayout(graphs_widget)
        
        # Create graph frames
        self.create_graph_frame("Upper Arm X-Axis (Shoulder Rotation)", "upper_arm_x", graphs_layout, 0, 0)
        self.create_graph_frame("Upper Arm Y-Axis (Shoulder Elevation)", "upper_arm_y", graphs_layout, 0, 1)
        self.create_graph_frame("Upper Arm Z-Axis (Shoulder Abduction)", "upper_arm_z", graphs_layout, 1, 0)
        self.create_graph_frame("Elbow Flexion/Extension", "elbow_flexion", graphs_layout, 1, 1)
        
        # Right side: Info panel
        info_widget = self.create_info_panel()
        
        # Add to main layout
        main_layout.addWidget(graphs_widget, 3)  # 75% width
        main_layout.addWidget(info_widget, 1)    # 25% width
    
    def create_graph_frame(self, title, key, layout, row, col):
        """Create a framed graph widget"""
        frame = QFrame()
        frame.setFrameStyle(QFrame.Shape.StyledPanel)
        frame_layout = QVBoxLayout(frame)
        
        # Title
        title_label = QLabel(title)
        title_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        frame_layout.addWidget(title_label)
        
        # Plot widget
        plot_widget = pg.PlotWidget()
        plot_widget.setBackground('#2b2b2b')
        plot_widget.setLabel('left', 'Angle (degrees)', color='white')
        plot_widget.setLabel('bottom', 'Time (s)', color='white')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # Store reference
        setattr(self, f'plot_{key}', plot_widget)
        
        frame_layout.addWidget(plot_widget)
        layout.addWidget(frame, row, col)
    
    def create_info_panel(self):
        """Create the information panel"""
        info_frame = QFrame()
        info_frame.setFrameStyle(QFrame.Shape.StyledPanel)
        info_layout = QVBoxLayout(info_frame)
        
        # Title
        title = QLabel("Real-time Values")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        info_layout.addWidget(title)
        
        # Calibration status
        self.calibration_label = QLabel("Status: Initializing...")
        self.calibration_label.setFont(QFont("Arial", 12))
        info_layout.addWidget(self.calibration_label)
        
        # Connection status
        self.connection_label = QLabel("Connection: Disconnected")
        self.connection_label.setFont(QFont("Arial", 10))
        info_layout.addWidget(self.connection_label)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        info_layout.addWidget(separator)
        
        # Current values
        info_layout.addWidget(QLabel("Current Angles:"))
        
        self.angle_labels = {}
        angle_names = [
            ('upper_arm_x', 'Shoulder Rotation'),
            ('upper_arm_y', 'Shoulder Elevation'),
            ('upper_arm_z', 'Shoulder Abduction'),
            ('elbow_flexion', 'Elbow Flexion/Extension')
        ]
        
        for key, name in angle_names:
            label = QLabel(f"{name}: 0.0°")
            label.setFont(QFont("Courier", 11))
            self.angle_labels[key] = label
            info_layout.addWidget(label)
        
        # Add spacer
        info_layout.addStretch()
        
        # Control buttons
        self.start_button = QPushButton("Start Connection")
        self.start_button.clicked.connect(self.start_connection)
        info_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Connection")
        self.stop_button.clicked.connect(self.stop_connection)
        self.stop_button.setEnabled(False)
        info_layout.addWidget(self.stop_button)
        
        return info_frame
    
    def setup_plots(self):
        """Initialize plot curves"""
        self.curves = {}
        plot_colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#96ceb4']
        
        for i, key in enumerate(['upper_arm_x', 'upper_arm_y', 'upper_arm_z', 'elbow_flexion']):
            plot_widget = getattr(self, f'plot_{key}')
            curve = plot_widget.plot(pen=pg.mkPen(color=plot_colors[i], width=2))
            self.curves[key] = curve
    
    def start_connection(self):
        """Start BLE connection"""
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
        # Create async task for connection
        asyncio.create_task(self.ble_manager.connect())
    
    def stop_connection(self):
        """Stop BLE connection"""
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        
        # Create async task for disconnection
        if self.ble_manager.is_connected:
            asyncio.create_task(self.ble_manager.disconnect())
        else:
            self.connection_label.setText("Connection: Disconnected")
    
    def update_connection_status(self, status):
        """Update connection status display"""
        self.connection_label.setText(f"Connection: {status}")
    
    def process_motion_data(self, raw_data):
        """Process incoming motion data"""
        # Process through motion tracker
        joint_measurements = self.motion_tracker.process_frame(raw_data)
        
        # Update calibration status
        if not self.motion_tracker.is_calibrated():
            progress = self.motion_tracker.get_calibration_progress()
            self.calibration_label.setText(f"Status: Calibrating... {progress:.1f}%")
            self.calibration_label.setStyleSheet("color: orange;")
        else:
            self.calibration_label.setText("Status: Calibrated ✓")
            self.calibration_label.setStyleSheet("color: green;")
        
        # Extract angle data if available
        if joint_measurements and self.motion_tracker.is_calibrated():
            current_time = time.time() - self.start_time
            self.data_buffers['time'].append(current_time)
            
            # Extract upper arm angles
            if 'upper_arm' in joint_measurements:
                upper_arm = joint_measurements['upper_arm']
                self.current_angles['upper_arm_x'] = upper_arm['shoulder_rotation']
                self.current_angles['upper_arm_y'] = upper_arm['shoulder_elevation']
                self.current_angles['upper_arm_z'] = upper_arm['shoulder_abduction']
                
                self.data_buffers['upper_arm_x'].append(upper_arm['shoulder_rotation'])
                self.data_buffers['upper_arm_y'].append(upper_arm['shoulder_elevation'])
                self.data_buffers['upper_arm_z'].append(upper_arm['shoulder_abduction'])
            else:
                # Add zeros to maintain buffer sync
                for key in ['upper_arm_x', 'upper_arm_y', 'upper_arm_z']:
                    self.data_buffers[key].append(0.0)
                    self.current_angles[key] = 0.0
            
            # Extract elbow angles
            if 'elbow' in joint_measurements:
                elbow = joint_measurements['elbow']
                self.current_angles['elbow_flexion'] = elbow['flexion_extension']
                self.data_buffers['elbow_flexion'].append(elbow['flexion_extension'])
            else:
                self.data_buffers['elbow_flexion'].append(0.0)
                self.current_angles['elbow_flexion'] = 0.0
            
            # Update info panel
            self.update_info_display()
    
    def update_info_display(self):
        """Update the information display with current values"""
        angle_names = [
            ('upper_arm_x', 'Shoulder Rotation'),
            ('upper_arm_y', 'Shoulder Elevation'),
            ('upper_arm_z', 'Shoulder Abduction'),
            ('elbow_flexion', 'Elbow Flexion/Extension')
        ]
        
        for key, name in angle_names:
            value = self.current_angles[key]
            self.angle_labels[key].setText(f"{name}: {value:+6.1f}°")
    
    def update_plots(self):
        """Update all plots with current data"""
        if len(self.data_buffers['time']) < 2:
            return
        
        time_data = np.array(self.data_buffers['time'])
        
        # Update each plot
        for key in ['upper_arm_x', 'upper_arm_y', 'upper_arm_z', 'elbow_flexion']:
            if key in self.curves:
                angle_data = np.array(self.data_buffers[key])
                self.curves[key].setData(time_data, angle_data)
    
    def closeEvent(self, event):
        """Handle application close"""
        if self.ble_manager.is_connected:
            asyncio.create_task(self.ble_manager.disconnect())
        event.accept()

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Setup async event loop
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    # Create and show main window
    window = MotionTrackingGUI()
    window.show()
    
    # Run the application
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()

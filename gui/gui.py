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
import pyqtgraph.opengl as gl

from utils import ArmMotionTracker

# Configuration
CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"
CALIBRATION_TIME = 5
DATA_BUFFER_SIZE = 500

class Arm3DModel(QObject):
    """3D Arm visualization using PyQtGraph OpenGL"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create 3D view widget  
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=35, elevation=15, azimuth=30)  # Better framing for arm proportions
        self.view.setBackgroundColor('k')  # Black background
        
        # Arm segment dimensions (realistic proportions)
        self.upper_arm_length = 12.0  # Slightly shorter for better proportions
        self.forearm_length = 10.0    # Proportional to upper arm
        self.bone_radius = 1.2        # Thicker for better visibility
        self.joint_radius = 1.8       # Prominent joint visualization
        
        # Initialize arm geometry
        self.setup_arm_geometry()
        
        # Current rotations (will be updated from IMU data)
        self.upper_arm_rotation = np.array([1, 0, 0, 0])  # Identity quaternion
        self.forearm_rotation = np.array([1, 0, 0, 0])    # Identity quaternion
        
    def setup_arm_geometry(self):
        """Create the 3D arm model geometry"""
        # Create shoulder reference point (proportional sphere)
        shoulder_radius = self.bone_radius * 0.8  # Smaller than bone radius
        shoulder_sphere = gl.MeshData.sphere(rows=10, cols=20, radius=shoulder_radius)
        self.shoulder = gl.GLMeshItem(meshdata=shoulder_sphere, color=(0.9, 0.9, 0.9, 1.0))
        self.shoulder.translate(0, 0, 0)  # Origin point
        self.view.addItem(self.shoulder)
        
        # Create upper arm (cylinder extending forward along X-axis)
        # Slight taper from shoulder to elbow for realism
        upper_arm_mesh = gl.MeshData.cylinder(rows=24, cols=24, 
                                              radius=[self.bone_radius * 1.1, self.bone_radius * 0.9], 
                                              length=self.upper_arm_length)
        self.upper_arm = gl.GLMeshItem(meshdata=upper_arm_mesh, color=(1.0, 0.5, 0.5, 1.0))  # Red
        self.view.addItem(self.upper_arm)
        
        # Create elbow joint (sphere at end of upper arm)
        elbow_sphere = gl.MeshData.sphere(rows=12, cols=24, radius=self.joint_radius)
        self.elbow = gl.GLMeshItem(meshdata=elbow_sphere, color=(0.4, 0.4, 1.0, 1.0))  # Blue
        self.view.addItem(self.elbow)
        
        # Create forearm (cylinder extending forward from elbow along X-axis)
        # Slight taper from elbow to wrist
        forearm_mesh = gl.MeshData.cylinder(rows=20, cols=24, 
                                           radius=[self.bone_radius * 0.9, self.bone_radius * 0.7], 
                                           length=self.forearm_length)
        self.forearm = gl.GLMeshItem(meshdata=forearm_mesh, color=(0.4, 1.0, 0.4, 1.0))  # Green
        self.view.addItem(self.forearm)
        
        # Create coordinate axes for reference
        self.add_coordinate_axes()
        
    def add_coordinate_axes(self):
        """Add coordinate axes for reference"""
        # Calculate axis length proportional to arm segments
        axis_length = (self.upper_arm_length + self.forearm_length) * 0.35  # ~35% of total arm length
        
        # X-axis (red) - Forward/Along arm towards fingers
        x_axis = np.array([[0, 0, 0], [axis_length, 0, 0]])
        x_line = gl.GLLinePlotItem(pos=x_axis, color=(1, 0, 0, 1), width=5)
        self.view.addItem(x_line)
        
        # Y-axis (green) - Right-hand rule (left when looking forward)  
        y_axis = np.array([[0, 0, 0], [0, axis_length * 0.8, 0]])
        y_line = gl.GLLinePlotItem(pos=y_axis, color=(0, 1, 0, 1), width=5)
        self.view.addItem(y_line)
        
        # Z-axis (blue) - Up to ceiling
        z_axis = np.array([[0, 0, 0], [0, 0, axis_length * 0.8]])
        z_line = gl.GLLinePlotItem(pos=z_axis, color=(0, 0, 1, 1), width=5)
        self.view.addItem(z_line)
    
    def _normalize_quaternion(self, q):
        q = np.asarray(q, dtype=float)
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [w, x, y, z] to 4x4 rotation matrix (normalized)."""
        w, x, y, z = self._normalize_quaternion(q)
        rotation_matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),     0],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),     0],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y), 0],
            [0,                 0,                 0,                 1]
        ], dtype=float)
        return rotation_matrix
    
    def _axis_align_matrix(self):
        """Constant matrix that maps cylinder local Z-axis to the model's X-axis."""
        # Equivalent to +90° rotation about Y-axis
        return np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ], dtype=float)
    
    def update_arm_pose(self, upper_arm_quat, forearm_quat):
        """Update arm pose from IMU quaternions (X forward, Z up, Y right-hand)."""
        # Reset transforms for all arm components
        self.upper_arm.resetTransform()
        self.forearm.resetTransform()
        self.elbow.resetTransform()
        
        # Ensure quaternion consistency and fix coordinate system
        # Normalize and ensure positive w component to avoid sign ambiguity
        quat = self._normalize_quaternion(upper_arm_quat)
        if quat[0] < 0:  # If w is negative, flip entire quaternion
            quat = -quat
            
        # Apply coordinate system corrections for anatomical alignment
        corrected_quat = np.array([
            quat[0],   # w stays the same
            quat[1],   # x stays the same  
            -quat[2],  # y - negate for elevation direction
            quat[3]    # z stays the same
        ])
        
        # Build rotation matrix from corrected quaternion
        R_upper = self.quaternion_to_rotation_matrix(corrected_quat)
        A = self._axis_align_matrix()
        
        # CORRECTED: Apply rotation and translation properly
        # The cylinder should rotate around the shoulder joint (origin)
        # and its center should be positioned along its own rotated X-axis
        
        # Step 1: Create transform matrix with rotation
        T_upper = np.eye(4, dtype=float)
        T_upper[:3, :3] = R_upper[:3, :3] @ A[:3, :3]
        
        # Step 2: Calculate translation in the rotated coordinate system
        # The cylinder center should be at upper_arm_length/2 along the rotated X-axis
        rotated_direction = R_upper[:3, :3] @ np.array([0.0, 0.0, 0.0])
        T_upper[:3, 3] = rotated_direction * (self.upper_arm_length * 0.5)
        
        self.upper_arm.setTransform(T_upper)
        
        # Calculate elbow position (end of upper arm)
        elbow_world = rotated_direction * self.upper_arm_length
        
        # Move elbow sphere to elbow position
        self.elbow.translate(elbow_world[0], elbow_world[1], elbow_world[2])
        
        # Process forearm quaternion with same corrections
        forearm_quat_norm = self._normalize_quaternion(forearm_quat)
        if forearm_quat_norm[0] < 0:
            forearm_quat_norm = -forearm_quat_norm
            
        corrected_forearm_quat = np.array([
            forearm_quat_norm[0],   # w stays the same
            forearm_quat_norm[1],   # x stays the same  
            -forearm_quat_norm[2],  # y - negate for elevation direction
            forearm_quat_norm[3]    # z stays the same
        ])
        
        # Build forearm transform
        R_fore = self.quaternion_to_rotation_matrix(corrected_forearm_quat)
        dir_fore = R_fore[:3, :3] @ np.array([1.0, 0.0, 0.0])
        
        T_fore = np.eye(4, dtype=float)
        T_fore[:3, :3] = R_fore[:3, :3] @ A[:3, :3]
        T_fore[:3, 3] = elbow_world + dir_fore * (self.forearm_length*1.2)
        self.forearm.setTransform(T_fore)
    
    def quaternion_conjugate(self, q):
        """Return quaternion conjugate [w, -x, -y, -z]"""
        return np.array([q[0], -q[1], -q[2], -q[3]])
    
    def quat_multiply(self, q1, q2):
        """Multiply two quaternions q1 * q2 (Hamilton product)"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        q = np.array([w, x, y, z])
        return q / np.linalg.norm(q)  # normalize to unit length
    
    def get_widget(self):
        """Return the 3D view widget for embedding in GUI"""
        return self.view

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
        
        # View mode (True = graphs, False = 3D)
        self.show_graphs = True
        
        # Setup 3D model first (needed by setup_ui)
        self.arm_3d = Arm3DModel()
        
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
        
        # Left side: Content area (graphs or 3D view)
        self.content_widget = QWidget()
        self.content_layout = QVBoxLayout(self.content_widget)
        
        # Create graphs widget
        self.graphs_widget = QWidget()
        graphs_layout = QGridLayout(self.graphs_widget)
        
        # Create graph frames
        self.create_graph_frame("Upper Arm X-Axis (Shoulder Rotation)", "upper_arm_x", graphs_layout, 0, 0)
        self.create_graph_frame("Upper Arm Y-Axis (Shoulder Elevation)", "upper_arm_y", graphs_layout, 0, 1)
        self.create_graph_frame("Upper Arm Z-Axis (Shoulder Abduction)", "upper_arm_z", graphs_layout, 1, 0)
        self.create_graph_frame("Elbow Flexion/Extension", "elbow_flexion", graphs_layout, 1, 1)
        
        # Create 3D widget container
        self.view_3d_widget = QWidget()
        view_3d_layout = QVBoxLayout(self.view_3d_widget)
        view_3d_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        view_3d_layout.setSpacing(0)  # Remove spacing
        
        # Add title for 3D view with fixed height
        view_3d_title = QLabel("3D Arm Visualization")
        view_3d_title.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        view_3d_title.setFixedHeight(32)  # Fixed 32 pixels height
        view_3d_title.setStyleSheet("padding: 8px; background-color: #3c3c3c;")
        view_3d_layout.addWidget(view_3d_title)
        
        # Add 3D view (takes remaining space)
        view_3d_layout.addWidget(self.arm_3d.get_widget(), 1)  # Stretch factor = 1
        
        # Initially show graphs
        self.content_layout.addWidget(self.graphs_widget)
        
        # Right side: Info panel
        info_widget = self.create_info_panel()
        
        # Add to main layout
        main_layout.addWidget(self.content_widget, 3)  # 75% width
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
        
        # View toggle button
        self.view_toggle_button = QPushButton("Switch to 3D View")
        self.view_toggle_button.clicked.connect(self.toggle_view)
        self.view_toggle_button.setStyleSheet("""
            QPushButton { 
                background-color: #5a5a5a; 
                border: 2px solid #777; 
                padding: 8px; 
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #6a6a6a; }
        """)
        info_layout.addWidget(self.view_toggle_button)
        
        # Separator
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.Shape.HLine)
        separator2.setFrameShadow(QFrame.Shadow.Sunken)
        info_layout.addWidget(separator2)
        
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
    
    def toggle_view(self):
        """Toggle between graphs and 3D view"""
        if self.show_graphs:
            # Switch to 3D view
            self.content_layout.removeWidget(self.graphs_widget)
            self.graphs_widget.hide()
            self.content_layout.addWidget(self.view_3d_widget)
            self.view_3d_widget.show()
            
            self.show_graphs = False
            self.view_toggle_button.setText("Switch to Graphs View")
            
        else:
            # Switch to graphs view
            self.content_layout.removeWidget(self.view_3d_widget)
            self.view_3d_widget.hide()
            self.content_layout.addWidget(self.graphs_widget)
            self.graphs_widget.show()
            
            self.show_graphs = True
            self.view_toggle_button.setText("Switch to 3D View")
    
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
            
            # Update 3D model if available - use calibrated quaternions for proper zero reference
            if 'calibrated_quaternions' in joint_measurements:
                upper_arm_quat = joint_measurements['calibrated_quaternions']['upper_arm']
                forearm_quat = joint_measurements['calibrated_quaternions']['forearm']
                self.arm_3d.update_arm_pose(upper_arm_quat, forearm_quat)
    
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

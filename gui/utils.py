import numpy as np
import time

# ===== MULTI-IMU DATA PROCESSING =====

def parse_imu_data(raw_data):
    """
    Extract and organize three quaternions from 12-double array.
    Format: [q0(w,x,y,z), q1(w,x,y,z), q2(w,x,y,z)]
    Returns: dict with 'upper_arm', 'forearm', 'palm' quaternions
    """
    if len(raw_data) != 12:
        raise ValueError(f"Expected 12 doubles, got {len(raw_data)}")
    
    return {
        'upper_arm': np.array([raw_data[0], raw_data[1], raw_data[2], raw_data[3]]),
        'forearm': np.array([raw_data[4], raw_data[5], raw_data[6], raw_data[7]]),
        'palm': np.array([raw_data[8], raw_data[9], raw_data[10], raw_data[11]])
    }

def quaternion_conjugate(q):
    """Return quaternion conjugate [w, -x, -y, -z]"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def calculate_relative_quaternion(q_current, q_reference):
    """
    Calculate rotation relative to reference frame.
    q_relative = q_current * conjugate(q_reference)
    """
    q_ref_conj = quaternion_conjugate(q_reference)
    return quat_multiply(q_current, q_ref_conj)

def calculate_joint_rotation(q_distal, q_proximal):
    """
    Calculate rotation of distal segment relative to proximal segment.
    q_joint = q_distal * conjugate(q_proximal)
    """
    q_prox_conj = quaternion_conjugate(q_proximal)
    return quat_multiply(q_distal, q_prox_conj)

def quaternion_mean(q, q_ref, S, w = 1):
    # Normalize Input Quaternion
    q_normalized = normalize_quaternion(q, q_ref)

    if np.dot(q, q_ref) < 0:
        q_normalized = -q_normalized

    print("Normalized Quaternion (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_normalized[0], q_normalized[1], q_normalized[2], q_normalized[3]))
    Q = np.outer(q_normalized, q_normalized)
    print ("Outer Product Q:\n", Q)
    S_new = S + w*Q
    print("Updated Covariance Matrix S:\n", S_new)

    eigvals, eigvecs = np.linalg.eigh(S_new)  # eigh() is for symmetric matrices
    max_index = np.argmax(eigvals)
    q_selected = eigvecs[:, max_index]
    print ("Selected Eigenvector (before normalization) (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_selected[0], q_selected[1], q_selected[2], q_selected[3]))

    q_selected =q_selected/np.linalg.norm(q_selected)
    print ("Updated Mean Quaternion (after normalization) (w,x,y,z): %.2g, %.2g, %.2g, %.2g" % (q_selected[0], q_selected[1], q_selected[2], q_selected[3]))
    return q_selected, S_new

def normalize_quaternion(q, q_ref):
    return q * (1 if (q_ref.T).dot(q) >= 0 else -1)


def quat_multiply(q1, q2):
    """
    Multiply two quaternions q1 * q2 (Hamilton product).
    q1, q2: array-like, shape (4,) [w, x, y, z]
    Returns: numpy array (4,) quaternion product.
    """
    w1 = q1[0]; x1 = q1[1]; y1 = q1[2]; z1 = q1[3]
    w2 = q2[0]; x2 = q2[1]; y2 = q2[2]; z2 = q2[3]

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)  # normalize result to unit length


def quaternion_to_euler_zyx(q):
    """
    Convert a quaternion (w, x, y, z) to Euler angles (ZYX order: roll, pitch, yaw).
    Returns (x, y, z) angles in radians.
    """
    w, x, y, z = q
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z


# ===== MULTI-IMU CALIBRATION SYSTEM =====

class MultiIMUCalibrator:
    """Handles simultaneous calibration of all three IMUs"""
    
    def __init__(self, calibration_time=5):
        self.calibration_time = calibration_time
        self.segments = ['upper_arm', 'forearm', 'palm']
        
        # Initialize reference quaternions and covariance matrices
        self.reference_quats = {segment: np.array([1, 0, 0, 0]) for segment in self.segments}
        self.covariance_matrices = {segment: np.zeros((4, 4)) for segment in self.segments}
        self.quaternion_arrays = {segment: [] for segment in self.segments}
        
        # Calibration state
        self.start_time = None
        self.is_complete = False
        self.sample_count = 0
    
    def update_calibration(self, imu_data):
        """Update calibration for all three IMUs simultaneously"""
        if self.is_complete:
            return
            
        # Initialize timing on first sample
        if self.sample_count == 0:
            self.start_time = time.time()
            print("Starting calibration for all IMUs, don't move")
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Update calibration for each IMU
        for segment in self.segments:
            q_current = imu_data[segment]
            self.quaternion_arrays[segment].append(q_current)
            
            # Update running mean using existing quaternion_mean function
            if len(self.quaternion_arrays[segment]) > 1:
                self.reference_quats[segment], self.covariance_matrices[segment] = quaternion_mean(
                    q_current, 
                    self.quaternion_arrays[segment][0], 
                    self.covariance_matrices[segment], 
                    w=1
                )
        
        self.sample_count += 1
        
        if elapsed_time < self.calibration_time:
            print(f"Calibrating... {elapsed_time:.2f} / {self.calibration_time} seconds")
            # Print current reference quaternions for monitoring
            for segment in self.segments:
                q = self.reference_quats[segment]
                print(f"  {segment}: (w,x,y,z): {q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}")
        else:
            if not self.is_complete:
                self.is_complete = True
                print("Calibration complete for all IMUs! You can move now.")
                print("Reference quaternions:")
                for segment in self.segments:
                    q = self.reference_quats[segment]
                    print(f"  {segment}: (w,x,y,z): {q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}")
    
    def get_reference_quaternions(self):
        """Return calibrated reference quaternions for all IMUs"""
        return self.reference_quats.copy()
    
    def get_calibrated_data(self, imu_data):
        """Convert raw IMU data to calibrated relative quaternions"""
        if not self.is_complete:
            return None
            
        calibrated = {}
        for segment in self.segments:
            calibrated[segment] = calculate_relative_quaternion(
                imu_data[segment], 
                self.reference_quats[segment]
            )
        return calibrated


# ===== JOINT ANGLE CALCULATOR =====

class JointAngleCalculator:
    """Convert relative quaternions to anatomical joint angles"""
    
    def __init__(self):
        pass
    
    def wrist_angles(self, palm_quat, forearm_quat, verbose=False):
        """
        Convert to wrist flexion/extension, radial/ulnar deviation, pronation/supination.
        
        Coordinate Frame Convention:
        - X axis: points toward fingers
        - Z axis: points toward ceiling  
        - Y axis: follows right-hand rule (points laterally)
        
        Returns angles in degrees.
        """
        if verbose:
            # DEBUG: Log input quaternions
            print(f"\n--- WRIST DEBUG ---")
            print(f"Forearm Quat:    w={forearm_quat[0]:6.3f} x={forearm_quat[1]:6.3f} y={forearm_quat[2]:6.3f} z={forearm_quat[3]:6.3f}")
            print(f"Palm Quat:       w={palm_quat[0]:6.3f} x={palm_quat[1]:6.3f} y={palm_quat[2]:6.3f} z={palm_quat[3]:6.3f}")
        
        # Calculate palm rotation relative to forearm
        wrist_quat = calculate_joint_rotation(palm_quat, forearm_quat)
        
        if verbose:
            print(f"Relative Quat:   w={wrist_quat[0]:6.3f} x={wrist_quat[1]:6.3f} y={wrist_quat[2]:6.3f} z={wrist_quat[3]:6.3f}")
        
        # Convert to Euler angles (ZYX convention: roll=X, pitch=Y, yaw=Z)
        roll, pitch, yaw = quaternion_to_euler_zyx(wrist_quat)
        
        # Convert to degrees
        roll_deg = roll * 180 / np.pi
        pitch_deg = pitch * 180 / np.pi  
        yaw_deg = yaw * 180 / np.pi
        
        if verbose:
            # DEBUG: Log all Euler angles
            print(f"Euler Angles:")
            print(f"  Roll  (X-axis): {roll_deg:7.2f}° <- Pronation/Supination")
            print(f"  Pitch (Y-axis): {pitch_deg:7.2f}° <- Flexion/Extension")
            print(f"  Yaw   (Z-axis): {yaw_deg:7.2f}° <- Radial/Ulnar Deviation")
        
        # Map to anatomical terms based on coordinate frame:
        # - Flexion/Extension: rotation around Y axis (lateral axis)
        # - Radial/Ulnar: rotation around Z axis (vertical axis) 
        # - Pronation/Supination: rotation around X axis (finger direction)
        flexion_extension = pitch_deg        # Y-axis: Flexion(+) / Extension(-)
        radial_ulnar_deviation = yaw_deg     # Z-axis: Radial(+) / Ulnar(-)  
        pronation_supination = roll_deg      # X-axis: Pronation(+) / Supination(-)
        
        if verbose:
            print(f"Final Wrist Angles:")
            print(f"  Flexion/Extension:      {flexion_extension:7.2f}°")
            print(f"  Radial/Ulnar Deviation: {radial_ulnar_deviation:7.2f}°")
            print(f"  Pronation/Supination:   {pronation_supination:7.2f}°")
            print(f"--- END WRIST DEBUG ---\n")
        
        return {
            'flexion_extension': flexion_extension,
            'radial_ulnar_deviation': radial_ulnar_deviation,
            'pronation_supination': pronation_supination
        }
    
    def elbow_angles(self, forearm_quat, upperarm_quat, verbose=False):
        """
        Convert to elbow flexion/extension.
        
        Coordinate Frame Convention:
        - X axis: points toward fingers (distal direction)
        - Z axis: points toward ceiling
        - Y axis: follows right-hand rule (points laterally)
        
        Returns angles in degrees.
        """
        if verbose:
            # DEBUG: Log input quaternions
            print(f"\n--- ELBOW DEBUG ---")
            print(f"Upper Arm Quat:  w={upperarm_quat[0]:6.3f} x={upperarm_quat[1]:6.3f} y={upperarm_quat[2]:6.3f} z={upperarm_quat[3]:6.3f}")
            print(f"Forearm Quat:    w={forearm_quat[0]:6.3f} x={forearm_quat[1]:6.3f} y={forearm_quat[2]:6.3f} z={forearm_quat[3]:6.3f}")
        
        # Calculate forearm rotation relative to upper arm
        elbow_quat = calculate_joint_rotation(forearm_quat, upperarm_quat)
        
        if verbose:
            print(f"Relative Quat:   w={elbow_quat[0]:6.3f} x={elbow_quat[1]:6.3f} y={elbow_quat[2]:6.3f} z={elbow_quat[3]:6.3f}")
        
        # Convert to Euler angles (ZYX convention: roll=X, pitch=Y, yaw=Z)
        roll, pitch, yaw = quaternion_to_euler_zyx(elbow_quat)
        
        # Convert to degrees
        roll_deg = roll * 180 / np.pi
        pitch_deg = pitch * 180 / np.pi  
        yaw_deg = yaw * 180 / np.pi
        
        if verbose:
            # DEBUG: Log all Euler angles
            print(f"Euler Angles:")
            print(f"  Roll  (X-axis): {roll_deg:7.2f}°")
            print(f"  Pitch (Y-axis): {pitch_deg:7.2f}° <- Currently using as flexion")
            print(f"  Yaw   (Z-axis): {yaw_deg:7.2f}°")
        
        # Map to anatomical terms based on coordinate frame:
        # Based on debug data, elbow flexion appears to be Z-axis rotation (yaw)
        # - Elbow Flexion/Extension: rotation around Z axis (vertical axis)
        #   When forearm moves toward bicep = positive flexion
        flexion_extension = yaw_deg  # Z-axis: Flexion(+) / Extension(-)
        
        if verbose:
            print(f"TESTING: Pitch={pitch_deg:7.2f}° vs Yaw={yaw_deg:7.2f}° (Using Yaw as flexion)")
            print(f"Final Flexion/Extension: {flexion_extension:7.2f}°")
            print(f"--- END ELBOW DEBUG ---\n")
        
        return {
            'flexion_extension': flexion_extension
        }
    
    def upper_arm_angles(self, upperarm_quat, verbose=False):
        """
        Convert upper arm quaternion to global orientation angles.
        This represents upper arm movement relative to the calibrated reference.
        
        Coordinate Frame Convention:
        - X axis: points toward fingers (distal direction)
        - Z axis: points toward ceiling
        - Y axis: follows right-hand rule (points laterally)
        
        Returns angles in degrees.
        """
        if verbose:
            print(f"\n--- UPPER ARM DEBUG ---")
            print(f"Upper Arm Quat:  w={upperarm_quat[0]:6.3f} x={upperarm_quat[1]:6.3f} y={upperarm_quat[2]:6.3f} z={upperarm_quat[3]:6.3f}")
        
        # Convert to Euler angles (ZYX convention: roll=X, pitch=Y, yaw=Z)
        roll, pitch, yaw = quaternion_to_euler_zyx(upperarm_quat)
        
        # Convert to degrees
        roll_deg = roll * 180 / np.pi
        pitch_deg = pitch * 180 / np.pi  
        yaw_deg = yaw * 180 / np.pi
        
        if verbose:
            print(f"Euler Angles:")
            print(f"  Roll  (X-axis): {roll_deg:7.2f}° <- Shoulder rotation")
            print(f"  Pitch (Y-axis): {pitch_deg:7.2f}° <- Shoulder elevation")
            print(f"  Yaw   (Z-axis): {yaw_deg:7.2f}° <- Shoulder abduction")
        
        # Map to anatomical terms based on coordinate frame:
        shoulder_rotation = roll_deg     # X-axis: Internal(+) / External(-) rotation
        shoulder_elevation = pitch_deg   # Y-axis: Elevation(+) / Depression(-)
        shoulder_abduction = yaw_deg     # Z-axis: Abduction(+) / Adduction(-)
        
        # Use supplement angle for abduction when it exceeds anatomical limits
        # If abduction > 90°, use supplement angle (180° - angle)
        if abs(shoulder_abduction) > 90.0:
            shoulder_abduction = 180.0 - abs(shoulder_abduction)
            if shoulder_abduction < 0:  # Preserve sign for direction
                shoulder_abduction = -shoulder_abduction
        
        if verbose:
            print(f"Final Upper Arm Angles:")
            print(f"  Shoulder Rotation:   {shoulder_rotation:7.2f}°")
            print(f"  Shoulder Elevation:  {shoulder_elevation:7.2f}°")
            print(f"  Shoulder Abduction:  {shoulder_abduction:7.2f}° (supplement applied if >90°)")
            print(f"--- END UPPER ARM DEBUG ---\n")
        
        return {
            'shoulder_rotation': shoulder_rotation,
            'shoulder_elevation': shoulder_elevation,
            'shoulder_abduction': shoulder_abduction
        }


# ===== MAIN MOTION TRACKING SYSTEM =====

class ArmMotionTracker:
    """Main class for processing multi-IMU arm motion data"""
    
    def __init__(self, calibration_time=5, verbose=False):
        self.calibrator = MultiIMUCalibrator(calibration_time)
        self.angle_calculator = JointAngleCalculator()
        self.last_joint_measurements = None
        self.verbose = verbose
        self.active_body_parts = {'upper_arm': True, 'elbow': True, 'wrist': True}
    
    def set_verbose(self, verbose):
        """Enable or disable verbose debug output"""
        self.verbose = verbose
    
    def set_active_body_parts(self, upper_arm=True, elbow=True, wrist=True):
        """Configure which body parts to track and display"""
        self.active_body_parts = {
            'upper_arm': upper_arm,
            'elbow': elbow, 
            'wrist': wrist
        }
    
    def process_frame(self, raw_data):
        """
        Process a single frame of IMU data.
        Returns structured joint measurements or None during calibration.
        """
        # Parse the raw 12-double data
        try:
            imu_data = parse_imu_data(raw_data)
            if self.verbose:
                # DEBUG: Log raw parsed data
                print(f"\n=== RAW IMU DATA ===")
                print(f"Upper Arm RAW:  w={imu_data['upper_arm'][0]:6.3f} x={imu_data['upper_arm'][1]:6.3f} y={imu_data['upper_arm'][2]:6.3f} z={imu_data['upper_arm'][3]:6.3f}")
                print(f"Forearm RAW:    w={imu_data['forearm'][0]:6.3f} x={imu_data['forearm'][1]:6.3f} y={imu_data['forearm'][2]:6.3f} z={imu_data['forearm'][3]:6.3f}")
                print(f"Palm RAW:       w={imu_data['palm'][0]:6.3f} x={imu_data['palm'][1]:6.3f} y={imu_data['palm'][2]:6.3f} z={imu_data['palm'][3]:6.3f}")
                print(f"==================\n")
        except ValueError as e:
            print(f"Error parsing IMU data: {e}")
            return None
        
        # Handle calibration phase
        if not self.calibrator.is_complete:
            self.calibrator.update_calibration(imu_data)
            return None
        
        # Get calibrated relative quaternions
        calibrated_data = self.calibrator.get_calibrated_data(imu_data)
        if calibrated_data is None:
            return None
            
        if self.verbose:
            # DEBUG: Log calibrated data
            print(f"=== CALIBRATED DATA ===")
            print(f"Upper Arm CAL:  w={calibrated_data['upper_arm'][0]:6.3f} x={calibrated_data['upper_arm'][1]:6.3f} y={calibrated_data['upper_arm'][2]:6.3f} z={calibrated_data['upper_arm'][3]:6.3f}")
            print(f"Forearm CAL:    w={calibrated_data['forearm'][0]:6.3f} x={calibrated_data['forearm'][1]:6.3f} y={calibrated_data['forearm'][2]:6.3f} z={calibrated_data['forearm'][3]:6.3f}")
            print(f"Palm CAL:       w={calibrated_data['palm'][0]:6.3f} x={calibrated_data['palm'][1]:6.3f} y={calibrated_data['palm'][2]:6.3f} z={calibrated_data['palm'][3]:6.3f}")
            print(f"=====================\n")
        
        # Calculate joint angles based on active body parts
        try:
            results = {}
            
            if self.active_body_parts['upper_arm']:
                upper_arm_angles = self.angle_calculator.upper_arm_angles(
                    calibrated_data['upper_arm'], 
                    verbose=self.verbose
                )
                results['upper_arm'] = upper_arm_angles
                
            if self.active_body_parts['elbow']:
                elbow_angles = self.angle_calculator.elbow_angles(
                    calibrated_data['forearm'], 
                    calibrated_data['upper_arm'],
                    verbose=self.verbose
                )
                results['elbow'] = elbow_angles
                
            if self.active_body_parts['wrist']:
                wrist_angles = self.angle_calculator.wrist_angles(
                    calibrated_data['palm'], 
                    calibrated_data['forearm'],
                    verbose=self.verbose
                )
                results['wrist'] = wrist_angles
            
            # Structure the complete results
            joint_measurements = {
                **results,  # Include computed angles (upper_arm, elbow, wrist as available)
                'calibration_status': 'complete',
                'data_quality': 'good',
                'raw_quaternions': {
                    'upper_arm': imu_data['upper_arm'],
                    'forearm': imu_data['forearm'],
                    'palm': imu_data['palm']
                },
                'calibrated_quaternions': calibrated_data
            }
            
            # Clean output when not verbose
            if not self.verbose:
                self._display_clean_output(results)
            
            self.last_joint_measurements = joint_measurements
            return joint_measurements
            
        except Exception as e:
            print(f"Error calculating joint angles: {e}")
            return None
    
    def is_calibrated(self):
        """Check if calibration is complete"""
        return self.calibrator.is_complete
    
    def get_calibration_progress(self):
        """Get calibration progress as percentage"""
        if self.calibrator.start_time is None:
            return 0.0
        
        elapsed = time.time() - self.calibrator.start_time
        progress = min(elapsed / self.calibrator.calibration_time * 100, 100.0)
        return progress
    
    def _display_clean_output(self, results):
        """Display clean, non-verbose output for joint angles"""
        print("\n" + "="*50)
        print("ARM MOTION MEASUREMENTS")
        print("="*50)
        
        if 'upper_arm' in results:
            upper_arm = results['upper_arm']
            print("UPPER ARM (Shoulder):")
            print(f"  Rotation:   {upper_arm['shoulder_rotation']:6.1f}° {'(Internal)' if upper_arm['shoulder_rotation'] > 0 else '(External)'}")
            print(f"  Elevation:  {upper_arm['shoulder_elevation']:6.1f}° {'(Up)' if upper_arm['shoulder_elevation'] > 0 else '(Down)'}")
            print(f"  Abduction:  {upper_arm['shoulder_abduction']:6.1f}° {'(Out)' if upper_arm['shoulder_abduction'] > 0 else '(In)'}")
            print()
            
        if 'elbow' in results:
            elbow = results['elbow']
            print("ELBOW:")
            print(f"  Flexion/Extension: {elbow['flexion_extension']:6.1f}° {'(Flex)' if elbow['flexion_extension'] > 0 else '(Ext)'}")
            print()
            
        if 'wrist' in results:
            wrist = results['wrist']
            print("WRIST:")
            print(f"  Flexion/Extension:     {wrist['flexion_extension']:6.1f}° {'(Flex)' if wrist['flexion_extension'] > 0 else '(Ext)'}")
            print(f"  Radial/Ulnar Deviation: {wrist['radial_ulnar_deviation']:6.1f}° {'(Radial)' if wrist['radial_ulnar_deviation'] > 0 else '(Ulnar)'}")
            print(f"  Pronation/Supination:   {wrist['pronation_supination']:6.1f}° {'(Pronate)' if wrist['pronation_supination'] > 0 else '(Supinate)'}")
            
        print("="*50 + "\n")


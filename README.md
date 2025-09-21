# Amaru Maki v2 — Upper-Arm Wearable for Physical Rehabilitation

Amaru Maki v2 is the second generation of an upper-arm wearable device designed to support physical rehabilitation. It captures 3D motion from multiple inertial measurement units (IMUs) placed along the arm and streams the data over Bluetooth Low Energy (BLE) for real‑time analysis and visualization. The system focuses on tracking clinically meaningful movements of the shoulder, elbow, and wrist to aid therapy, assessment, and progress monitoring.

## What this project is
- **A wearable sensing platform**: ESP32-based firmware reads multiple Adafruit BNO055 IMUs through an I2C multiplexer and publishes orientation data as quaternions at ~60 Hz.
- **A real-time BLE data link**: A custom BLE GATT service notifies a continuous packet of 12 doubles (3 quaternions: upper arm, forearm, palm/wrist) for host applications.
- **A host-side toolkit and GUI**: Python utilities convert quaternions into anatomical joint angles and provide a PyQt6 GUI with plots and a 3D arm view for immediate visual feedback.

## Why it matters
- **Rehab-focused metrics**: Converts raw sensor orientation to interpretable measures (e.g., shoulder rotation/elevation/abduction, elbow flexion/extension, wrist motions).
- **Calibration for consistency**: A short, guided calibration establishes consistent reference frames across sessions and users.
- **Therapy visibility**: Real-time visuals help patients and clinicians verify motion quality, range, and symmetry during exercises.

## High-level architecture
- **Firmware (ESP32)**
  - Modules: `BLE_Wearable` (GATT service/characteristic, notifications), `I2C_MUX` (PCA9548A channel control), Adafruit BNO055 drivers.
  - Main loop: cycles MUX channels, reads quaternions from each IMU, and transmits a packed buffer over BLE.
- **Host applications (Python)**
  - `gui/` provides a PyQt6 application for real-time visualization with line plots and a simple 3D arm model.
  - `utils.py` implements motion processing: parsing multi-IMU data, calibration (mean quaternion reference), relative segment quaternions, and anatomical angle extraction.

## Data model
- **BLE payload**: 12 little-endian doubles representing three quaternions in order `upper_arm`, `forearm`, `palm`.
  - Quaternion layout per segment: `[w, x, y, z]`.
- **Processing pipeline**:
  1) Parse raw payload → 3 segment quaternions.
  2) Calibrate for a fixed window to estimate reference quaternions.
  3) Compute relative orientations (distal vs proximal).
  4) Convert to joint angles aligned to simple anatomical conventions.

## Key capabilities
- **Multi-IMU synchronization via I2C MUX**: Scans multiple BNO055 sensors across channels for shoulder/elbow/wrist coverage.
- **Stable BLE notifications**: Lightweight GATT service with notify-only characteristic for streaming orientation data.
- **Real-time visualization**: 2D plots of angles and a 3D arm representation with basic segment geometry.
- **Extensible analysis**: Modular utilities to experiment with coordinate frames, angle definitions, and additional joints.

## Improvements vs V1
- **Sensor & link architecture**: V1 used an IMU 9250 with integrated quaternion solution and streamed over BLE UART. V2 uses multiple Adafruit BNO055 sensors via an I2C MUX and streams a structured quaternion payload via BLE GATT notifications for more robust, typed data exchange.
- **Timing & throughput**: V1 achieved 60 Hz streaming but updated its configuration/estimation state at ~10 Hz, introducing inconsistency. V2 maintains a uniform ~60 Hz pipeline, using a short calibration window and then computing orientations each frame for consistent real‑time behavior.
- **Mathematical model**: V1 estimated joint states via per‑joint gradient‑descent optimization. V2 replaces this with direct quaternion algebra (relative orientation between segments), reducing compute and latency while improving numerical stability.
- **Flexible zeroing**: V2 enables setting a rest/zero position at any pose; joint angles are computed relative to the calibrated reference, simplifying user setup and allowing re‑zeroing on demand.



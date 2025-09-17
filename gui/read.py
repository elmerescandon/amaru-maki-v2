import asyncio
from bleak import BleakScanner, BleakClient
import struct
from scipy.spatial.transform import Rotation as R

CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"  # characteristic UUID

async def main():
    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    esp32 = None
    for d in devices:
        if d.name == "ESP32_Hello":  # Match by advertised name
            esp32 = d
            break

    if not esp32:
        print("ESP32 not found!")
        return

    print(f"Found ESP32: {esp32.address}")

    async with BleakClient(esp32) as client:
        print("Connected!")
        
        def handle_data(s, data):
            # Unpack 4 doubles (little-endian, 8 bytes each)
            quat = struct.unpack('<12d', data)
            # print("Quaternion:", quat)
            # Convert quaternion (w, x, y, z) to (x, y, z, w) for scipy
            quat_shoulder = [quat[1], quat[2], quat[3], quat[0]]
            quat_elbow = [quat[5], quat[6], quat[7], quat[4]]
            quat_wrist = [quat[9], quat[10], quat[11], quat[8]]
            print('Q1: %.2g, %.2g, %.2g, %.2g' % (quat_shoulder[0], quat_shoulder[1], quat_shoulder[2], quat_shoulder[3]))
            print('Q2: %.2g, %.2g, %.2g, %.2g' % (quat_elbow[0], quat_elbow[1], quat_elbow[2], quat_elbow[3]))
            print('Q3: %.2g, %.2g, %.2g, %.2g' % (quat_wrist[0], quat_wrist[1], quat_wrist[2], quat_wrist[3]))
            # print("Euler angles (deg): %.2g, %.2g, %.2g" % (euler_x, euler_y, euler_z))
            # print("Euler angles (deg): %.2g" % (euler_x))

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)

asyncio.run(main())

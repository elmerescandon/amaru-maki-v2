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
            quat = struct.unpack('<4d', data)
            # print("Quaternion:", quat)
            # Convert quaternion (w, x, y, z) to (x, y, z, w) for scipy
            quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]
            r = R.from_quat(quat_xyzw)
            euler = r.as_euler('xyz', degrees=True)
            euler_x = round(euler[0], 2)
            euler_y = round(euler[1], 2)
            euler_z = round(euler[2], 2)
            # print("Euler angles (deg): %.2g, %.2g, %.2g" % (euler_x, euler_y, euler_z))
            print("Euler angles (deg): %.2g" % (euler_x))

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)

asyncio.run(main())

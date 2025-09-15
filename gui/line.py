
import asyncio
from bleak import BleakScanner, BleakClient
import struct
from scipy.spatial.transform import Rotation as R

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import qasync
import sys

euler_x_queue = asyncio.Queue()

CHAR_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"  # characteristic UUID


async def ble_task():
    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    esp32 = None
    for d in devices:
        if d.name == "ESP32_Hello":
            esp32 = d
            break

    if not esp32:
        print("ESP32 not found!")
        return

    print(f"Found ESP32: {esp32.address}")

    async with BleakClient(esp32) as client:
        print("Connected!")

        def handle_data(s, data):
            quat = struct.unpack('<4d', data)
            quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]
            r = R.from_quat(quat_xyzw)
            euler = r.as_euler('xyz', degrees=True)
            euler_x = float(euler[0])
            print(f"Received Euler X: {euler_x:.2f}")
            # Robust: put_nowait, drop if queue is full
            try:
                euler_x_queue.put_nowait(euler_x)
            except asyncio.QueueFull:
                pass

        await client.start_notify(CHAR_UUID, handle_data)
        await asyncio.sleep(100)





plot_window = None
update_timer = None

def start_plot():
    global plot_window, update_timer
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot_window = pg.GraphicsLayoutWidget(title="Real-time Euler X Angle")
    plot_window.show()
    plot = plot_window.addPlot(title="Euler X (deg)")
    curve = plot.plot(pen='y')
    data = []
    max_points = 500

    def update():
        while not euler_x_queue.empty():
            try:
                val = euler_x_queue.get_nowait()
                print(f"Euler X: {val:.2f}")
                data.append(val)
                if len(data) > max_points:
                    data.pop(0)
            except Exception:
                break
        curve.setData(data)


    update_timer = QtCore.QTimer(plot_window)
    update_timer.timeout.connect(update)
    update_timer.start(10)



def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    start_plot()  # just setup, no event loop start
    loop.create_task(ble_task())
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()

import asyncio
from bleak import BleakScanner, BleakClient

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
        await client.start_notify(CHAR_UUID, lambda s, data: print(data.decode()))
        await asyncio.sleep(10)

asyncio.run(main())

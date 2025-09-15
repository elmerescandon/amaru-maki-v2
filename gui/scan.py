import asyncio
from bleak import BleakScanner

async def scan():
    async with BleakScanner(scanning_mode="active") as scanner:
        await asyncio.sleep(10.0)
        for d in scanner.discovered_devices:
            print(d)

asyncio.run(scan())

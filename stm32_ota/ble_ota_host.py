import asyncio
import struct
import zlib
from bleak import BleakClient, BleakScanner

# === BLE Configuration ===
DEVICE_NAME = "ESP32_OTA_BLE"
SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CHAR_RX_UUID = "12345678-1234-1234-1234-1234567890ac"  # Write
CHAR_TX_UUID = "12345678-1234-1234-1234-1234567890ad"  # Notify

# === OTA Protocol ===
SOF = 0xA5
EOF = 0xB6
PACKET_CMD = 0x01
PACKET_HEADER = 0x02
PACKET_DATA = 0x03
PACKET_RESP = 0x04
CMD_START = 0xA0
CMD_END = 0xA1
RESP_ACK = 0xAB
RESP_NACK = 0xCD
CHUNK_SIZE = 128

ack_event = asyncio.Event()
last_status = None

def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF

def build_frame(packet_type, payload: bytes) -> bytes:
    return (
        bytearray([SOF, packet_type]) +
        struct.pack('<H', len(payload)) +
        payload +
        struct.pack('<I', crc32(payload)) +
        bytearray([EOF])
    )

def handle_notification(_, data: bytearray):
    global last_status
    if len(data) >= 5 and data[0] == SOF and data[1] == PACKET_RESP:
        last_status = data[4]
        ack_event.set()

async def wait_for_ack(timeout=2.0):
    global last_status
    try:
        await asyncio.wait_for(ack_event.wait(), timeout)
        ack_event.clear()
        return last_status == RESP_ACK
    except asyncio.TimeoutError:
        print("Timeout waiting for ACK")
        return False

async def send_cmd(client, cmd_id):
    frame = build_frame(PACKET_CMD, bytes([cmd_id]))
    await client.write_gatt_char(CHAR_RX_UUID, frame)
    return await wait_for_ack()

async def send_header(client, fw_size, fw_crc32):
    payload = struct.pack('<III', fw_size, fw_crc32, 0) + b'\x00' * 4
    await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_HEADER, payload))
    return await wait_for_ack()

async def send_data_chunks(client, fw_data):
    for i in range(0, len(fw_data), CHUNK_SIZE):
        chunk = fw_data[i:i+CHUNK_SIZE]
        await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_DATA, chunk))
        if not await wait_for_ack():
            print(f"NACK at chunk {i // CHUNK_SIZE}")
            return False
    return True

async def send_ota_sequence(client, filepath):
    try:
        with open(filepath, "rb") as f:
            fw_data = f.read()
    except FileNotFoundError:
        print("[Error] File not found:", filepath)
        return

    fw_size = len(fw_data)
    fw_crc32 = crc32(fw_data)
    print(f"Firmware size: {fw_size}, CRC32: 0x{fw_crc32:08X}")

    print("Sending CMD_START...")
    if not await send_cmd(client, CMD_START):
        return

    print("Sending header...")
    if not await send_header(client, fw_size, fw_crc32):
        return

    print("Sending firmware data...")
    if not await send_data_chunks(client, fw_data):
        return

    print("Sending CMD_END...")
    await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_CMD, bytes([CMD_END])))

async def main():
    print("Scanning for ESP32...")
    device = await BleakScanner.find_device_by_filter(lambda d, _: d.name == DEVICE_NAME)
    if not device:
        print("Device not found.")
        return

    async with BleakClient(device) as client:
        await client.start_notify(CHAR_TX_UUID, handle_notification)

        while True:
            cmd = input("> ").strip()
            if cmd == "send":
                filepath = input("Enter firmware path: ").strip()
                await send_ota_sequence(client, filepath)
            elif cmd == "exit":
                break
            else:
                await client.write_gatt_char(CHAR_RX_UUID, (cmd + "\n").encode())

        await client.stop_notify(CHAR_TX_UUID)

if __name__ == "__main__":
    asyncio.run(main())

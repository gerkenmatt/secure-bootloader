import asyncio
import struct
import zlib
from bleak import BleakClient, BleakScanner
import os

# === BLE Configuration ===
DEVICE_NAME   = "ESP32_OTA_BLE"
SERVICE_UUID  = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHAR_RX_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to ESP32
CHAR_TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notify from ESP32

# === OTA Protocol ===
SOF = 0xA5
EOF = 0xB6
PACKET_CMD = 0x01
PACKET_HEADER = 0x02
PACKET_DATA = 0x03
PACKET_RESP = 0x04
PACKET_SIG = 0x05
CMD_START = 0xA0
CMD_END = 0xA1
RESP_ACK = 0xAB
RESP_NACK = 0xCD
CHUNK_SIZE = 128

# === Color Codes ===
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

# --- Globals for async communication ---
ack_event = asyncio.Event()
slot_status_event = asyncio.Event()
active_slot = -1
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
    """
    Callback for BLE notifications from the ESP32.
    Handles both OTA protocol ACKs and general string messages.
    """
    global last_status, active_slot
    # Check if it's a structured OTA response (ACK/NACK)
    if len(data) >= 10 and data[0] == SOF and data[1] == PACKET_RESP:
        status = data[4]
        last_status = status
        if status == RESP_ACK:
            print(BLUE + "  --> ACK received" + RESET)
        elif status == RESP_NACK:
            print(RED + "  --> NACK received" + RESET)
        else:
            print(RED + "  --> Bad ACK data: " + data.hex() + RESET)
        ack_event.set()
    else:
        # Treat as a general text message from the device
        try:
            line = data.partition(b'\0')[0].decode('utf-8').strip()
            # NEW: Specifically look for the "active slot" message
            if line.startswith("Active slot:"):
                try:
                    active_slot = int(line.split(":")[1].strip())
                    print("    " + GREEN + f"Received status: Active slot is {active_slot}" + RESET)
                    slot_status_event.set() # Signal that we have received the status
                except (ValueError, IndexError):
                    print(RED + f"  --> Could not parse slot status from: {line}" + RESET)
            else:
                print("    " + GREEN + line + RESET)
        except UnicodeDecodeError:
            print("    " + GREEN + data.hex() + RESET)

async def wait_for_ack(timeout=4.0):
    try:
        await asyncio.wait_for(ack_event.wait(), timeout)
        ack_event.clear()
        return last_status == RESP_ACK
    except asyncio.TimeoutError:
        print(RED + "Timeout: No valid ACK received" + RESET)
        return False

async def send_cmd(client, cmd_id, wait=True):
    frame = build_frame(PACKET_CMD, bytes([cmd_id]))
    await client.write_gatt_char(CHAR_RX_UUID, frame)
    return await wait_for_ack() if wait else True

async def send_header(client, fw_size, fw_crc32):
    payload = struct.pack('<III', fw_size, fw_crc32, 0) + b'\x00' * 4
    await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_HEADER, payload))
    return await wait_for_ack()

async def send_data_chunks(client, fw_data):
    for i in range(0, len(fw_data), CHUNK_SIZE):
        print(f"sending chunk {i // CHUNK_SIZE}")
        chunk = fw_data[i:i+CHUNK_SIZE]
        await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_DATA, chunk))
        if not await wait_for_ack():
            print(RED + f"  --> No ACK for chunk {i // CHUNK_SIZE}" + RESET)
            return False
    return True

async def send_sig_chunks(client, sig_data):
    for i in range(0, len(sig_data), CHUNK_SIZE):
        print(f"sending signature chunk {i // CHUNK_SIZE}")
        chunk = sig_data[i:i+CHUNK_SIZE]
        await client.write_gatt_char(CHAR_RX_UUID, build_frame(PACKET_SIG, chunk))
        if not await wait_for_ack():
            print(RED + f"  --> No ACK for signature chunk {i // CHUNK_SIZE}" + RESET)
            return False
    return True

async def send_ota_sequence(client, fw_path, sig_path):
    ack_event.clear() 
    try:
        with open(fw_path, "rb") as f_fw:
            fw_data = f_fw.read()
        with open(sig_path, "rb") as f_sig:
            sig_data = f_sig.read()
    except FileNotFoundError as e:
        print(RED + f"[Error] File not found: {e.filename}" + RESET)
        return

    fw_size = len(fw_data)
    fw_crc32 = crc32(fw_data)

    print(f"Firmware size: {fw_size} bytes")
    print(f"CRC32: 0x{fw_crc32:08X}")

    print("Sending OTA_START")
    if not await send_cmd(client, CMD_START, wait=True):
        return

    print("Sending header")
    if not await send_header(client, fw_size, fw_crc32):
        return

    print("Sending firmware data...")
    if not await send_data_chunks(client, fw_data):
        return

    print("Sending signature...")
    if not await send_sig_chunks(client, sig_data):
        return

    print("Sending OTA_END")
    await send_cmd(client, CMD_END, wait=False)

async def main():
    global active_slot
    print(CYAN + BOLD + "\r\nScanning for ESP32...\n" + RESET)
    device = await BleakScanner.find_device_by_filter(lambda d, _: d.name == DEVICE_NAME)
    if not device:
        print(RED + "Device not found." + RESET)
        return

    try:
        async with BleakClient(device) as client:
            await client.start_notify(CHAR_TX_UUID, handle_notification)
            print(CYAN + BOLD + "\r\nConnected. Type: update <name>, status, send <msg>, quit\n" + RESET)
            while True:
                PROMPT = f"{CYAN}{BOLD}➜ {RESET}"
                cmd = (await asyncio.to_thread(input, PROMPT)).strip().lower()

                if cmd in ("exit", "quit", "q"):
                    break
                
                elif cmd == "status":
                    slot_status_event.clear()
                    active_slot = -1
                    print("Querying device status...")
                    await client.write_gatt_char(CHAR_RX_UUID, b"status\n")
                    try:
                        await asyncio.wait_for(slot_status_event.wait(), timeout=5.0)
                        # The notification handler will print the result
                    except asyncio.TimeoutError:
                        print(RED + "Timeout: Did not receive status response." + RESET)

                elif cmd.lower().startswith("update"):
                    parts = cmd.split()
                    if len(parts) != 2:
                        print(RED + "Invalid command. Usage: update <firmware_name>" + RESET)
                        continue
                    
                    base_name = parts[1]
                    fw_path_a = f"firmware/{base_name}_slota.bin"
                    sig_path_a = f"firmware/{base_name}_slota.sig"
                    fw_path_b = f"firmware/{base_name}_slotb.bin"
                    sig_path_b = f"firmware/{base_name}_slotb.sig"

                    # Ensure both slota and slotb files exist before starting
                    if not (os.path.exists(fw_path_a) and os.path.exists(sig_path_a)):
                        print(RED + f"[Error] Firmware/signature for slota not found for '{base_name}'" + RESET)
                        continue
                    if not (os.path.exists(fw_path_b) and os.path.exists(sig_path_b)):
                        print(RED + f"[Error] Firmware/signature for slotb not found for '{base_name}'" + RESET)
                        continue

                    # Query for active slot to determine which file to send
                    print("Querying for active slot...")
                    slot_status_event.clear()
                    active_slot = -1
                    await client.write_gatt_char(CHAR_RX_UUID, b"status\n")
                    try:
                        await asyncio.wait_for(slot_status_event.wait(), timeout=5.0)
                    except asyncio.TimeoutError:
                        print(RED + "Timeout: Could not determine active slot. Aborting update." + RESET)
                        continue

                    if active_slot not in [0, 1]:
                        print(RED + f"Invalid active slot ({active_slot}) received. Aborting update." + RESET)
                        continue
                    
                    # Select firmware for the INACTIVE slot
                    # Assumes slota -> partition 0, slotb -> partition 1
                    if active_slot == 0:
                        fw_path, sig_path = fw_path_b, sig_path_b
                        print(f"Active slot is 0. Updating slot 1 with '{os.path.basename(fw_path)}'")
                    else: # active_slot == 1
                        fw_path, sig_path = fw_path_a, sig_path_a
                        print(f"Active slot is 1. Updating slot 0 with '{os.path.basename(fw_path)}'")

                    print(f"Initiating update for firmware: '{base_name}'")
                    # Send "update" command to put bootloader into OTA transaction mode
                    await client.write_gatt_char(CHAR_RX_UUID, b"update\n")
                    
                    await asyncio.sleep(0.1) # Give bootloader a moment to switch modes

                    # Start the detailed OTA transaction
                    await send_ota_sequence(client, fw_path, sig_path)
                else:
                    await client.write_gatt_char(CHAR_RX_UUID, (cmd + "\n").encode())
            await client.stop_notify(CHAR_TX_UUID)
    except EOFError:
        pass  # Ignore disconnect errors on exit
    except Exception as e:
        print(RED + f"An unexpected error occurred: {e}" + RESET)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")

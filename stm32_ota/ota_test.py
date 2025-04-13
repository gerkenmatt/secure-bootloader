import serial
import struct
import threading
import time
import zlib
from threading import Event

# === Configuration ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
CHUNK_SIZE = 128
ACK_TIMEOUT = 2.0
USE_CRC32 = True # Set to True to enable real CRC32

# === Protocol Constants ===
SOF  = 0xA5
EOF  = 0xB6

PACKET_CMD    = 0x01
PACKET_HEADER = 0x02
PACKET_DATA   = 0x03
PACKET_RESP   = 0x04

CMD_START = 0xA0
CMD_END   = 0xA1

RESP_ACK  = 0xAB
RESP_NACK = 0xCD

# === Color Codes ===
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

# === State ===
ser_lock = threading.Lock()
uart_reading_enabled = Event()
uart_reading_enabled.set()

# === Utility ===
def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF if USE_CRC32 else 0

def build_frame(packet_type, payload: bytes) -> bytes:
    return (
        bytearray([SOF, packet_type])
        + struct.pack('<H', len(payload))
        + payload
        + struct.pack('<I', crc32(payload))
        + bytearray([EOF])
    )

# === Serial Communication ===
def wait_for_ack(ser, timeout=ACK_TIMEOUT) -> bool:
    start = time.time()
    buffer = bytearray()

    while time.time() - start < timeout:
        with ser_lock:
            data = ser.read(10 - len(buffer))  # Read as many bytes as needed
        if data:
            buffer.extend(data)
            if len(buffer) >= 10 and buffer[0] == SOF and buffer[1] == PACKET_RESP:
                status = buffer[4]
                if status == RESP_ACK:
                    print(BLUE + "  --> ACK received" + RESET)
                    return True
                elif status == RESP_NACK:
                    uart_reading_enabled.set()
                    print(RED + f"  --> NACK received" + RESET)
                    return False

    if buffer:
        print(YELLOW + f"Partial/invalid data before timeout. Full buffer: {buffer.hex()}" + RESET)
    print(RED + "Timeout: No valid ACK received" + RESET)
    return False



def read_from_uart(ser):
    while True:
        uart_reading_enabled.wait()
        try:
            with ser_lock:
                line = ser.readline()
            if line:
                print("    " + GREEN + line.decode(errors='ignore').strip() + RESET)
        except:
            continue

# === OTA Logic ===
def send_cmd(ser, cmd_id, wait=True):
    ser.write(build_frame(PACKET_CMD, bytes([cmd_id])))
    if wait: 
        return wait_for_ack(ser)
    else: 
        return True

def send_header(ser, fw_size, fw_crc32, version=0):
    payload = struct.pack('<III', fw_size, fw_crc32, version) + b'\x00' * 4
    ser.reset_input_buffer()  # Clear UART buffer before waiting for ACK
    ser.write(build_frame(PACKET_HEADER, payload))
    return wait_for_ack(ser)


def send_data_chunks(ser, fw_data):
    for i in range(0, len(fw_data), CHUNK_SIZE):
        print("sending chunk " + str(i))
        chunk = fw_data[i:i+CHUNK_SIZE]
        ser.write(build_frame(PACKET_DATA, chunk))

        if not wait_for_ack(ser):
            print(RED + f"  --> No ACK for chunk {i // CHUNK_SIZE}" + RESET)
            return False
    return True

def send_ota_sequence(ser, filepath):
    try:
        with open(filepath, 'rb') as f:
            fw_data = f.read()
    except FileNotFoundError:
        print(f"[Error] File not found: {filepath}")
        return

    fw_size = len(fw_data)
    fw_crc32 = crc32(fw_data)

    print(f"Firmware size: {fw_size} bytes")
    print(f"CRC32: 0x{fw_crc32:08X}")

    # Disable UART reading when the OTA sequence is being sent
    uart_reading_enabled.clear()

    print("Sending OTA_START")
    if not send_cmd(ser, CMD_START):
        uart_reading_enabled.set()
        return

    print("Sending header")
    if not send_header(ser, fw_size, fw_crc32):
        uart_reading_enabled.set()
        return

    print("Sending firmware data...")
    if not send_data_chunks(ser, fw_data):
        uart_reading_enabled.set()
        return

    # Re-enable UART reading at the end of the OTA sequence
    uart_reading_enabled.set()

    print("Sending OTA_END")
    send_cmd(ser, CMD_END, False)


# === Main Interface ===
def main():
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) as ser:
                time.sleep(2)
                print(CYAN + BOLD + "\r\nConnected. Type: ota, run, help, send\n" + RESET)
                threading.Thread(target=read_from_uart, args=(ser,), daemon=True).start()

                while True:
                    try:
                        cmd = input().strip()
                        if not cmd:
                            continue
                        if cmd == "send":
                            filepath = input("Enter firmware file path: ").strip()
                            send_ota_sequence(ser, filepath)
                        else:
                            ser.write((cmd + '\n').encode())
                    except KeyboardInterrupt:
                        print("\nExiting.")
                        return
        except serial.SerialException as e:
            print(RED + f"[Serial Error] {e}" + RESET)
            print(f"Retrying in 5 seconds... (Check {SERIAL_PORT})")
            time.sleep(5)

if __name__ == "__main__":
    main()
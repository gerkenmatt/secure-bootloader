# ota_host.py
import struct
import zlib
import time
import serial

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

CHUNK_SIZE = 128
FRAME_LEN = 10


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


class OTAHost:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for device reset
        self.flush()

    def flush(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def send_text_cmd(self, cmd: str):
        """Send a plain text command like 'ota', 'run', etc."""
        self.ser.write((cmd.strip() + "\n").encode())

    def read_output(self, delay=0.2):
        """Read all output from device (non-frame)."""
        time.sleep(delay)
        return self.ser.read_all().decode(errors="ignore")


    def wait_for_ack(self, timeout=2.0):
        start = time.time()
        buffer = bytearray()

        while time.time() - start < timeout:
            data = self.ser.read(FRAME_LEN - len(buffer))
            if data:
                buffer.extend(data)
                if len(buffer) >= FRAME_LEN and buffer[0] == SOF and buffer[1] == PACKET_RESP:
                    status = buffer[4]
                    if status == RESP_ACK: 
                        return True;
                    if status == RESP_NACK: 
                        print("[DEBUG] NACK received")
                        return False
        print("[DEBUG] No ACK Received")
        return False

    def send_cmd(self, cmd_id, wait=True):
        print(f"[DEBUG] send_cmd: sending CMD=0x{cmd_id:02X}")
        self.ser.write(build_frame(PACKET_CMD, bytes([cmd_id])))
        return self.wait_for_ack() if wait else True

    def send_header(self, fw_size, fw_crc32, version=0):
        payload = struct.pack('<III', fw_size, fw_crc32, version) + b'\x00' * 4
        self.ser.reset_input_buffer()
        self.ser.write(build_frame(PACKET_HEADER, payload))
        return self.wait_for_ack()

    def send_data_chunks(self, fw_data):
        for i in range(0, len(fw_data), CHUNK_SIZE):
            chunk = fw_data[i:i+CHUNK_SIZE]
            print(f"[DEBUG] Sending chunk {i // CHUNK_SIZE}: {chunk[:8].hex()}")
            self.ser.write(build_frame(PACKET_DATA, chunk))
            if not self.wait_for_ack():
                print(f"[ERROR] Chunk {i // CHUNK_SIZE} failed")
                return False
        return True

    def send_ota_sequence(self, fw_data):
        self.send_text_cmd("ota")
        time.sleep(0.1)
        self.flush()

        if not self.send_cmd(CMD_START): 
            print("[ERROR] CMD_START failed")
            return False
        if not self.send_header(len(fw_data), crc32(fw_data)): 
            print("[ERROR] send_header failed")
            return False
        if not self.send_data_chunks(fw_data): 
            print("[ERROR] send_data_chunks failed")
            return False
        return self.send_cmd(CMD_END, wait=False)

    def is_device_ready(self):
        return self.ser and self.ser.is_open

    # def reset_device(self):
    #     """Toggle DTR/RTS to reset the board if supported."""
    #     print("[DEBUG] Resetting device")
    #     self.ser.setDTR(False)
    #     self.ser.setRTS(True)
    #     time.sleep(0.1)
    #     self.ser.setRTS(False)
    #     time.sleep(1.5)  # allow board to reboot into bootloader
    #     self.ser.reset_input_buffer()
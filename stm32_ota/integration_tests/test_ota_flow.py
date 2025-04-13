import pytest
import time
from ota_host import OTAHost, crc32, RESP_ACK, RESP_NACK, build_frame, PACKET_DATA, CMD_START, CMD_END

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
CHUNK_SIZE = 128


@pytest.fixture(scope="module")
def ota():
    host = OTAHost(SERIAL_PORT, BAUDRATE)
    yield host
    host.close()

def test_ota_successful(ota):
    print("[TEST] Starting OTA success test...")
    firmware_path = "test_binaries/valid_firmware.bin"
    with open(firmware_path, "rb") as f:
        fw_data = f.read()

    assert ota.send_ota_sequence(fw_data)
    print("[TEST] OTA sequence result: Success")

    # Give the device time to reboot
    time.sleep(1)

    ota.send_text_cmd("run")
    print("[TEST] Sent 'run' command. Reading output...")

    # Give the device time to load the application
    time.sleep(3)

    output = ota.read_output()
    print(f"[TEST] Bootloader output:\n{output}")
    assert "Jumping to application" in output

def test_ota_crc_failure(ota):

    ota.reset_device()
    time.sleep(2)

    with open("test_binaries/valid_firmware.bin", "rb") as f:
        data = bytearray(f.read())
    data[10] ^= 0xFF  # Corrupt one byte

    fw_size = len(data)
    fw_crc = crc32(data)

    assert ota.send_cmd(CMD_START)
    assert ota.send_header(fw_size, fw_crc)

    corrupted_chunk = data[:CHUNK_SIZE]
    ota.ser.write(build_frame(PACKET_DATA, corrupted_chunk))
    ack = ota.ser.read(10)
    assert ack[4] == RESP_NACK

def test_bootloader_recovery_from_bad_slot1(ota):
    ota.send_text_cmd("run")
    time.sleep(2)
    output = ota.read_output()
    assert "CRC check failed" in output or "slot 1 CRC check failed" in output

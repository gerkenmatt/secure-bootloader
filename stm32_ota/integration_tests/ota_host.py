import asyncio
import struct
import zlib
import time # For wait_for_log_message timeout
from bleak import BleakClient, BleakScanner, BleakError

# === OTA Protocol Constants ===
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

# === BLE Configuration ===
DEFAULT_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
DEFAULT_CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
DEFAULT_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# Color Codes
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m" # Retained for potential verbose ACK/NACK distinction
RESET = "\033[0m"

def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF

def build_frame(packet_type: int, payload: bytes) -> bytes:
    payload_len = len(payload)
    frame = bytearray([SOF, packet_type])
    frame.extend(struct.pack('<H', payload_len))
    frame.extend(payload)
    frame.extend(struct.pack('<I', crc32(payload)))
    frame.append(EOF)
    return bytes(frame)

class OTAHost:
    def __init__(self, device_name: str,
                 service_uuid: str = DEFAULT_SERVICE_UUID,
                 char_rx_uuid: str = DEFAULT_CHAR_RX_UUID,
                 char_tx_uuid: str = DEFAULT_CHAR_TX_UUID,
                 verbose: bool = False): # Added verbose flag
        self.device_name = device_name
        self.service_uuid = service_uuid
        self.char_rx_uuid = char_rx_uuid
        self.char_tx_uuid = char_tx_uuid
        self.verbose = verbose # Store verbose flag

        self.client: BleakClient | None = None
        self.ack_event = asyncio.Event()
        self.log_event = asyncio.Event()
        self.last_status: int | None = None
        self.device_logs = []
        self._notification_buffer = bytearray()

    async def connect(self, timeout=10.0) -> bool:
        if self.verbose:
            print(YELLOW + f"Scanning for device: {self.device_name}..." + RESET)
        try:
            device = await BleakScanner.find_device_by_name(self.device_name, timeout=timeout)
            if not device:
                print(RED + f"Device '{self.device_name}' not found." + RESET) # Error, always print
                return False

            if self.verbose:
                print(GREEN + f"Found device: {device.name} ({device.address})" + RESET)
            self.client = BleakClient(device)
            if self.verbose:
                print(YELLOW + "Connecting..." + RESET)
            await self.client.connect()
            if self.verbose:
                print(GREEN + "Connected." + RESET)

            if self.verbose:
                print(YELLOW + f"Starting notifications on {self.char_tx_uuid}" + RESET)
            await self.client.start_notify(self.char_tx_uuid, self._handle_notification)
            if self.verbose:
                print(GREEN + "Notifications started." + RESET)
            return True
        except BleakError as e:
            print(RED + f"BLE Error during connect/setup: {e}" + RESET) # Error
            self.client = None
            return False
        except Exception as e:
            print(RED + f"Unexpected error during connect/setup: {e}" + RESET) # Error
            self.client = None
            return False

    async def disconnect(self):
        if self.client and self.client.is_connected:
            if self.verbose:
                print(YELLOW + "Disconnecting..." + RESET)
            try:
                await self.client.stop_notify(self.char_tx_uuid)
            except Exception as e:
                print(RED + f"Error stopping notifications: {e}" + RESET) # Error
            try:
                await self.client.disconnect()
                if self.verbose:
                    print(GREEN + "Disconnected." + RESET)
            except Exception as e:
                print(RED + f"Error during disconnect: {e}" + RESET) # Error
        self.client = None
        self.ack_event.clear()
        self.log_event.clear()
        self.last_status = None

    def _process_incoming_data(self, data: bytes):
        is_response_frame_processed = False
        if len(data) >= 10 and data[0] == SOF and data[1] == PACKET_RESP and data[-1] == EOF:
            try:
                payload_len_from_field = struct.unpack('<H', data[2:4])[0]
                expected_total_len = 1 + 1 + 2 + payload_len_from_field + 4 + 1
                if len(data) == expected_total_len:
                    payload = data[4 : 4 + payload_len_from_field]
                    received_crc_bytes = data[4 + payload_len_from_field : 4 + payload_len_from_field + 4]
                    received_crc = struct.unpack('<I', received_crc_bytes)[0]
                    calculated_crc = crc32(payload)

                    if payload_len_from_field == 1: # Standard 1-byte ACK/NACK
                        is_response_frame_processed = True
                        status = payload[0]
                        self.last_status = status
                        # MODIFIED: Only print NACKs/Unknown, or if verbose print ACK too
                        if status != RESP_ACK or self.verbose:
                            status_str = "ACK" if status == RESP_ACK else "NACK" if status == RESP_NACK else f"UNK_STAT({status:#02X})"
                            color = BLUE if status == RESP_ACK else RED
                            print(color + f"  <-- {status_str}" + RESET)
                        self.ack_event.set()
                    elif received_crc == calculated_crc:
                        is_response_frame_processed = True
                        if self.verbose:
                             print(YELLOW + f"  <-- Frame: Valid PACKET_RESP (p_len={payload_len_from_field}, CRC OK): {payload.hex()}" + RESET)
                        self.ack_event.set()
                    else:
                        print(RED + f"  <-- Frame: Invalid PACKET_RESP (p_len={payload_len_from_field}, CRC BAD - RX:{received_crc:#08X}, CALC:{calculated_crc:#08X}). Data: {data.hex()}. Treating as log." + RESET) # Error
            except (struct.error, IndexError) as e:
                print(RED + f"  <-- Error parsing PACKET_RESP fields: {e}. Data: {data.hex()}. Treating as log." + RESET) # Error

        # Treat as log if not a processed response frame
        if not is_response_frame_processed: 
            try:
                line = data.decode("utf-8", errors="replace").strip()
                if line:
                    log_msg = f"  DEV_LOG: {line}" # Keep DEV_LOGs
                    print(GREEN + log_msg + RESET)
                    self.device_logs.append(line)
                    self.log_event.set()
            except Exception as e:
                log_msg_raw = f"  DEV_LOG (raw, decode_err {e}): {data.hex()}" # Keep DEV_LOGs
                print(GREEN + log_msg_raw + RESET)
                self.device_logs.append(data.hex())
                self.log_event.set()

    def _handle_notification(self, _: int | str, data: bytearray):
        self.log_event.clear()
        self._process_incoming_data(bytes(data))

    async def clear_device_logs(self):
        self.device_logs = []
        self.log_event.clear()

    async def wait_for_ack(self, timeout=5.0) -> bool:
        if not self.client or not self.client.is_connected:
            print(RED + "Not connected. Cannot wait for ACK." + RESET) # Error
            return False
        try:
            await asyncio.wait_for(self.ack_event.wait(), timeout)
            self.ack_event.clear()
            return self.last_status == RESP_ACK
        except asyncio.TimeoutError:
            print(RED + f"Timeout waiting for ACK/NACK (>{timeout}s)." + RESET)
            self.last_status = None
            return False
        except Exception as e:
            print(RED + f"Error waiting for ACK: {e}" + RESET) # Error
            return False

    async def wait_for_log_message(self, log_substring: str, timeout: float = 10.0, quiet_success: bool = False) -> bool:
        if not self.client or not self.client.is_connected:
            print(RED + "Not connected. Cannot wait for log." + RESET) # Error
            return False

        if self.verbose and not quiet_success:
            print(YELLOW + f"Waiting for log: '{log_substring}' (timeout: {timeout}s)" + RESET)

        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout:
            current_logs = list(self.device_logs)
            for log_entry in reversed(current_logs):
                if log_substring in log_entry:
                    if self.verbose and not quiet_success:
                        print(GREEN + f"Log found: '{log_substring}'" + RESET)
                    return True
            self.log_event.clear()
            try:
                remaining_timeout = timeout - (time.monotonic() - start_time)
                if remaining_timeout <= 0: break
                await asyncio.wait_for(self.log_event.wait(), timeout=min(0.5, remaining_timeout))
            except asyncio.TimeoutError:
                pass
        print(RED + f"Timeout waiting for log: '{log_substring}'" + RESET) # Error
        return False

    async def send_text_command(self, command: str,
                                expected_response_log: str | None = None,
                                log_timeout: float = 5.0) -> bool:
        if not self.client or not self.client.is_connected:
            print(RED + "Not connected. Cannot send text command." + RESET) # Error
            return False
        if self.verbose:
            print(f"Sending text: '{command}'")
        try:
            await self.client.write_gatt_char(self.char_rx_uuid, (command.strip() + "\n").encode("utf-8"), response=False)
            if expected_response_log:
                return await self.wait_for_log_message(expected_response_log, timeout=log_timeout, quiet_success=not self.verbose)
            return True
        except Exception as e:
            print(RED + f"Error sending text '{command}': {e}" + RESET) # Error
            return False

    async def _send_frame_and_wait_for_ack(self, packet_type: int, payload: bytes, ack_timeout=5.0) -> bool:
        if not self.client or not self.client.is_connected:
            print(RED + "Not connected. Cannot send frame." + RESET) # Error
            return False
        frame = build_frame(packet_type, payload)
        try:
            self.ack_event.clear()
            await self.client.write_gatt_char(self.char_rx_uuid, frame, response=False)
            return await self.wait_for_ack(timeout=ack_timeout)
        except Exception as e:
            print(RED + f"Error sending frame type {packet_type:#02X}: {e}" + RESET) # Error
            return False

    async def send_cmd(self, cmd_id: int, ack_timeout=5.0) -> bool:
        if self.verbose:
            print(f"Sending CMD {cmd_id:#02X}")
        return await self._send_frame_and_wait_for_ack(PACKET_CMD, bytes([cmd_id]), ack_timeout)

    async def send_header(self, fw_size: int, fw_crc: int, version: int = 0, ack_timeout=5.0) -> bool:
        payload = struct.pack('<III', fw_size, fw_crc, version) + b'\x00' * 4
        if self.verbose:
            print(f"Sending Header: Size={fw_size}, CRC=0x{fw_crc:08X}, Ver={version}")
        return await self._send_frame_and_wait_for_ack(PACKET_HEADER, payload, ack_timeout)

    async def send_data_chunks(self, fw_data: bytes, ack_timeout=2.0) -> bool:
        num_total_chunks = (len(fw_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        
        if self.verbose:
            print(f"Sending FW Data ({len(fw_data)}B in {num_total_chunks} chunks)...")
        elif num_total_chunks > 0: 
            print(GREEN + f"Sending FW Data ({len(fw_data)}B)... " + RESET, end='', flush=True) 

        # For non-verbose mode, print a dot every N chunks. Adjust as needed.
        progress_dot_interval = 10 
        if num_total_chunks <= 50: # For fewer chunks, print dots more frequently
            progress_dot_interval = 5
        if num_total_chunks <= 20:
            progress_dot_interval = 1


        for i in range(0, len(fw_data), CHUNK_SIZE):
            chunk = fw_data[i:i+CHUNK_SIZE]
            chunk_num = (i // CHUNK_SIZE) + 1

            if self.verbose:
                # Verbose logging for chunk sending
                print_interval = max(1, num_total_chunks // 10) 
                if chunk_num == 1 or chunk_num % print_interval == 0 or chunk_num == num_total_chunks:
                    print(f"  Sending FW Chunk {chunk_num}/{num_total_chunks} ({len(chunk)}B)")
            elif num_total_chunks > 0 and (chunk_num % progress_dot_interval == 0 or chunk_num == num_total_chunks) : 
                # Non-verbose progress indicator: print a dot
                print(GREEN + "." + RESET, end='', flush=True)

            if not await self._send_frame_and_wait_for_ack(PACKET_DATA, chunk, ack_timeout):
                if not self.verbose and num_total_chunks > 0:
                    print() # Newline to clean up if progress dots were printed before error
                print(RED + f"  NACK/Timeout for FW Chunk {chunk_num}." + RESET) # Error
                return False

        if not self.verbose and num_total_chunks > 0:
            print(GREEN + " Done." + RESET) # Finish the progress line for non-verbose mode
        elif self.verbose and num_total_chunks > 0 : 
            print(GREEN + f"  All {num_total_chunks} FW chunks sent successfully." + RESET)
        return True


    async def send_sig_chunks(self, sig_data: bytes, ack_timeout=2.0) -> bool:
        if not sig_data:
            if self.verbose:
                print(YELLOW + "Signature data empty. Skipping send." + RESET)
            return True
        num_total_chunks = (len(sig_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        if self.verbose:
            print(f"Sending Sig Data ({len(sig_data)}B in {num_total_chunks} chunks)...")
        for i in range(0, len(sig_data), CHUNK_SIZE):
            chunk = sig_data[i:i+CHUNK_SIZE]
            chunk_num = (i // CHUNK_SIZE) + 1
            if self.verbose : 
                 print(f"  Sending Sig Chunk {chunk_num}/{num_total_chunks} ({len(chunk)}B)")
            if not await self._send_frame_and_wait_for_ack(PACKET_SIG, chunk, ack_timeout):
                print(RED + f"  NACK/Timeout for Sig Chunk {chunk_num}." + RESET) # Error
                return False
        if self.verbose and num_total_chunks > 0:
             print(GREEN + f"  All {num_total_chunks} SIG chunks sent successfully." + RESET)
        return True

    async def perform_full_ota(self, fw_data: bytes, sig_data: bytes,
                               initial_ota_command: str = "ota",
                               ota_mode_log: str = "Entering OTA mode...",
                               post_ota_ready_log: str = "Bootloader ready. Waiting for command",
                               log_timeout: float = 10.0,
                               reboot_timeout: float = 20.0
                               ) -> bool:
        if self.verbose:
            print(YELLOW + "Starting Full OTA Sequence..." + RESET)
        await self.clear_device_logs()

        if self.verbose:
            print(f"Entering OTA mode with '{initial_ota_command}'...")
        if not await self.send_text_command(initial_ota_command, expected_response_log=ota_mode_log, log_timeout=log_timeout):
            print(RED + f"Failed to enter OTA mode (expected '{ota_mode_log}'). Aborting." + RESET) # Error
            return False

        if not await self.send_cmd(CMD_START):
            print(RED + "CMD_START failed. Aborting OTA." + RESET) # Error
            return False

        fw_size = len(fw_data)
        fw_crc = crc32(fw_data)
        if not await self.send_header(fw_size, fw_crc):
            print(RED + "Header send failed. Aborting OTA." + RESET) # Error
            return False

        if not await self.send_data_chunks(fw_data):
            print(RED + "FW Data send failed. Aborting OTA." + RESET) # Error
            return False

        if not await self.send_sig_chunks(sig_data):
            print(RED + "Sig Data send failed. Aborting OTA." + RESET) # Error
            return False

        if self.verbose:
            print("Sending CMD_END for final verification...")
        if not await self.send_cmd(CMD_END):
            print(RED + "CMD_END failed. OTA not finalized." + RESET) # Error
            return False
        if self.verbose:
            print(GREEN + "CMD_END ACKed by device." + RESET)

        if self.verbose:
            print(YELLOW + f"Waiting for reboot and '{post_ota_ready_log}' (>{reboot_timeout}s)..." + RESET)
        if await self.wait_for_log_message(post_ota_ready_log, timeout=reboot_timeout, quiet_success=not self.verbose):
            if self.verbose: 
                print(GREEN + "Full OTA Sequence Successful: Device ready post-OTA." + RESET)
            return True
        else:
            print(RED + f"Full OTA Failed: Device not ready ('{post_ota_ready_log}') post-OTA." + RESET) # Error
            return False
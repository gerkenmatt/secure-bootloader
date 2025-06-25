import pytest
import pytest_asyncio
import asyncio
import os
import re
import struct

from ota_host import OTAHost, crc32

# ==============================================================================
# === 1. CENTRALIZED CONSTANTS
# ==============================================================================

# --- Test Configuration ---
DEVICE_NAME = "ESP32_OTA_BLE"
TEST_FW_DIR = "../test_data/"

# --- Terminal Colors ---
BLUE = "\033[94m"
PURPLE = "\033[95m"
CYAN = "\033[96m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"

# --- Firmware File Paths ---
VALID_FW_A_SLOTA = os.path.join(TEST_FW_DIR, "firmware_A_slota.bin")
VALID_SIG_A_SLOTA = os.path.join(TEST_FW_DIR, "firmware_A_slota.sig")
VALID_FW_A_SLOTB = os.path.join(TEST_FW_DIR, "firmware_A_slotb.bin")
VALID_SIG_A_SLOTB = os.path.join(TEST_FW_DIR, "firmware_A_slotb.sig")

VALID_FW_B_SLOTA = os.path.join(TEST_FW_DIR, "firmware_B_slota.bin")
VALID_SIG_B_SLOTA = os.path.join(TEST_FW_DIR, "firmware_B_slota.sig")
VALID_FW_B_SLOTB = os.path.join(TEST_FW_DIR, "firmware_B_slotb.bin")
VALID_SIG_B_SLOTB = os.path.join(TEST_FW_DIR, "firmware_B_slotb.sig")

VALID_FW_LARGE_SLOTA = os.path.join(TEST_FW_DIR, "firmware_large_slota.bin")
VALID_SIG_LARGE_SLOTA = os.path.join(TEST_FW_DIR, "firmware_large_slota.sig")
VALID_FW_LARGE_SLOTB = os.path.join(TEST_FW_DIR, "firmware_large_slotb.bin")
VALID_SIG_LARGE_SLOTB = os.path.join(TEST_FW_DIR, "firmware_large_slotb.sig")

DIFFERENT_KEY_SIG = os.path.join(TEST_FW_DIR, "firmware_A_slota_diff_key.sig")
WRONG_HASH_SIG = os.path.join(TEST_FW_DIR, "firmware_B_slota.sig")

# ==============================================================================
# === 2. SHARED HELPER COROUTINES
# ==============================================================================

async def reboot_and_wait_for_ready(host: OTAHost, reboot_cmd_delay: float = 2.0, ready_timeout: float = 15.0) -> bool:
    """Consolidated helper to reboot the device and wait for readiness."""
    if host.verbose:
        print(YELLOW + "  HELPER: Rebooting device and waiting for readiness..." + RESET)
    await host.clear_device_logs()
    
    await host.send_text_command(command="reboot")
    await asyncio.sleep(reboot_cmd_delay)

    # Wait for the bootloader to be ready to accept new commands
    if not await host.wait_for_log_message("Bootloader ready. Waiting for command", timeout=ready_timeout):
        print(RED + "  HELPER: Device did not signal readiness after reboot." + RESET)
        return False
    
    if host.verbose:
        print(GREEN + "  HELPER: Device rebooted and is ready." + RESET)
    return True

async def perform_simple_ota(host: OTAHost, fw_path: str, sig_path: str) -> bool:
    """A wrapper for performing a basic, successful OTA update."""
    if not os.path.exists(fw_path) or not os.path.exists(sig_path):
        pytest.fail(f"Missing OTA files: {fw_path} or {sig_path}")
    
    with open(fw_path, "rb") as f: fw_data = f.read()
    with open(sig_path, "rb") as f: sig_data = f.read()

    return await host.perform_full_ota(fw_data, sig_data)

async def get_bootloader_info(host: OTAHost) -> dict | None:
    """Sends 'info' and parses the output into a dictionary."""
    await host.clear_device_logs()
    if host.verbose:
        print(YELLOW + "  HELPER: Requesting bootloader info..." + RESET)
    
    if not await host.send_text_command("info"):
        return None
    
    # Wait for a line that indicates the end of the info block
    if not await host.wait_for_log_message("| Slot B:", timeout=5):
        print(RED + "  HELPER: Did not receive 'Slot B' part of info block." + RESET)
        return None
    await asyncio.sleep(0.5)

    if host.verbose:
        print(CYAN + f"--- Received Info Block ---\n" + "\n".join(host.device_logs) + f"\n---------------------------" + RESET)

    info_dict = {}
    slot_context_dict = None
    try:
        for line in host.device_logs:
            line = line.strip()
            if not line.startswith('|') or ":" not in line: continue

            if "| Slot A:" in line:
                info_dict["slot_a"] = {}
                slot_context_dict = info_dict["slot_a"]
                continue
            elif "| Slot B:" in line:
                info_dict["slot_b"] = {}
                slot_context_dict = info_dict["slot_b"]
                continue

            parts = line.split(':', 1)
            key = parts[0].replace('|', '').strip().replace(' ', '_')
            value_str = parts[1].strip()

            if '0x' in value_str: value = value_str
            elif value_str.lower() == 'true': value = True
            elif value_str.lower() == 'false': value = False
            elif value_str.split(" ")[0].isdigit(): value = int(value_str.split(" ")[0])
            else: value = value_str
            
            if slot_context_dict is not None and key in ["is_valid", "boot_attempts", "fw_size", "fw_crc"]:
                 slot_context_dict[key] = value
            else: 
                 info_dict[key] = value
    except Exception as e:
        print(RED + f"  HELPER: Failed to parse info block: {e}\nRaw text:\n" + "\n".join(host.device_logs) + RESET)
        return None
    
    return info_dict

async def get_slot_memory_words(host: OTAHost, slot: int) -> list[int] | None:
    """Sends 'print <slot>' and parses the hex output."""
    await host.clear_device_logs()
    command = f"print {slot}"
    if not await host.send_text_command(command, f"First 10 words of Slot {slot}"):
        print(RED + f"Did not receive response for '{command}'" + RESET)
        return None
    
    await asyncio.sleep(0.2)
    
    output_line = next((log for log in host.device_logs if log.startswith("0x")), None)

    if not output_line:
        print(RED + "Could not find data line in 'print' command output." + RESET)
        return None
    
    try:
        return [int(val, 16) for val in output_line.split()]
    except (ValueError, IndexError) as e:
        print(RED + f"Failed to parse 'print' output: {e}\nRaw line: '{output_line}'" + RESET)
        return None

# ==============================================================================
# === 3. SHARED PYTEST FIXTURE
# ==============================================================================

@pytest_asyncio.fixture(scope="function")
async def ota_host_fixture(request):
    """
    Centralized fixture to set up and tear down the OTAHost connection.
    - Connects to the device.
    - Performs an initial reboot to ensure a known, clean state.
    - Yields the connected host to the test function.
    - Disconnects cleanly after the test is done.
    """
    # Determine verbosity from pytest command line (--verbose) or env var
    is_verbose = request.config.getoption("verbose") > 0 or os.getenv("TEST_VERBOSE", "False").lower() == "true"
    
    print(PURPLE + BOLD + f"\n{'='*20} SETUP FIXTURE: ota_host_fixture (Verbose: {is_verbose}) {'='*20}" + RESET)
    host = OTAHost(DEVICE_NAME, verbose=is_verbose)
    
    if not await host.connect():
        pytest.fail(f"Fixture: Failed to connect to device '{DEVICE_NAME}'", pytrace=False)

    print(PURPLE + "Fixture: Connected. Performing initial reboot to ensure known state..." + RESET)
    if not await reboot_and_wait_for_ready(host, reboot_cmd_delay=4.0, ready_timeout=20):
        pytest.fail("Fixture: Device failed to become ready after initial controlled reboot.", pytrace=False)
    
    # In some cases, a test might need a specific starting firmware.
    # The fixture in `test_slot_management` did a full OTA. We can keep that behavior
    # by uncommenting the block below if the reboot isn't sufficient.
    # For now, a simple reboot is a faster and cleaner start.
    # print(PURPLE + BOLD + "Fixture: Performing a clean OTA to guarantee a known starting state..." + RESET)
    # if not await perform_simple_ota(host, VALID_FW_A_SLOTA, VALID_SIG_A_SLOTA):
    #   pytest.fail("Fixture: Initial OTA failed. Cannot guarantee a clean state for tests.", pytrace=False)

    print(GREEN + BOLD + f"{'='*20} SETUP FIXTURE COMPLETE {'='*20}\n" + RESET)
    
    yield host  # Hand control over to the test function
    
    print(PURPLE + BOLD + f"\n{'='*20} TEARDOWN FIXTURE: ota_host_fixture {'='*20}" + RESET)
    await host.disconnect()
    print(PURPLE + BOLD + f"{'='*20} TEARDOWN FIXTURE COMPLETE {'='*20}\n" + RESET)

# ==============================================================================
# === 4. PYTEST HOOKS (Replaces setup_module)
# ==============================================================================

def pytest_sessionstart(session):
    """
    Pytest hook that runs once at the beginning of the entire test session.
    This is the ideal place to check for prerequisites like firmware files.
    """
    print(YELLOW + BOLD + f"\n{'#'*20} CHECKING TEST PREREQUISITES {'#'*20}" + RESET)
    print(YELLOW + f"Test firmware directory: '{TEST_FW_DIR}'" + RESET)
    os.makedirs(TEST_FW_DIR, exist_ok=True)

    required_files_info = {
        "Valid FW A (slota)": [VALID_FW_A_SLOTA, VALID_SIG_A_SLOTA],
        "Valid FW A (slotb)": [VALID_FW_A_SLOTB, VALID_SIG_A_SLOTB],
        "Valid FW B (slota)": [VALID_FW_B_SLOTA, VALID_SIG_B_SLOTA],
        "Valid FW B (slotb)": [VALID_FW_B_SLOTB, VALID_SIG_B_SLOTB],
        "Valid FW Large (slota)": [VALID_FW_LARGE_SLOTA, VALID_SIG_LARGE_SLOTA],
        "Valid FW Large (slotb)": [VALID_FW_LARGE_SLOTB, VALID_SIG_LARGE_SLOTB],
        "Wrong Hash Signature (Critical)": [WRONG_HASH_SIG],
        "Untrusted Key Signature (Optional)": [DIFFERENT_KEY_SIG],
    }
    all_critical_files_present = True

    print(CYAN + "Checking for required test files..." + RESET)
    for desc, paths in required_files_info.items():
        is_optional = "optional" in desc.lower()
        for path in paths:
            if not os.path.exists(path):
                if is_optional:
                    print(YELLOW + f"  OPTIONAL FILE MISSING (related test will be skipped): {desc} - {path}" + RESET)
                else:
                    print(RED + f"  CRITICAL FILE MISSING: {desc} - {path}" + RESET)
                    all_critical_files_present = False
            else:
                print(GREEN + f"  PRESENT: {desc} - {path}" + RESET)

    if not all_critical_files_present:
        pytest.fail("One or more critical test files are missing. Please generate/place them and retry.", pytrace=False)
    
    print(GREEN + BOLD + f"{'#'*20} PREREQUISITES MET {'#'*20}\n" + RESET)
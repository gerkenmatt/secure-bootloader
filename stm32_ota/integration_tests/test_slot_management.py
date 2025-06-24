import pytest
import pytest_asyncio
import asyncio
import os
import re

# Imports from your ota_host.py module
from ota_host import OTAHost

# --- Test Configuration (Adjust as needed) ---
DEVICE_NAME = "ESP32_OTA_BLE"
TEST_FW_DIR = "test_firmware"

# Using Firmware A for these tests as a representative valid firmware
VALID_FW_A_SLOTA = os.path.join(TEST_FW_DIR, "firmware_A_slota.bin")
VALID_SIG_A_SLOTA = os.path.join(TEST_FW_DIR, "firmware_A_slota.sig")
VALID_FW_A_SLOTB = os.path.join(TEST_FW_DIR, "firmware_A_slotb.bin")
VALID_SIG_A_SLOTB = os.path.join(TEST_FW_DIR, "firmware_A_slotb.sig")

# --- Terminal Colors ---
BLUE = "\033[94m"
PURPLE = "\033[95m"
CYAN = "\033[96m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"


# --- Helper Coroutines ---

async def reboot_and_wait_for_ready(host: OTAHost, reboot_cmd_delay: float = 2.0, ready_timeout: float = 15.0) -> bool:
    """Sends a reboot command and waits for the device to be ready."""
    if host.verbose:
        print(YELLOW + "  HELPER: Rebooting device and waiting for readiness..." + RESET)
    await host.clear_device_logs()
    
    await host.send_text_command(command="reboot")
    await asyncio.sleep(reboot_cmd_delay) 

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
    
    if not await host.wait_for_log_message("| Slot B:", timeout=5):
        print(RED + "  HELPER: Did not receive 'Slot B' part of info block." + RESET)
        return None
    await asyncio.sleep(0.5) 

    # Use host.device_logs directly to avoid NameError
    if host.verbose:
        print(CYAN + f"--- Received Info Block ---\n" + "\n".join(host.device_logs) + f"\n---------------------------" + RESET)

    info_dict = {}
    slot_context_dict = None
    try:
        for line in host.device_logs:
            line = line.strip()
            if not line.startswith('|') or ":" not in line:
                continue

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
            
            # Check if key belongs in a slot context
            if slot_context_dict is not None and key in ["is_valid", "boot_attempts", "fw_size", "fw_crc"]:
                 slot_context_dict[key] = value
            else: 
                 info_dict[key] = value
    except Exception as e:
        print(RED + f"  HELPER: Failed to parse info block: {e}\nRaw text:\n" + "\n".join(host.device_logs) + RESET)
        return None
    
    return info_dict


# --- Pytest Fixture ---

@pytest_asyncio.fixture(scope="function")
async def ota_host_fixture(request):
    """
    Sets up and tears down the OTAHost connection for each test.
    """
    is_verbose = os.getenv("TEST_VERBOSE", "False").lower() == "true" or request.config.getoption("verbose") > 0
    
    print(PURPLE + BOLD + f"\n{'='*20} SETUP FIXTURE: ota_host_fixture {'='*20}" + RESET)
    host = OTAHost(DEVICE_NAME, verbose=is_verbose)
    if not await host.connect():
         pytest.fail(f"Fixture: Failed to connect to device '{DEVICE_NAME}'", pytrace=False)

    print(PURPLE + BOLD + "Fixture: Performing a clean OTA to guarantee a known starting state..." + RESET)
    # We flash to Slot A. After this, Slot A will be active, Slot B will be the previous valid image.
    if not await perform_simple_ota(host, VALID_FW_A_SLOTA, VALID_SIG_A_SLOTA):
        pytest.fail("Fixture: Initial OTA failed. Cannot guarantee a clean state for tests.", pytrace=False)

    print(GREEN + BOLD + f"{'='*20} SETUP FIXTURE COMPLETE {'='*20}\n" + RESET)
    
    yield host
    
    print(PURPLE + BOLD + f"\n{'='*20} TEARDOWN FIXTURE: ota_host_fixture {'='*20}" + RESET)
    await host.disconnect()
    print(PURPLE + BOLD + f"{'='*20} TEARDOWN FIXTURE COMPLETE {'='*20}\n" + RESET)


# --- Slot Management Integration Tests ---

@pytest.mark.asyncio
async def test_manual_slot_activation_and_persistence(ota_host_fixture: OTAHost):
    """
    Test 1: Verifies the 'activate' command correctly changes the active slot
    and that the setting persists across a reboot.
    """
    test_name = "test_manual_slot_activation_and_persistence"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    initial_slot = await host.get_active_slot()
    assert initial_slot in [0, 1], f"Could not determine initial active slot, got: {initial_slot}"
    
    target_slot = 1 if initial_slot == 0 else 0
    print(f"Initial active slot is {initial_slot}. Activating target slot {target_slot}...")
    
    assert await host.send_text_command(f"activate {target_slot}", f"Active slot switched to: {target_slot}"), \
        f"Failed to receive confirmation for activating slot {target_slot}."

    print("Rebooting device to check for persistence...")
    assert await reboot_and_wait_for_ready(host), "Device failed to reboot."

    persistent_slot = await host.get_active_slot()
    assert persistent_slot == target_slot, f"Activation was not persistent. Expected {target_slot}, got {persistent_slot}."
    print(GREEN + f"SUCCESS: Slot activation persisted. Active slot is now {persistent_slot}." + RESET)
    
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_erase_inactive_slot_and_verify(ota_host_fixture: OTAHost):
    """
    Test 2: Verifies that erasing an inactive slot marks it as invalid
    without affecting the active slot.
    """
    test_name = "test_erase_inactive_slot_and_verify"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    
    active_slot = await host.get_active_slot()
    assert active_slot in [0, 1], f"Could not determine active slot, got: {active_slot}"
    
    slot_to_erase = 1 if active_slot == 0 else 0
    print(f"Active slot is {active_slot}. Erasing inactive slot {slot_to_erase}...")

    assert await host.send_text_command(f"erase {slot_to_erase}", "Boot config updated to mark slot as invalid."), \
        "Failed to receive confirmation for erasing slot."

    print("Checking bootloader info to confirm slot invalidation...")
    info = await get_bootloader_info(host)
    assert info, "Failed to get bootloader info."
    
    slot_key = f"slot_{'a' if slot_to_erase == 0 else 'b'}"
    assert info.get(slot_key, {}).get('is_valid') is False, f"Slot {slot_to_erase} was not marked as invalid."
    print(GREEN + f"SUCCESS: Slot {slot_to_erase} correctly marked as invalid." + RESET)

    current_active_slot = await host.get_active_slot()
    assert current_active_slot == active_slot, \
        f"Active slot changed unexpectedly. Expected {active_slot}, got {current_active_slot}."
    print(GREEN + f"SUCCESS: Active slot remained {current_active_slot}." + RESET)

    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_protection_against_erasing_active_slot(ota_host_fixture: OTAHost):
    """
    Test 3 (Corrected): Verifies the bootloader's safety feature
    that prevents a user from erasing the currently active slot.
    """
    test_name = "test_protection_against_erasing_active_slot"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    active_slot = await host.get_active_slot()
    assert active_slot in [0, 1], f"Could not determine active slot, got: {active_slot}"
    print(f"Current active slot is {active_slot}. Attempting to erase it (this should fail)...")
    
    assert await host.send_text_command(f"erase {active_slot}", "[ERROR] Cannot erase the currently active slot."), \
        "Did not receive expected error message when trying to erase the active slot."
    print(GREEN + "SUCCESS: Bootloader correctly refused to erase the active slot." + RESET)

    print("Checking bootloader info to confirm slot was NOT erased...")
    info = await get_bootloader_info(host)
    assert info, "Failed to get bootloader info after failed erase attempt."
    
    slot_key = f"slot_{'a' if active_slot == 0 else 'b'}"
    assert info.get(slot_key, {}).get('is_valid') is True, f"Slot {active_slot} was incorrectly marked as invalid."
    print(GREEN + f"SUCCESS: Active slot {active_slot} remains valid." + RESET)

    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)
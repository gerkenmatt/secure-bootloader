import pytest
import pytest_asyncio
import asyncio
import os

# Imports from the ota_host.py module
from ota_host import (
    OTAHost,
    RESP_ACK,
    RESP_NACK,
    CMD_START,
    CMD_END,
    crc32, # Import crc32 for use in the test logic if needed
)

# --- Test Configuration ---
DEVICE_NAME = "ESP32_OTA_BLE"
TEST_FW_DIR = "test_firmware"

VALID_FW_A_NAME = "firmware_A"
VALID_FW_B_NAME = "firmware_B" # Was firmware_small, remapped to firmware_B from file list
VALID_FW_LARGE_NAME = "firmware_large"

# --- Firmware A ---
VALID_FW_A_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_A_NAME}_slota.bin")
VALID_SIG_A_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_A_NAME}_slota.sig")
VALID_FW_A_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_A_NAME}_slotb.bin")
VALID_SIG_A_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_A_NAME}_slotb.sig")

# --- Firmware B  ---
VALID_FW_B_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_B_NAME}_slota.bin")
VALID_SIG_B_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_B_NAME}_slota.sig")
VALID_FW_B_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_B_NAME}_slotb.bin")
VALID_SIG_B_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_B_NAME}_slotb.sig")

# --- Firmware Large ---
VALID_FW_LARGE_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}_slota.bin")
VALID_SIG_LARGE_SLOTA = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}_slota.sig")
VALID_FW_LARGE_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}_slotb.bin")
VALID_SIG_LARGE_SLOTB = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}_slotb.sig")

DIFFERENT_KEY_SIG = os.path.join(TEST_FW_DIR, f"{VALID_FW_A_NAME}_slota_diff_key.sig")
WRONG_HASH_SIG = os.path.join(TEST_FW_DIR, f"{VALID_FW_B_NAME}_slota.sig") # Signature for firmware_B to be used with firmware_A

# --- Terminal Colors (for console output formatting) ---
BLUE = "\033[94m"
PURPLE = "\033[95m"
CYAN = "\033[96m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"

# --- Helper Coroutine for Reboot and Ready Check ---
async def reboot_and_wait_for_ready(host: OTAHost,
                                     reboot_cmd_delay: float = 1.0,
                                     boot_started_timeout: float = 15.0,
                                     ready_timeout: float = 10.0) -> bool:
    """
    Sends a reboot command to the device and waits for it to signal readiness.
    Handles verbosity for its logging through host.verbose.
    """
    if host.verbose:
        print(YELLOW + "  HELPER: Attempting to reboot device and wait for ready state..." + RESET)
    await host.clear_device_logs()

    if host.verbose:
        print(YELLOW + "  HELPER: Sending 'reboot' command..." + RESET)
    if not await host.send_text_command(command="reboot", expected_response_log=None):
        print(RED + "  HELPER: Failed to send 'reboot' command (or BLE write failed)." + RESET)
        if host.verbose:
            print(YELLOW + "  HELPER: Proceeding to wait for boot messages." + RESET)

    if host.verbose:
        print(YELLOW + f"  HELPER: Waiting {reboot_cmd_delay}s for reboot to initiate..." + RESET)
    await asyncio.sleep(reboot_cmd_delay)

    if not await host.wait_for_log_message("Bootloader started.", timeout=boot_started_timeout, quiet_success=not host.verbose):
        print(RED + "  HELPER: Did not receive 'Bootloader started.' log after reboot command." + RESET)
        return False
    if host.verbose:
        print(GREEN + "  HELPER: 'Bootloader started.' log received." + RESET)

    if not await host.wait_for_log_message("Bootloader ready. Waiting for command", timeout=ready_timeout, quiet_success=not host.verbose):
        print(RED + "  HELPER: Did not receive 'Bootloader ready. Waiting for command' log." + RESET)
        return False

    if host.verbose:
        print(GREEN + "  HELPER: Device rebooted and is ready." + RESET)
    return True

# --- Pytest Fixture for OTAHost  ---
@pytest_asyncio.fixture(scope="function")
async def ota_host_fixture(request): # 'request' is a built-in pytest fixture
    """
    Pytest fixture to set up and tear down the OTAHost connection for each test function.
    Initializes OTAHost, connects, performs an initial reboot, and handles disconnection.
    Verbosity is controlled by the --verbose flag passed to pytest or TEST_VERBOSE environment variable.
    """
    pytest_verbosity = request.config.getoption("verbose")
    if pytest_verbosity > 0:
        is_verbose = True
    else:
        is_verbose = os.getenv("TEST_VERBOSE", "False").lower() == "true"

    print(PURPLE + BOLD + f"\n{'='*20} SETUP FIXTURE: ota_host_fixture (Verbose: {is_verbose}) {'='*20}" + RESET)
    host = OTAHost(DEVICE_NAME, verbose=is_verbose)
    connected = await host.connect()
    if not connected:
        pytest.fail(RED + f"Fixture: Failed to connect to device '{DEVICE_NAME}' via OTAHost." + RESET)

    if host.verbose:
        print(YELLOW + "Fixture: Connected. Performing initial reboot to ensure known state..." + RESET)
    if not await reboot_and_wait_for_ready(host, reboot_cmd_delay=4.0, ready_timeout=20):
        pytest.fail(RED + "Fixture: Device failed to become ready after initial controlled reboot." + RESET)
    if host.verbose:
        print(GREEN + "Fixture: Device is initially rebooted and ready." + RESET)

    print(PURPLE + BOLD + f"{'='*20} SETUP FIXTURE COMPLETE {'='*20}\n" + RESET)
    yield host

    print(PURPLE + BOLD + f"\n{'='*20} TEARDOWN FIXTURE: ota_host_fixture {'='*20}" + RESET)
    if host.verbose:
        print(YELLOW + "\nFixture: Cleaning up OTAHost connection..." + RESET)
    await host.disconnect()
    if host.verbose:
        print(GREEN + "Fixture: OTAHost disconnected." + RESET)
    print(PURPLE + BOLD + f"{'='*20} TEARDOWN FIXTURE COMPLETE {'='*20}\n" + RESET)


# --- Helper Test OTA Sequence ---
async def perform_ota_update(host: OTAHost, fw_data: bytes, sig_data: bytes, expect_success=True):
    """
    Performs the OTA update process by calling the host's high-level method.
    This function now acts as a wrapper around `host.perform_full_ota` to integrate
    with the existing test structure and handle the success/failure logic.
    """
    if host.verbose:
        print(CYAN + f"  TEST_OTA_LOGIC: Attempting OTA: FW Size={len(fw_data)}, Sig Size={len(sig_data)}, Expect Success={expect_success}" + RESET)

    ota_result = await host.perform_full_ota(
        fw_data=fw_data,
        sig_data=sig_data,
        initial_ota_command="update",
        ota_mode_log="Entering OTA mode...",
        post_ota_ready_log="Bootloader ready. Waiting for command",
        reboot_timeout=25 
    )

    if expect_success:
        if ota_result:
            if host.verbose:
                print(GREEN + "  TEST_OTA_LOGIC: OTA sequence succeeded as expected." + RESET)
            return True
        else:
            print(RED + "  TEST_OTA_LOGIC: OTA sequence FAILED but was expected to succeed." + RESET)
            return False
    else:  # Expecting failure
        if not ota_result:
            if host.verbose:
                print(GREEN + "  TEST_OTA_LOGIC: OTA sequence FAILED as expected." + RESET)
            # After an expected failure, reboot the device to ensure it's in a clean state for the next test.
            print(YELLOW + "  TEST_OTA_LOGIC: Rebooting device to a clean state after expected failure..." + RESET)
            await reboot_and_wait_for_ready(host)
            return True # The test assertion passes because the OTA failed correctly.
        else:
            print(RED + "  TEST_OTA_LOGIC: OTA sequence SUCCEEDED but was expected to fail." + RESET)
            return False # The test assertion fails because the OTA didn't fail as expected.


# --- Test Cases ---

@pytest.mark.asyncio
async def test_ota_valid_firmware_multiple_sizes(ota_host_fixture: OTAHost):
    test_name = "test_ota_valid_firmware_multiple_sizes"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    valid_firmwares = [
        (VALID_FW_A_SLOTA, VALID_SIG_A_SLOTA, VALID_FW_A_SLOTB, VALID_SIG_A_SLOTB, "Firmware A"),
        (VALID_FW_B_SLOTA, VALID_SIG_B_SLOTA, VALID_FW_B_SLOTB, VALID_SIG_B_SLOTB, "Firmware B"),
        (VALID_FW_LARGE_SLOTA, VALID_SIG_LARGE_SLOTA, VALID_FW_LARGE_SLOTB, VALID_SIG_LARGE_SLOTB, "Firmware Large"),
    ]

    for i, (fw_path_a, sig_path_a, fw_path_b, sig_path_b, desc) in enumerate(valid_firmwares):
        print(PURPLE + f"\n--- Iteration {i+1}/{len(valid_firmwares)}: Testing Valid OTA: {desc} ---" + RESET)
        if host.verbose:
            print(YELLOW + f"  PRE-TEST ({desc}): Rebooting device to ensure fresh state for this iteration..." + RESET)
        assert await reboot_and_wait_for_ready(host), f"Device failed to reboot and become ready before testing {desc}"

        active_slot = await host.get_active_slot()
        assert active_slot in [0, 1], f"Could not determine active slot. Got: {active_slot}"

        if active_slot == 0:
            target_slot, fw_path, sig_path = 1, fw_path_b, sig_path_b
        else: # active_slot == 1
            target_slot, fw_path, sig_path = 0, fw_path_a, sig_path_a
        
        if host.verbose:
            print(CYAN + f"  Device reports active slot: {active_slot}. Targeting slot {target_slot} with '{os.path.basename(fw_path)}'." + RESET)

        assert os.path.exists(fw_path), f"Firmware file not found: {fw_path}"
        assert os.path.exists(sig_path), f"Signature file not found: {sig_path}"

        with open(fw_path, "rb") as f: fw_data = f.read()
        with open(sig_path, "rb") as f: sig_data = f.read()

        assert await perform_ota_update(host, fw_data, sig_data, expect_success=True), f"OTA failed for {desc}"
        print(GREEN + f"  ITERATION SUCCESS: Valid OTA for {desc} completed and device rebooted to ready state." + RESET)
    
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_corrupted_signature_data(ota_host_fixture: OTAHost):
    test_name = "test_ota_corrupted_signature_data"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    print(PURPLE + "\n--- Testing OTA with Corrupted Signature Data (bit-flipped) ---" + RESET)

    # Using Slot A firmware for this test. The failure mechanism is independent of the slot.
    assert os.path.exists(VALID_FW_A_SLOTA), f"Missing firmware file: {VALID_FW_A_SLOTA}"
    assert os.path.exists(VALID_SIG_A_SLOTA), f"Missing signature file: {VALID_SIG_A_SLOTA}"

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_A_SLOTA, "rb") as f: sig_data_list = list(f.read())

    if not sig_data_list:
        pytest.skip("Original signature data is empty, cannot corrupt.")
    
    idx_to_corrupt = len(sig_data_list) // 2
    sig_data_list[idx_to_corrupt] ^= 0xFF
    corrupted_sig_data = bytes(sig_data_list)

    if host.verbose:
        print(f"  Using firmware '{os.path.basename(VALID_FW_A_SLOTA)}' with corrupted signature.")
    
    assert await perform_ota_update(host, fw_data, corrupted_sig_data, expect_success=False), \
        "OTA with corrupted signature was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for corrupted signature data passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)

@pytest.mark.asyncio
async def test_ota_signature_from_different_key(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_from_different_key"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    print(PURPLE + "\n--- Testing OTA with Signature from Different Private Key ---" + RESET)

    if not os.path.exists(DIFFERENT_KEY_SIG):
        pytest.skip(f"Skipping test, missing signature file signed by a different key: {DIFFERENT_KEY_SIG}")
    
    # Using Slot A firmware for this test.
    assert os.path.exists(VALID_FW_A_SLOTA), f"Missing firmware file: {VALID_FW_A_SLOTA}"

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(DIFFERENT_KEY_SIG, "rb") as f: diff_key_sig_data = f.read()

    if host.verbose:
        print(f"  Using firmware '{os.path.basename(VALID_FW_A_SLOTA)}' with signature from a different private key.")
    
    assert await perform_ota_update(host, fw_data, diff_key_sig_data, expect_success=False), \
        "OTA with signature from different key was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for signature from different key passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_for_different_firmware(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_for_different_firmware"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    print(PURPLE + "\n--- Testing OTA with Signature for a Different Firmware Image (Wrong Hash) ---" + RESET)
    
    # Using Firmware A with Signature B. Using slota versions for simplicity.
    assert os.path.exists(VALID_FW_A_SLOTA), f"Missing firmware file: {VALID_FW_A_SLOTA}"
    assert os.path.exists(WRONG_HASH_SIG), f"Missing signature file for different firmware: {WRONG_HASH_SIG}"

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data_A = f.read()
    with open(WRONG_HASH_SIG, "rb") as f: sig_data_B = f.read()

    if host.verbose:
        print(f"  Using firmware '{os.path.basename(VALID_FW_A_SLOTA)}' with a signature from '{os.path.basename(WRONG_HASH_SIG)}'.")
    
    assert await perform_ota_update(host, fw_data_A, sig_data_B, expect_success=False), \
        "OTA with signature for different firmware was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for signature for different firmware passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_incorrect_length(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_incorrect_length"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    # Using Slot A files for this test.
    assert os.path.exists(VALID_FW_A_SLOTA), f"Missing firmware file: {VALID_FW_A_SLOTA}"
    assert os.path.exists(VALID_SIG_A_SLOTA), f"Missing signature file: {VALID_SIG_A_SLOTA}"

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_A_SLOTA, "rb") as f: original_sig_data = f.read()

    if not original_sig_data:
        pytest.skip("Original signature data is empty, cannot test length variations.")

    # --- Sub-test: OTA with too short signature ---
    print(PURPLE + "\n--- Sub-test: OTA with Too Short Signature ---" + RESET)
    assert await reboot_and_wait_for_ready(host), "Device failed to reboot and become ready before Short Sig sub-test"

    if len(original_sig_data) <= 10:
        print(YELLOW + f"  Skipping 'too short' signature test as original signature length ({len(original_sig_data)}) is too small." + RESET)
    else:
        short_sig_data = original_sig_data[:len(original_sig_data) // 2]
        if host.verbose:
            print(f"  Testing with too short signature (length {len(short_sig_data)}, original {len(original_sig_data)}).")
        assert await perform_ota_update(host, fw_data, short_sig_data, expect_success=False), \
            "OTA with too short signature was unexpectedly successful or did not fail as anticipated."
        print(GREEN + "  SUCCESS: Test for too short signature passed (OTA rejected as expected)." + RESET)

    # --- Sub-test: OTA with too long signature ---
    print(PURPLE + "\n--- Sub-test: OTA with Too Long Signature ---" + RESET)
    assert await reboot_and_wait_for_ready(host), "Device failed to reboot and become ready before Long Sig sub-test"

    long_sig_data = original_sig_data + b'\xDE\xAD\xBE\xEF' * 2
    if host.verbose:
        print(f"  Testing with too long signature (length {len(long_sig_data)}, original {len(original_sig_data)}).")
    assert await perform_ota_update(host, fw_data, long_sig_data, expect_success=False), \
        "OTA with too long signature was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "  SUCCESS: Test for too long signature passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_firmware_modified_after_signing(ota_host_fixture: OTAHost):
    test_name = "test_ota_firmware_modified_after_signing"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    print(PURPLE + "\n--- Testing OTA with Firmware Modified After Signing (1-bit flip) ---" + RESET)

    # Using Slot A files for this test.
    assert os.path.exists(VALID_FW_A_SLOTA), f"Missing firmware file: {VALID_FW_A_SLOTA}"
    assert os.path.exists(VALID_SIG_A_SLOTA), f"Missing signature file: {VALID_SIG_A_SLOTA}"

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data_list = list(f.read())
    with open(VALID_SIG_A_SLOTA, "rb") as f: original_sig_data = f.read()

    if not fw_data_list:
        pytest.skip("Original firmware is empty, cannot modify.")
    
    idx_to_modify = len(fw_data_list) // 2
    fw_data_list[idx_to_modify] ^= 0x01
    modified_fw_data = bytes(fw_data_list)

    if host.verbose:
        print(f"  Using modified firmware (byte {idx_to_modify} LSB flipped) with its original, unmodified signature.")
    
    assert await perform_ota_update(host, modified_fw_data, original_sig_data, expect_success=False), \
        "OTA with modified firmware was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for firmware modified after signing passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


# --- Pytest Module Setup ---
def setup_module(module):
    """
    Checks for the existence of essential firmware and signature files before running any tests.
    """
    print(YELLOW + BOLD + f"\n{'#'*20} RUNNING MODULE SETUP {'#'*20}" + RESET)
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
    missing_optional_files = []

    print(CYAN + "Checking for required test files..." + RESET)
    for desc, paths in required_files_info.items():
        is_optional = "optional" in desc.lower()
        for path in paths:
            if not os.path.exists(path):
                if is_optional:
                    print(YELLOW + f"  OPTIONAL FILE MISSING (related test will be skipped): {desc} - {path}" + RESET)
                    if path not in missing_optional_files:
                        missing_optional_files.append(path)
                else:
                    print(RED + f"  CRITICAL FILE MISSING: {desc} - {path}" + RESET)
                    all_critical_files_present = False
            else:
                print(GREEN + f"  PRESENT: {desc} - {path}" + RESET)

    if not all_critical_files_present:
        pytest.fail(RED + "One or more critical test files are missing. Please generate/place them and retry." + RESET, pytrace=False)
    
    if missing_optional_files:
        print(YELLOW + "Note: Some optional files are missing. Corresponding tests will be skipped." + RESET)
    
    if all_critical_files_present:
        print(GREEN + "All critical test files appear to be present." + RESET)
    print(YELLOW + BOLD + f"{'#'*20} MODULE SETUP COMPLETE {'#'*20}\n" + RESET)
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
    # DEFAULT_CHAR_RX_UUID, DEFAULT_CHAR_TX_UUID, DEFAULT_SERVICE_UUID # Kept for context, assuming they might be used elsewhere or good to know
)

# --- Test Configuration ---
DEVICE_NAME = "ESP32_OTA_BLE"
TEST_FW_DIR = "test_firmware"
VALID_FW_ORIGINAL_NAME = "firmware_A"
VALID_FW_SMALL_NAME = "firmware_small"
VALID_FW_LARGE_NAME = "firmware_large"

# Construct full paths for firmware and signature files
VALID_FW_ORIGINAL = os.path.join(TEST_FW_DIR, f"{VALID_FW_ORIGINAL_NAME}.bin")
VALID_SIG_ORIGINAL = os.path.join(TEST_FW_DIR, f"{VALID_FW_ORIGINAL_NAME}.sig")
VALID_FW_SMALL = os.path.join(TEST_FW_DIR, f"{VALID_FW_SMALL_NAME}.bin")
VALID_SIG_SMALL = os.path.join(TEST_FW_DIR, f"{VALID_FW_SMALL_NAME}.sig")
VALID_FW_LARGE = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}.bin")
VALID_SIG_LARGE = os.path.join(TEST_FW_DIR, f"{VALID_FW_LARGE_NAME}.sig")

# Paths for specific test case files
DIFFERENT_KEY_SIG = os.path.join(TEST_FW_DIR, f"{VALID_FW_ORIGINAL_NAME}_diff_key.sig")
WRONG_HASH_SIG = os.path.join(TEST_FW_DIR, "firmware_B.sig") # Signature for a different firmware

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
        # Proceeding to wait for boot messages despite potential send issue for reboot command,
        # as the device might still reboot or already be in the process.
        if host.verbose:
            print(YELLOW + "  HELPER: Proceeding to wait for boot messages." + RESET)

    if host.verbose:
        print(YELLOW + f"  HELPER: Waiting {reboot_cmd_delay}s for reboot to initiate..." + RESET)
    await asyncio.sleep(reboot_cmd_delay)

    # wait_for_log_message respects host.verbose for its own success/failure printing
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

# --- Pytest Fixture for OTAHost ---
@pytest_asyncio.fixture(scope="function")
async def ota_host_fixture(request): # 'request' is a built-in pytest fixture
    """
    Pytest fixture to set up and tear down the OTAHost connection for each test function.
    Initializes OTAHost, connects, performs an initial reboot, and handles disconnection.
    Verbosity is controlled by the --verbose flag passed to pytest or TEST_VERBOSE environment variable.
    """
    # Determine verbosity: prefer pytest's own verbosity flag, fallback to environment variable
    pytest_verbosity = request.config.getoption("verbose")
    if pytest_verbosity > 0:
        is_verbose = True
    else:
        is_verbose = os.getenv("TEST_VERBOSE", "False").lower() == "true"

    print(PURPLE + BOLD + f"\n{'='*20} SETUP FIXTURE: ota_host_fixture (Verbose: {is_verbose}) {'='*20}" + RESET)
    host = OTAHost(DEVICE_NAME, verbose=is_verbose)
    connected = await host.connect()
    if not connected:
        # Critical failure if connection cannot be established
        pytest.fail(RED + f"Fixture: Failed to connect to device '{DEVICE_NAME}' via OTAHost." + RESET)

    if host.verbose:
        print(YELLOW + "Fixture: Connected. Performing initial reboot to ensure known state..." + RESET)
    # Ensure device is in a known ready state before tests run
    # reboot_and_wait_for_ready respects host.verbose for its internal logging
    if not await reboot_and_wait_for_ready(host, reboot_cmd_delay=4.0, ready_timeout=20):
        pytest.fail(RED + "Fixture: Device failed to become ready after initial controlled reboot." + RESET)
    if host.verbose:
        print(GREEN + "Fixture: Device is initially rebooted and ready." + RESET)

    print(PURPLE + BOLD + f"{'='*20} SETUP FIXTURE COMPLETE {'='*20}\n" + RESET)
    yield host # Provide the initialized host to the test function

    # Teardown phase
    print(PURPLE + BOLD + f"\n{'='*20} TEARDOWN FIXTURE: ota_host_fixture {'='*20}" + RESET)
    if host.verbose:
        print(YELLOW + "\nFixture: Cleaning up OTAHost connection..." + RESET)
    # host.disconnect respects host.verbose for its internal logging
    await host.disconnect()
    if host.verbose:
        print(GREEN + "Fixture: OTAHost disconnected." + RESET)
    print(PURPLE + BOLD + f"{'='*20} TEARDOWN FIXTURE COMPLETE {'='*20}\n" + RESET)


# --- Helper Test OTA Sequence (Main OTA Logic) ---
async def perform_ota_update(host: OTAHost, fw_data: bytes, sig_data: bytes, expect_success=True):
    """
    Performs the core OTA update process.
    This function encapsulates the sequence of commands and data transfers for an OTA update.
    It checks for expected ACK/NACK responses at various stages.
    Verbosity of internal steps is handled by host.verbose.
    """
    await host.clear_device_logs()
    fw_size = len(fw_data)
    if host.verbose:
        print(CYAN + f"  TEST_OTA_LOGIC: Attempting OTA: FW Size={fw_size}, Sig Size={len(sig_data)}, Expect Success={expect_success}" + RESET)

    if host.verbose:
        print(CYAN + "  TEST_OTA_LOGIC: Sending 'ota' text command to enter OTA mode..." + RESET)
    await host.clear_device_logs()
    # host.send_text_command respects host.verbose
    if not await host.send_text_command("ota", expected_response_log="Entering OTA mode...", log_timeout=5):
        print(RED + "  TEST_OTA_LOGIC: Failed to enter OTA mode." + RESET)
        return False # Cannot proceed if OTA mode isn't entered

    if host.verbose:
        print(CYAN + "  TEST_OTA_LOGIC: Sending CMD_START..." + RESET)
    # host.send_cmd respects host.verbose
    if not await host.send_cmd(CMD_START):
        print(RED + "  TEST_OTA_LOGIC: CMD_START failed." + RESET)
        # If we expected success, this is a failure.
        # If we expected failure, ensure it was a NACK (or timeout, implied by last_status not being ACK).
        if expect_success: return False
        else:
            assert host.last_status == RESP_NACK if host.last_status is not None else True, \
                   f"Expected NACK or timeout for CMD_START in failed OTA. Got: {host.last_status}"
            return True # CMD_START NACKed as expected for a failing OTA scenario

    # Local import for clarity, as crc32 is specifically used here
    from ota_host import crc32 as calculate_crc32
    fw_crc_val = calculate_crc32(fw_data)
    if host.verbose:
        print(CYAN + f"  TEST_OTA_LOGIC: Sending Header (FW Size: {fw_size}, CRC: {fw_crc_val:#08x})..." + RESET)
    # host.send_header respects host.verbose
    if not await host.send_header(fw_size, fw_crc_val):
        print(RED + "  TEST_OTA_LOGIC: Sending Header failed." + RESET)
        if expect_success: return False
        else:
            assert host.last_status == RESP_NACK if host.last_status is not None else True, \
                   f"Expected NACK or timeout for Header in failed OTA. Got: {host.last_status}"
            return True # Header NACKed as expected

    if host.verbose:
        print(CYAN + "  TEST_OTA_LOGIC: Sending Firmware Data..." + RESET)
    # host.send_data_chunks respects host.verbose
    if not await host.send_data_chunks(fw_data):
        print(RED + "  TEST_OTA_LOGIC: Failed during Firmware Data transfer." + RESET)
        if expect_success: return False
        else:
            assert host.last_status == RESP_NACK if host.last_status is not None else True, \
                   f"Expected NACK or timeout for FW Data in failed OTA. Got: {host.last_status}"
            return True # FW Data NACKed as expected

    if host.verbose:
        print(CYAN + "  TEST_OTA_LOGIC: Sending Signature Data..." + RESET)
    # host.send_sig_chunks respects host.verbose
    sig_transfer_ok = await host.send_sig_chunks(sig_data)

    if not sig_transfer_ok:
        # send_sig_chunks would have printed specific errors if host.verbose is true
        if host.verbose or expect_success: # Print if verbose or if this failure is unexpected
            print(YELLOW + "  TEST_OTA_LOGIC: Signature chunk transfer failed." + RESET)
        if expect_success: return False
        else: # Expecting failure here (e.g., due to bad signature data causing NACK)
            assert host.last_status == RESP_NACK, \
                   f"Expected NACK during signature transfer for a failing OTA, got: {host.last_status}"
            if host.verbose:
                print(GREEN + "  TEST_OTA_LOGIC: NACK during signature (expected for this failure scenario)." + RESET)
            await host.send_cmd(CMD_END, ack_timeout=0.2) # Attempt to send CMD_END to gracefully terminate on device side
            return True # Signature transfer failed as expected

    if host.verbose:
        print(CYAN + "  TEST_OTA_LOGIC: Sending CMD_END..." + RESET)
    cmd_end_response_is_ack = await host.send_cmd(CMD_END)

    if expect_success:
        if cmd_end_response_is_ack and host.last_status == RESP_ACK:
            if host.verbose:
                print(GREEN + "  TEST_OTA_LOGIC: OTA sequence ACKed. Waiting for post-OTA reboot..." + RESET)
            # host.wait_for_log_message respects host.verbose for its output
            reboot_ready = await host.wait_for_log_message("Bootloader ready. Waiting for command", timeout=25, quiet_success=not host.verbose)
            if reboot_ready:
                if host.verbose:
                    print(GREEN + "  TEST_OTA_LOGIC: Device rebooted and signaled 'Bootloader ready'." + RESET)
                return True
            else:
                print(RED + "  TEST_OTA_LOGIC: Device did NOT signal 'Bootloader ready' post-OTA." + RESET)
                return False
        else:
            print(RED + f"  TEST_OTA_LOGIC: Expected CMD_END ACK for successful OTA, got ACK_FLAG: {cmd_end_response_is_ack}, Device_Status: {host.last_status}" + RESET)
            return False
    else: # Expecting overall failure (e.g. bad signature, bad firmware hash, etc.)
        if not cmd_end_response_is_ack and host.last_status == RESP_NACK:
            # NACK for CMD_END is expected if the device detected an issue (e.g., signature verification failed)
            if host.verbose:
                print(GREEN + "  TEST_OTA_LOGIC: NACK for CMD_END (expected for this failure scenario)." + RESET)
            return True
        else:
            print(RED + f"  TEST_OTA_LOGIC: Expected NACK on CMD_END for failed OTA, got ACK_FLAG: {cmd_end_response_is_ack}, Device_Status: {host.last_status}" + RESET)
            return False

# --- Test Cases ---
# Test functions utilize the ota_host_fixture for setup and teardown.
# High-level success/failure messages are printed by the tests themselves.
# Detailed step-by-step logging within helpers (like perform_ota_update) depends on host.verbose.

@pytest.mark.asyncio
async def test_ota_valid_firmware_multiple_sizes(ota_host_fixture: OTAHost):
    test_name = "test_ota_valid_firmware_multiple_sizes"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture # Fixture provides connected and ready host
    valid_firmwares = [
        (VALID_FW_ORIGINAL, VALID_SIG_ORIGINAL, "Original Size"),
        (VALID_FW_SMALL, VALID_SIG_SMALL, "Small Size"),
        (VALID_FW_LARGE, VALID_SIG_LARGE, "Large Size"),
    ]

    for i, (fw_path, sig_path, desc) in enumerate(valid_firmwares):
        print(PURPLE + f"\n--- Iteration {i+1}/{len(valid_firmwares)}: Testing Valid OTA: {desc} ({os.path.basename(fw_path)}) ---" + RESET)
        if host.verbose:
            print(YELLOW + f"  PRE-TEST ({desc}): Rebooting device to ensure fresh state for this iteration..." + RESET)
        # Ensure a clean state before each OTA attempt in the loop
        assert await reboot_and_wait_for_ready(host), f"Device failed to reboot and become ready before testing {desc}"
        if host.verbose:
            print(GREEN + f"  PRE-TEST ({desc}): Device is rebooted and ready." + RESET)

        assert os.path.exists(fw_path), f"Firmware file not found: {fw_path}"
        assert os.path.exists(sig_path), f"Signature file not found: {sig_path}"

        with open(fw_path, "rb") as f: fw_data = f.read()
        with open(sig_path, "rb") as f: sig_data = f.read()

        # Perform the OTA update, expecting it to succeed
        assert await perform_ota_update(host, fw_data, sig_data, expect_success=True), f"OTA failed for {desc}"
        print(GREEN + f"  ITERATION SUCCESS: Valid OTA for {desc} completed and device rebooted to ready state." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_corrupted_signature_data(ota_host_fixture: OTAHost):
    test_name = "test_ota_corrupted_signature_data"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    print(PURPLE + "\n--- Testing OTA with Corrupted Signature Data (bit-flipped) ---" + RESET)
    assert os.path.exists(VALID_FW_ORIGINAL), f"Missing firmware file: {VALID_FW_ORIGINAL}"
    assert os.path.exists(VALID_SIG_ORIGINAL), f"Missing signature file: {VALID_SIG_ORIGINAL}"

    with open(VALID_FW_ORIGINAL, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_ORIGINAL, "rb") as f: sig_data_list = list(f.read())

    if not sig_data_list: # Guard against empty signature file
        pytest.skip("Original signature data is empty, cannot corrupt.")
    
    idx_to_corrupt = len(sig_data_list) // 2 # Corrupt a byte in the middle
    sig_data_list[idx_to_corrupt] ^= 0xFF # Flip all bits of the chosen byte
    corrupted_sig_data = bytes(sig_data_list)

    if host.verbose:
        print(f"  Using original firmware with corrupted signature (byte {idx_to_corrupt} was bit-flipped).")
    
    # Perform OTA, expecting it to fail due to corrupted signature
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
    assert os.path.exists(VALID_FW_ORIGINAL), f"Missing firmware file: {VALID_FW_ORIGINAL}"

    with open(VALID_FW_ORIGINAL, "rb") as f: fw_data = f.read()
    with open(DIFFERENT_KEY_SIG, "rb") as f: diff_key_sig_data = f.read()

    if host.verbose:
        print(f"  Using firmware '{os.path.basename(VALID_FW_ORIGINAL)}' with signature from a different private key.")
    
    # Perform OTA, expecting it to fail
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
    assert os.path.exists(VALID_FW_ORIGINAL), f"Missing firmware file: {VALID_FW_ORIGINAL}"
    assert os.path.exists(WRONG_HASH_SIG), f"Missing signature file for different firmware: {WRONG_HASH_SIG}"

    with open(VALID_FW_ORIGINAL, "rb") as f: fw_data_A = f.read() # Firmware A
    with open(WRONG_HASH_SIG, "rb") as f: sig_data_B = f.read()   # Signature for Firmware B

    if host.verbose:
        print(f"  Using firmware '{os.path.basename(VALID_FW_ORIGINAL)}' with a signature intended for a different firmware.")
    
    # Perform OTA, expecting it to fail
    assert await perform_ota_update(host, fw_data_A, sig_data_B, expect_success=False), \
        "OTA with signature for different firmware was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for signature for different firmware passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_incorrect_length(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_incorrect_length"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    assert os.path.exists(VALID_FW_ORIGINAL), f"Missing firmware file: {VALID_FW_ORIGINAL}"
    assert os.path.exists(VALID_SIG_ORIGINAL), f"Missing signature file: {VALID_SIG_ORIGINAL}"

    with open(VALID_FW_ORIGINAL, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_ORIGINAL, "rb") as f: original_sig_data = f.read()

    if not original_sig_data:
        pytest.skip("Original signature data is empty, cannot test length variations.")

    # --- Sub-test: OTA with too short signature ---
    print(PURPLE + "\n--- Sub-test: OTA with Too Short Signature ---" + RESET)
    if host.verbose:
        print(YELLOW + "  PRE-SUB-TEST (Short Sig): Rebooting device..." + RESET)
    assert await reboot_and_wait_for_ready(host), "Device failed to reboot and become ready before Short Sig sub-test"
    if host.verbose:
        print(GREEN + "  PRE-SUB-TEST (Short Sig): Device is rebooted and ready." + RESET)

    # Define a significantly shorter length, but not zero if original is very small.
    short_len = len(original_sig_data) // 2
    if short_len == 0 and len(original_sig_data) > 1: # Ensure it's actually shorter
        short_len = 1 
    
    # Skip if original signature is too small to make a meaningful "short" version (e.g. <= 10 bytes)
    # This avoids issues if the "shortened" signature is still considered valid by the device for some reason,
    # or if shortening logic makes it empty.
    if len(original_sig_data) <= 10:
        print(YELLOW + f"  Skipping 'too short' signature test as original signature length ({len(original_sig_data)}) is too small." + RESET)
    else:
        short_sig_data = original_sig_data[:short_len]
        if host.verbose:
            print(f"  Testing with too short signature (length {len(short_sig_data)}, original {len(original_sig_data)}).")
        assert await perform_ota_update(host, fw_data, short_sig_data, expect_success=False), \
            "OTA with too short signature was unexpectedly successful or did not fail as anticipated."
        print(GREEN + "  SUCCESS: Test for too short signature passed (OTA rejected as expected)." + RESET)

    # --- Sub-test: OTA with too long signature ---
    print(PURPLE + "\n--- Sub-test: OTA with Too Long Signature ---" + RESET)
    if host.verbose:
        print(YELLOW + "  PRE-SUB-TEST (Long Sig): Rebooting device..." + RESET)
    assert await reboot_and_wait_for_ready(host), "Device failed to reboot and become ready before Long Sig sub-test"
    if host.verbose:
        print(GREEN + "  PRE-SUB-TEST (Long Sig): Device is rebooted and ready." + RESET)

    long_sig_data = original_sig_data + b'\xDE\xAD\xBE\xEF' * 2 # Append some arbitrary bytes
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
    assert os.path.exists(VALID_FW_ORIGINAL), f"Missing firmware file: {VALID_FW_ORIGINAL}"
    assert os.path.exists(VALID_SIG_ORIGINAL), f"Missing signature file: {VALID_SIG_ORIGINAL}"

    with open(VALID_FW_ORIGINAL, "rb") as f: fw_data_list = list(f.read()) # Read as list for easy modification
    with open(VALID_SIG_ORIGINAL, "rb") as f: original_sig_data = f.read()

    if not fw_data_list: # Guard against empty firmware
        pytest.skip("Original firmware is empty, cannot modify.")
    
    idx_to_modify = len(fw_data_list) // 2 # Modify a byte in the middle
    fw_data_list[idx_to_modify] ^= 0x01   # Flip the least significant bit
    modified_fw_data = bytes(fw_data_list)

    if host.verbose:
        print(f"  Using modified firmware (byte {idx_to_modify} LSB flipped) with its original, unmodified signature.")
    
    # Perform OTA, expecting failure because firmware hash won't match signature
    assert await perform_ota_update(host, modified_fw_data, original_sig_data, expect_success=False), \
        "OTA with modified firmware was unexpectedly successful or did not fail as anticipated."
    print(GREEN + "SUCCESS: Test for firmware modified after signing passed (OTA rejected as expected)." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


# --- Pytest Module Setup (runs once before all tests in this file) ---
def setup_module(module):
    """
    Checks for the existence of essential firmware and signature files before running any tests.
    If critical files are missing, tests will be halted.
    Optional files will result in skipped tests if missing.
    """
    print(YELLOW + BOLD + f"\n{'#'*20} RUNNING MODULE SETUP {'#'*20}" + RESET)
    print(YELLOW + f"Test firmware directory: '{TEST_FW_DIR}'" + RESET)
    os.makedirs(TEST_FW_DIR, exist_ok=True) # Ensure the test firmware directory exists

    # Define required files for the tests.
    # Structure: {description: [list_of_paths], ...}
    # Differentiate between critical files (must exist) and optional (test skipped if missing).
    required_files_info = {
        "Valid Firmware (Original)": [VALID_FW_ORIGINAL, VALID_SIG_ORIGINAL],
        "Valid Firmware (Small)": [VALID_FW_SMALL, VALID_SIG_SMALL],
        "Valid Firmware (Large)": [VALID_FW_LARGE, VALID_SIG_LARGE],
        "Untrusted Key Signature (Optional)": [DIFFERENT_KEY_SIG], # For test_ota_signature_from_different_key
        "Wrong Hash Signature (Critical)": [WRONG_HASH_SIG]      # For test_ota_signature_for_different_firmware
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
        pytest.fail(RED + "One or more critical test files are missing. Please generate/place them and retry. Halting tests." + RESET, pytrace=False)
    
    if missing_optional_files:
        print(YELLOW + "Note: Some optional files are missing. Corresponding tests will be skipped." + RESET)
    
    if all_critical_files_present:
        print(GREEN + "All critical test files appear to be present." + RESET)
    print(YELLOW + BOLD + f"{'#'*20} MODULE SETUP COMPLETE {'#'*20}\n" + RESET)
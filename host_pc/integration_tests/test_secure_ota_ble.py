import pytest
import asyncio
import os

from ota_host import OTAHost

# Import all constants, helpers, and fixtures from the central conftest.py
from conftest import (
    reboot_and_wait_for_ready,
    BLUE, PURPLE, CYAN, RED, GREEN, YELLOW, RESET, BOLD,
    VALID_FW_A_SLOTA, VALID_SIG_A_SLOTA, VALID_FW_A_SLOTB, VALID_SIG_A_SLOTB,
    VALID_FW_B_SLOTA, VALID_SIG_B_SLOTA, VALID_FW_B_SLOTB, VALID_SIG_B_SLOTB,
    VALID_FW_LARGE_SLOTA, VALID_SIG_LARGE_SLOTA, VALID_FW_LARGE_SLOTB, VALID_SIG_LARGE_SLOTB,
    DIFFERENT_KEY_SIG, WRONG_HASH_SIG
)


async def perform_ota_update(host: OTAHost, fw_data: bytes, sig_data: bytes, expect_success=True) -> bool:
    """Helper to run the OTA process and assert the outcome matches expectations."""
    if host.verbose:
        print(CYAN + f"  TEST_LOGIC: Attempting OTA: Expect Success={expect_success}" + RESET)

    ota_result_is_success = await host.perform_full_ota(fw_data=fw_data, sig_data=sig_data)

    if expect_success:
        assert ota_result_is_success, "OTA sequence FAILED but was expected to SUCCEED."
        if host.verbose:
            print(GREEN + "  TEST_LOGIC: OTA sequence SUCCEEDED as expected." + RESET)
    else:  # Expecting failure
        assert not ota_result_is_success, "OTA sequence SUCCEEDED but was expected to FAIL."
        if host.verbose:
            print(GREEN + "  TEST_LOGIC: OTA sequence FAILED as expected." + RESET)
        
        print(YELLOW + "  TEST_LOGIC: Rebooting device to a clean state after expected failure..." + RESET)
        assert await reboot_and_wait_for_ready(host), "Device failed to reboot after a controlled failure."

    return True # If asserts pass, the helper's job is done.


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
        assert await reboot_and_wait_for_ready(host), f"Device failed to reboot before testing {desc}"

        active_slot = await host.get_active_slot()
        assert active_slot in [0, 1], f"Could not determine active slot. Got: {active_slot}"

        fw_path, sig_path = (fw_path_b, sig_path_b) if active_slot == 0 else (fw_path_a, sig_path_a)
        
        with open(fw_path, "rb") as f: fw_data = f.read()
        with open(sig_path, "rb") as f: sig_data = f.read()

        assert await perform_ota_update(host, fw_data, sig_data, expect_success=True), f"OTA failed for {desc}"
        print(GREEN + f"  ITERATION SUCCESS: Valid OTA for {desc} completed." + RESET)
    
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_corrupted_signature_data(ota_host_fixture: OTAHost):
    test_name = "test_ota_corrupted_signature_data"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_A_SLOTA, "rb") as f: sig_data_list = list(f.read())
    
    idx_to_corrupt = len(sig_data_list) // 2
    sig_data_list[idx_to_corrupt] ^= 0xFF
    corrupted_sig_data = bytes(sig_data_list)
    
    assert await perform_ota_update(host, fw_data, corrupted_sig_data, expect_success=False)
    print(GREEN + "SUCCESS: Test for corrupted signature data passed." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_from_different_key(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_from_different_key"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    if not os.path.exists(DIFFERENT_KEY_SIG):
        pytest.skip(f"Missing optional file: {DIFFERENT_KEY_SIG}")
    
    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(DIFFERENT_KEY_SIG, "rb") as f: diff_key_sig_data = f.read()
    
    assert await perform_ota_update(host, fw_data, diff_key_sig_data, expect_success=False)
    print(GREEN + "SUCCESS: Test for signature from different key passed." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_for_different_firmware(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_for_different_firmware"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    
    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data_A = f.read()
    with open(WRONG_HASH_SIG, "rb") as f: sig_data_B = f.read()
    
    assert await perform_ota_update(host, fw_data_A, sig_data_B, expect_success=False)
    print(GREEN + "SUCCESS: Test for signature for different firmware passed." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_signature_incorrect_length(ota_host_fixture: OTAHost):
    test_name = "test_ota_signature_incorrect_length"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data = f.read()
    with open(VALID_SIG_A_SLOTA, "rb") as f: original_sig_data = f.read()

    # Sub-test: Too short
    print(PURPLE + "\n--- Sub-test: OTA with Too Short Signature ---" + RESET)
    short_sig_data = original_sig_data[:len(original_sig_data) // 2]
    assert await perform_ota_update(host, fw_data, short_sig_data, expect_success=False)

    # Sub-test: Too long
    print(PURPLE + "\n--- Sub-test: OTA with Too Long Signature ---" + RESET)
    long_sig_data = original_sig_data + b'\xDE\xAD\xBE\xEF'
    assert await perform_ota_update(host, fw_data, long_sig_data, expect_success=False)
    
    print(GREEN + "SUCCESS: Tests for incorrect signature length passed." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)


@pytest.mark.asyncio
async def test_ota_firmware_modified_after_signing(ota_host_fixture: OTAHost):
    test_name = "test_ota_firmware_modified_after_signing"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture

    with open(VALID_FW_A_SLOTA, "rb") as f: fw_data_list = list(f.read())
    with open(VALID_SIG_A_SLOTA, "rb") as f: original_sig_data = f.read()
    
    idx_to_modify = len(fw_data_list) // 2
    fw_data_list[idx_to_modify] ^= 0x01
    modified_fw_data = bytes(fw_data_list)
    
    assert await perform_ota_update(host, modified_fw_data, original_sig_data, expect_success=False)
    print(GREEN + "SUCCESS: Test for firmware modified after signing passed." + RESET)
    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)
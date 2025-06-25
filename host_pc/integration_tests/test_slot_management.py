import pytest
import os
import struct

# Helper functions and fixtures are now automatically imported from conftest.py
from conftest import (
    reboot_and_wait_for_ready,
    perform_simple_ota,
    get_bootloader_info,
    get_slot_memory_words,
    BLUE, PURPLE, CYAN, RED, GREEN, YELLOW, RESET, BOLD, # Import colors if needed for test-specific prints
    VALID_FW_B_SLOTA, VALID_SIG_B_SLOTA,
    VALID_FW_B_SLOTB, VALID_SIG_B_SLOTB,
)
from ota_host import OTAHost # Still need the type hint

@pytest.mark.asyncio
async def test_manual_slot_activation_and_persistence(ota_host_fixture: OTAHost):
    """
    Test 1: Verifies 'activate' command changes the active slot and it persists on reboot.
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
    Test 2: Verifies erasing an inactive slot marks it as invalid without affecting the active slot.
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
    Test 3: Verifies the bootloader prevents erasing the currently active slot.
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


@pytest.mark.asyncio
async def test_memory_content_after_erase_and_ota(ota_host_fixture: OTAHost):
    """
    Test 4: Uses 'print' to verify memory after 'erase' and a subsequent OTA.
    """
    test_name = "test_memory_content_after_erase_and_ota"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    
    active_slot = await host.get_active_slot()
    assert active_slot in [0, 1], f"Could not determine active slot, got: {active_slot}"
    slot_to_test = 1 if active_slot == 0 else 0
    
    print(f"Active slot is {active_slot}. Erasing inactive slot {slot_to_test} to check memory content...")
    
    assert await host.send_text_command(f"erase {slot_to_test}", "Boot config updated to mark slot as invalid"), \
        "Erase command did not complete successfully."
    
    print(f"Printing memory of slot {slot_to_test} to verify it was erased...")
    erased_words = await get_slot_memory_words(host, slot_to_test)
    assert erased_words is not None, "Failed to get memory words for erased slot."
    
    expected_erased_word = 0xFFFFFFFF
    assert all(word == expected_erased_word for word in erased_words), \
        f"Memory not fully erased. Expected all 0x{expected_erased_word:X}, got {erased_words}"
    print(GREEN + "SUCCESS: Memory is confirmed to be erased (0xFFFFFFFF)." + RESET)

    print(f"Performing OTA to slot {slot_to_test}...")
    fw_path = VALID_FW_B_SLOTB if slot_to_test == 1 else VALID_FW_B_SLOTA
    sig_path = VALID_SIG_B_SLOTB if slot_to_test == 1 else VALID_SIG_B_SLOTA
    assert await perform_simple_ota(host, fw_path, sig_path), "OTA to erased slot failed."
    
    print(f"Printing memory of slot {slot_to_test} again to verify written content...")
    written_words = await get_slot_memory_words(host, slot_to_test)
    assert written_words is not None, "Failed to get memory words for written slot."
    
    with open(fw_path, "rb") as f:
        firmware_bytes = f.read(40)
    
    expected_words = list(struct.unpack('<10I', firmware_bytes))

    assert written_words == expected_words, f"Mismatch between written memory and firmware file.\nExpected: {expected_words}\nGot: {written_words}"
    print(GREEN + "SUCCESS: Memory content correctly matches the flashed firmware." + RESET)

    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)

@pytest.mark.asyncio
async def test_reflashing_valid_inactive_slot(ota_host_fixture: OTAHost):
    """
    Test 5: Verifies re-flashing an inactive slot that already contains valid firmware.
    """
    test_name = "test_reflashing_valid_inactive_slot"
    print(BLUE + BOLD + f"\n{'~'*10} STARTING TEST: {test_name} {'~'*10}" + RESET)
    host = ota_host_fixture
    
    # After the initial OTA in the fixture, Slot 0 should be active.
    # To make this robust, we check and then target the inactive slot.
    active_slot = await host.get_active_slot()
    assert active_slot in [0, 1], f"Could not determine active slot, got: {active_slot}"
    slot_to_reflash = 1 if active_slot == 0 else 0
    
    # We need to perform one more OTA to ensure the inactive slot is valid before we test re-flashing it.
    # The fixture ensures a clean boot, but this test requires a valid inactive image.
    print("Performing one OTA to ensure inactive slot has a valid image...")
    assert await perform_simple_ota(host, "path/to/initial/firmware.bin", "path/to/initial/firmware.sig") # Replace with a known starting FW
    active_slot = await host.get_active_slot() # Re-check active slot
    slot_to_reflash = 1 if active_slot == 0 else 0


    print(f"Active slot is {active_slot}. Getting initial state of inactive slot {slot_to_reflash}...")
    initial_info = await get_bootloader_info(host)
    slot_key = f"slot_{'b' if slot_to_reflash == 1 else 'a'}"
    assert initial_info and slot_key in initial_info, f"Could not get initial info for Slot {slot_to_reflash}."
    initial_crc = initial_info[slot_key].get('fw_crc')
    print(f"Initial CRC for slot {slot_to_reflash} is {initial_crc}.")

    print(f"Re-flashing slot {slot_to_reflash} with new firmware (Firmware B)...")
    fw_path = VALID_FW_B_SLOTB if slot_to_reflash == 1 else VALID_FW_B_SLOTA
    sig_path = VALID_SIG_B_SLOTB if slot_to_reflash == 1 else VALID_SIG_B_SLOTA
    
    from ota_host import crc32 # crc32 is in ota_host, not conftest
    with open(fw_path, "rb") as f:
        expected_crc = crc32(f.read())
    print(f"Expected final CRC from '{os.path.basename(fw_path)}' is 0x{expected_crc:X}")
    
    assert await perform_simple_ota(host, fw_path, sig_path), "Re-flash OTA failed."
    
    final_active_slot = await host.get_active_slot()
    assert final_active_slot == slot_to_reflash, f"Device did not switch to new firmware. Active slot is {final_active_slot}"
    
    print("Getting final state to verify metadata was updated...")
    final_info = await get_bootloader_info(host)
    assert final_info and slot_key in final_info, f"Could not get final info for Slot {slot_to_reflash}."
    
    final_crc_str = final_info[slot_key].get('fw_crc')
    assert final_crc_str != initial_crc, \
        f"Firmware CRC did not change after re-flashing. Your '{os.path.basename(fw_path)}' may be identical to the original firmware."

    final_crc_int = int(final_crc_str, 16)
    assert final_crc_int == expected_crc, \
        f"Final CRC 0x{final_crc_int:X} does not match expected CRC 0x{expected_crc:X}"
        
    print(GREEN + "SUCCESS: Inactive slot was successfully re-flashed, and metadata was updated correctly." + RESET)

    print(BLUE + BOLD + f"~~~~~ FINISHED TEST: {test_name} ~~~~~" + RESET)
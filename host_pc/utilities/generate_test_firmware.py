#!/usr/bin/env python3

"""
Generates and signs firmware binaries for secure bootloader integration tests.

This script performs the following actions:
1.  Takes slot-specific firmware binaries as input (e.g., blinky_slota.bin).
2.  Copies and renames them to the test format (e.g., firmware_A_slota.bin).
3.  Creates a "large" version of one firmware by padding it to 64KB.
4.  Signs all generated firmware binaries using a primary private key.
5.  Creates one additional signature for a firmware binary using a secondary
    ("bad") private key to test signature verification failures.

The output files are placed in a specified directory, ready for use in
pytest integration tests.
"""

import argparse
import os
import shutil
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec, utils
from cryptography.hazmat.primitives import serialization

FIRMWARE_LARGE_SIZE_BYTES = 64 * 1024 # 64KB

def sign_file(private_key_path, file_path, signature_path):
    """Signs a file with the given private key and saves the signature."""
    # Load the private key
    with open(private_key_path, 'rb') as f:
        private_key = serialization.load_pem_private_key(f.read(), password=None)

    # Read the file to be signed
    with open(file_path, 'rb') as f:
        file_data = f.read()

    # Sign the data
    signature = private_key.sign(
        file_data,
        ec.ECDSA(hashes.SHA256())
    )

    # Save the signature
    with open(signature_path, 'wb') as f:
        f.write(signature)
    print(f"  - Signed {os.path.basename(file_path)} -> {os.path.basename(signature_path)}")

def prepare_firmware_files(input_dir, output_dir, key_dir):
    """
    Generates and signs all necessary firmware files for testing.

    Args:
        input_dir (str): Directory containing the base firmware binaries.
        output_dir (str): Directory to save the generated .bin and .sig files.
        key_dir (str): Directory containing the signing keys.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # Define paths for keys
    priv_key_path = os.path.join(key_dir, 'ec_priv.pem')
    priv_key_bad_path = os.path.join(key_dir, 'ec_priv_bad.pem')

    if not os.path.exists(priv_key_path) or not os.path.exists(priv_key_bad_path):
        print(f"Error: Signing keys not found in '{key_dir}'.")
        print("Please run the key generation script first.")
        return

    # --- Define input and output file mappings ---
    # Define the expected input files
    input_files = {
        "fw_a_slota": os.path.join(input_dir, 'blinky_slota.bin'),
        "fw_a_slotb": os.path.join(input_dir, 'blinky_slotb.bin'),
        "fw_b_slota": os.path.join(input_dir, 'uart_print_slota.bin'),
        "fw_b_slotb": os.path.join(input_dir, 'uart_print_slotb.bin'),
    }

    # Check if all source files exist
    for name, path in input_files.items():
        if not os.path.exists(path):
            print(f"Error: Could not find '{os.path.basename(path)}' in '{input_dir}'.")
            print("Please provide the base firmware files.")
            return

    print("--- Preparing Firmware A ---")
    # Copy firmware A for slot A and B
    fw_a_slota_bin = os.path.join(output_dir, 'firmware_A_slota.bin')
    fw_a_slotb_bin = os.path.join(output_dir, 'firmware_A_slotb.bin')
    shutil.copy(input_files["fw_a_slota"], fw_a_slota_bin)
    shutil.copy(input_files["fw_a_slotb"], fw_a_slotb_bin)
    print(f"Copied source binaries to firmware_A_slota.bin and firmware_A_slotb.bin")

    # Sign firmware A versions with the good key
    sign_file(priv_key_path, fw_a_slota_bin, fw_a_slota_bin.replace('.bin', '.sig'))
    sign_file(priv_key_path, fw_a_slotb_bin, fw_a_slotb_bin.replace('.bin', '.sig'))

    # Sign firmware A for slot A with the bad key
    bad_sig_path = os.path.join(output_dir, 'firmware_A_slota_diff_key.sig')
    sign_file(priv_key_bad_path, fw_a_slota_bin, bad_sig_path)

    print("\n--- Preparing Firmware B ---")
    # Copy firmware B for slot A and B
    fw_b_slota_bin = os.path.join(output_dir, 'firmware_B_slota.bin')
    fw_b_slotb_bin = os.path.join(output_dir, 'firmware_B_slotb.bin')
    shutil.copy(input_files["fw_b_slota"], fw_b_slota_bin)
    shutil.copy(input_files["fw_b_slotb"], fw_b_slotb_bin)
    print(f"Copied source binaries to firmware_B_slota.bin and firmware_B_slotb.bin")

    # Sign firmware B versions with the good key
    sign_file(priv_key_path, fw_b_slota_bin, fw_b_slota_bin.replace('.bin', '.sig'))
    sign_file(priv_key_path, fw_b_slotb_bin, fw_b_slotb_bin.replace('.bin', '.sig'))


    print("\n--- Preparing Large Firmware ---")
    # Create large firmware from firmware A (slota)
    fw_large_slota_bin = os.path.join(output_dir, 'firmware_large_slota.bin')
    fw_large_slotb_bin = os.path.join(output_dir, 'firmware_large_slotb.bin')
    
    # Use blinky_slota.bin as the base for both large firmwares
    large_fw_base_path = input_files["fw_a_slota"]

    for dest_path in [fw_large_slota_bin, fw_large_slotb_bin]:
        shutil.copy(large_fw_base_path, dest_path)
        with open(dest_path, 'ab') as f:
            current_size = f.tell()
            if current_size < FIRMWARE_LARGE_SIZE_BYTES:
                padding_size = FIRMWARE_LARGE_SIZE_BYTES - current_size
                f.write(b'\xFF' * padding_size)
        print(f"Created and padded {os.path.basename(dest_path)} to {FIRMWARE_LARGE_SIZE_BYTES} bytes")


    # Sign the large firmware versions with the good key
    sign_file(priv_key_path, fw_large_slota_bin, fw_large_slota_bin.replace('.bin', '.sig'))
    sign_file(priv_key_path, fw_large_slotb_bin, fw_large_slotb_bin.replace('.bin', '.sig'))

    print("\nTest firmware generation complete.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate and sign test firmware files.")
    parser.add_argument(
        '--input-dir',
        default='valid_firmware',
        help='Directory containing base firmware binaries (e.g., blinky_slota.bin)'
    )
    parser.add_argument(
        '--output-dir',
        default='test_data',
        help='Directory to save the generated test files'
    )
    parser.add_argument(
        '--key-dir',
        default='test_data',
        help='Directory containing the signing keys'
    )
    args = parser.parse_args()

    # Ensure the script is run from a location where it can find the input directory
    # A more robust solution might use absolute paths or better path logic.
    script_dir = os.path.dirname(os.path.realpath(__file__))
    input_path = os.path.join(script_dir, args.input_dir)
    output_path = os.path.join(script_dir, args.output_dir)
    key_path = os.path.join(script_dir, args.key_dir)

    prepare_firmware_files(input_path, output_path, key_path)

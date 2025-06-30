import argparse
import os
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization

def sign_file(private_key_path, file_path, signature_path):
    """
    Signs a file with the given private key and saves the signature.

    Args:
        private_key_path (str): The path to the PEM-encoded private key file.
        file_path (str): The path to the binary file to be signed.
        signature_path (str): The path where the output signature file will be saved.
    """
    try:
        # Load the private key from the specified file
        with open(private_key_path, 'rb') as f:
            private_key = serialization.load_pem_private_key(f.read(), password=None)

        # Read the binary data from the file to be signed
        with open(file_path, 'rb') as f:
            file_data = f.read()

        # Sign the data using ECDSA with a SHA256 hash
        signature = private_key.sign(
            file_data,
            ec.ECDSA(hashes.SHA256())
        )

        # Write the resulting signature to the output file
        with open(signature_path, 'wb') as f:
            f.write(signature)
            
        print(f"  - Signed: {os.path.basename(file_path)} -> {os.path.basename(signature_path)}")

    except FileNotFoundError:
        print(f"Error: Could not find file at '{file_path}' or key at '{private_key_path}'.")
    except Exception as e:
        print(f"An error occurred while signing {os.path.basename(file_path)}: {e}")

def sign_firmware_in_directory(firmware_dir, key_path):
    """
    Finds all .bin files in a directory and generates signatures for them.

    Args:
        firmware_dir (str): The directory containing firmware .bin files.
        key_path (str): The path to the private key for signing.
    """
    print(f"--- Searching for firmware in '{firmware_dir}' ---")

    # Check if the private key exists before starting
    if not os.path.exists(key_path):
        print(f"Error: Private key not found at '{key_path}'.")
        print("Please ensure the key file exists and the path is correct.")
        return

    # Check if the firmware directory exists
    if not os.path.isdir(firmware_dir):
        print(f"Error: Firmware directory not found at '{firmware_dir}'.")
        return

    # Iterate over all files in the specified directory
    files_found = False
    for filename in os.listdir(firmware_dir):
        # Check for files with the .bin extension
        if filename.endswith('.bin'):
            files_found = True
            file_path = os.path.join(firmware_dir, filename)
            # Define the output signature path
            signature_path = file_path.replace('.bin', '.sig')
            
            # Sign the file
            sign_file(key_path, file_path, signature_path)

    if not files_found:
        print("No .bin files found to sign.")
    
    print("\n--- Firmware signing process complete. ---")


if __name__ == '__main__':
    # Set up the command-line argument parser
    parser = argparse.ArgumentParser(description="Generate signatures for firmware binaries.")
    parser.add_argument(
        '--firmware-dir',
        default='firmware',
        help='Directory containing the firmware .bin files to sign.'
    )
    parser.add_argument(
        '--key',
        default='firmware/ec_priv.pem',
        help='Path to the private key file (e.g., ec_priv.pem).'
    )
    args = parser.parse_args()

    # Get the directory where the script is located to build absolute paths
    script_dir = os.path.dirname(os.path.realpath(__file__))
    firmware_path = os.path.join(script_dir, args.firmware_dir)
    key_file_path = os.path.join(script_dir, args.key)

    # Run the main signing function
    sign_firmware_in_directory(firmware_path, key_file_path)
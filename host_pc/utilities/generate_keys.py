#!/usr/bin/env python3

"""
Generates ECDSA key pairs for secure bootloader testing.

This script creates two sets of keys:
1.  A primary key pair (ec_priv.pem, ec_pub.pem, ec_pub.der) used for
    standard firmware signing.
2.  A secondary "bad" key pair (ec_priv_bad.pem) used to generate
    signatures that should fail verification, for testing purposes.

The keys are saved in the specified output directory.
"""

import argparse
import os
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization

def generate_and_save_keys(output_dir):
    """
    Generates and saves ECDSA key pairs.

    Args:
        output_dir (str): The directory to save the key files.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # --- Generate the primary ("good") key pair ---
    print("Generating primary ECDSA key pair...")
    private_key = ec.generate_private_key(ec.SECP256R1())
    public_key = private_key.public_key()

    # Save the private key in PEM format
    priv_pem_path = os.path.join(output_dir, 'ec_priv.pem')
    with open(priv_pem_path, 'wb') as f:
        f.write(private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption()
        ))
    print(f"  - Saved private key to {priv_pem_path}")

    # Save the public key in PEM format
    pub_pem_path = os.path.join(output_dir, 'ec_pub.pem')
    with open(pub_pem_path, 'wb') as f:
        f.write(public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
    print(f"  - Saved public key to {pub_pem_path}")

    # Save the public key in DER format
    pub_der_path = os.path.join(output_dir, 'ec_pub.der')
    with open(pub_der_path, 'wb') as f:
        f.write(public_key.public_bytes(
            encoding=serialization.Encoding.DER,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
    print(f"  - Saved public key to {pub_der_path}")


    # --- Generate the secondary ("bad") key pair ---
    print("\nGenerating secondary ('bad') ECDSA private key...")
    private_key_bad = ec.generate_private_key(ec.SECP256R1())

    # Save the "bad" private key in PEM format
    priv_bad_pem_path = os.path.join(output_dir, 'ec_priv_bad.pem')
    with open(priv_bad_pem_path, 'wb') as f:
        f.write(private_key_bad.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption()
        ))
    print(f"  - Saved 'bad' private key to {priv_bad_pem_path}")

    print("\nKey generation complete.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate ECDSA keys for bootloader testing.")
    parser.add_argument(
        '--output-dir',
        default='test_data',
        help='Directory to save the generated key files (default: test_data)'
    )
    args = parser.parse_args()
    generate_and_save_keys(args.output_dir)

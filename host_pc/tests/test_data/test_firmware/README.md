# Notes
- For now I am using Elliptical Curve Digital Signatures (ECDSA) for firmware signature verification.
- The secure-bootloader project will use mbed TLS to: 
    1. Calculate the hash SHA-256 of the firmware binary (blinky.bin)
    2. Use the embedded public key and the provided signature (blinky.sig) to verify the integrity and authenticity of the firmware. 
- This means we will want to configure mbedTLS with modules for ECDSA, public key parsing (DER), hashing (SHA-256), and the underlying elliptic curve and big number arithmetic. we will not need AES or other symmetric ciphers for now. 

# Command
openssl dgst -sha256 -sign ec_priv.pem -out firmware_A.sig firmware_A.bin

# Files
- ec_priv.pem: private key, used offline to sign the firmware (e.g. blinky.bin to produce blinky.sig)
- ec_pub.pem/ec_pub.der: public key, corresponding to the private key. This will be embedded into the secure-bootloader project


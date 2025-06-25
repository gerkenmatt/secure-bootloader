How to Use the Scripts
Project Structure: 

secure-bootloader/
└── host_pc/
    └── utilities/
        ├── generate_keys.py
        ├── generate_test_firmware.py
        ├── valid_firmware/
        │   ├── blinky.bin
        │   └── uart_print.bin
        └── test_data/
            ├── (files will be generated here)


python generate_keys.py --output-dir test_data
This will create the ec_*.pem, ec_*.der, and ec_*_bad.pem files inside the test_data directory.

Generate Firmware: Make sure your blinky.bin and uart_print.bin are inside the valid_firmware directory. Then, run the firmware generation script:


python generate_test_firmware.py --input-dir valid_firmware --output-dir ../test_data --key-dir ../test_data
This will create all the firmware_*.bin and firmware_*.sig files in the test_data directory, ready for your pytest suite.
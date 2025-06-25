# Building mbed TLS with CMake for STM32 Projects

This guide outlines how to build the mbed TLS library as a static library using CMake. This static library can then be linked into your STM32 microcontroller project. This method is recommended for modularity and easier updates.

## Prerequisites

1.  **mbed TLS Source Code:** You should have a local copy of the mbed TLS source code, preferably a stable release version.
2.  **Custom `mbedtls_config.h`:** Your tailored `mbedtls_config.h` file **must** be located at `include/mbedtls/mbedtls_config.h` within your mbed TLS source tree. The CMake build process will pick it up from there by default.
3.  **CMake:** CMake must be installed on your development machine. You can download it from [cmake.org](https://cmake.org/download/).
4.  **ARM Cross-Compiler Toolchain:** An ARM cross-compiler toolchain (e.g., GNU Arm Embedded Toolchain, `arm-none-eabi-gcc`) must be installed and ideally in your system's PATH.

## Steps to Compile mbed TLS

1.  **Navigate to mbed TLS Root Directory:**
    Open your terminal or command prompt and change to the root directory of your mbed TLS source code:
    ```bash
    cd /path/to/your/mbedtls_source_code
    ```

2.  **Create a Build Directory:**
    It's good practice to perform an out-of-source build. Create a build directory and navigate into it:
    ```bash
    mkdir build
    cd build
    ```

3.  **Configure the Build with CMake:**
    Run CMake from the `build` directory to generate the build files. You'll need to specify your ARM toolchain.
        ```bash
        cmake -DCMAKE_TOOLCHAIN_FILE=../../../cmake/arm-none-eabi.cmake  \
          -DENABLE_TESTING=OFF \
          -DENABLE_PROGRAMS=OFF \
          -DCMAKE_BUILD_TYPE=MinSizeRel \
          ../
      ..
        ```

4.  **Compile the Library:**
    After CMake successfully configures the project (no errors in the output), run your build tool. If using Makefiles (default on Linux/macOS):
    ```bash
    make
    ```
    If you are using a different generator (like Ninja), use the appropriate command (e.g., `ninja`). You can also increase parallelism:
    ```bash
    make -j4 # Example: build using 4 parallel jobs
    ```

5.  **Locate the Static Libraries:**
    Once the compilation is complete, the static libraries will be located in the `library/` subdirectory within your `build` directory.
    For example:
    * `build/library/libmbedcrypto.a` (This is the primary library you'll need for cryptographic functions like ECDSA).
    * `build/library/libmbedx509.a` (For X.509 parsing, should be very small if X.509 features are disabled in `mbedtls_config.h`).
    * `build/library/libmbedtls.a` (For TLS/SSL features, should be very small if SSL/TLS features are disabled).

    Given your configuration for ECDSA verification, `libmbedcrypto.a` will contain the bulk of the necessary code.

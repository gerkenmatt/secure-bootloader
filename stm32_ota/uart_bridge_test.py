import serial
import threading

ESP32_PORT = "/dev/ttyUSB0"
STM32_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

def read_from_stm32(ser_stm32):
    while True:
        try:
            if ser_stm32.in_waiting:
                data = ser_stm32.read(ser_stm32.in_waiting).decode(errors="ignore")
                print(data, end="")  # avoid double newlines
        except Exception as e:
            print(f"[Error] {e}")
            break

def main():
    try:
        with serial.Serial(ESP32_PORT, BAUDRATE, timeout=0.1) as ser_esp32, \
             serial.Serial(STM32_PORT, BAUDRATE, timeout=0.1) as ser_stm32:

            print(f"Connected to ESP32 on {ESP32_PORT}")
            print(f"Connected to STM32 on {STM32_PORT}")
            threading.Thread(target=read_from_stm32, args=(ser_stm32,), daemon=True).start()

            while True:
                user_input = input("> ")
                if user_input.lower() in ["exit", "quit"]:
                    break
                ser_esp32.write((user_input + "\n").encode())

    except serial.SerialException as e:
        print(f"[Serial Error] {e}")

if __name__ == "__main__":
    main()
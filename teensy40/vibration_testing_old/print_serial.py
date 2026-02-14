#!/usr/bin/env python3
import sys
import serial

def main():
    if len(sys.argv) < 2:
        print("Usage: python read_serial.py <serial_port> [baudrate]")
        sys.exit(1)

    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Listening on {port} at {baudrate} baud...\n")
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        sys.exit(1)

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(line)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

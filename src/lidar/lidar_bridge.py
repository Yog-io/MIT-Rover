#!/usr/bin/env python3
import serial
import socket
import time

# Configuration
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
UDP_IP = "127.0.0.1"
UDP_PORT = 9090
MIN_STRENGTH = 100
MAX_FAILURES = 100

def run_bridge():
    # Setup UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[LiDAR Bridge] UDP Socket created, targeting {UDP_IP}:{UDP_PORT}")

    # Setup Serial
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"[LiDAR Bridge] Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
            break
        except Exception as e:
            print(f"[LiDAR Bridge] Failed to open serial port: {e}. Retrying in 1s...")
            time.sleep(1)

    consecutive_failures = 0

    while True:
        try:
            # State 0: Wait for first 0x59
            byte0 = ser.read(1)
            if not byte0 or byte0[0] != 0x59:
                consecutive_failures += 1
            else:
                # State 1: Wait for second 0x59
                byte1 = ser.read(1)
                if not byte1 or byte1[0] != 0x59:
                    consecutive_failures += 1
                else:
                    # State 2: Read remaining 7 bytes
                    payload = ser.read(7)
                    if len(payload) == 7:
                        # Validate Checksum
                        frame = bytearray([0x59, 0x59]) + bytearray(payload)
                        checksum = sum(frame[0:8]) & 0xFF
                        if checksum == frame[8]:
                            # Parse distance and strength
                            dist_cm = frame[2] | (frame[3] << 8)
                            strength = frame[4] | (frame[5] << 8)
                            
                            dist_m = dist_cm / 100.0
                            
                            if strength >= MIN_STRENGTH:
                                sock.sendto(f"{dist_m:.2f}".encode('utf-8'), (UDP_IP, UDP_PORT))
                                consecutive_failures = 0
                            else:
                                consecutive_failures += 1
                        else:
                            consecutive_failures += 1
                    else:
                        consecutive_failures += 1

            # Loss of lock logic
            if consecutive_failures > MAX_FAILURES:
                sock.sendto(b"-1.0", (UDP_IP, UDP_PORT))
                consecutive_failures = MAX_FAILURES + 1 # Prevent overflow

        except Exception as e:
            print(f"[LiDAR Bridge] Exception in loop: {e}")
            time.sleep(0.1)

if __name__ == '__main__':
    run_bridge()

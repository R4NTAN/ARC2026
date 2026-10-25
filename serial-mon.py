import serial
import threading
import time
import sys

def read_serial(ser):
    capturing_dump = False
    dump_lines = []
    
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                
                if "#DUMP_BEGIN" in line:
                    capturing_dump = True
                    dump_lines = []
                    print(line)
                    continue
                elif "#DUMP_END" in line:
                    capturing_dump = False
                    print(line)
                    # Auto save dump to file
                    filename = f"measurements_dump_{int(time.time())}.csv"
                    with open(filename, "w", encoding="utf-8") as f:
                        f.write("\n".join(dump_lines) + "\n")
                    print(f"\n[Python Monitor] Success! CSV dump saved to: {filename}\n")
                    continue
                
                if capturing_dump:
                    dump_lines.append(line)
                
                print(line)
        except Exception as e:
            print(f"\n[Python Error] Serial read exception: {e}")
            break

def main():
    print("=========================================")
    print("   ESP32 LoRa Serial Monitor & Dumper    ")
    print("=========================================")
    
    port = input("Enter serial port (e.g., COM3 or /dev/ttyUSB0): ").strip()
    baud = 115200
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"[Python Monitor] Connected to {port} at {baud} baud.")
        print("[Python Monitor] Type commands below. Type 'exit' to quit.\n")
    except Exception as e:
        print(f"[Python Error] Could not open port {port}: {e}")
        return

    # Background thread for reading incoming serial data
    thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
    thread.start()

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() == 'exit':
                break
            if cmd.strip():
                ser.write((cmd + "\n").encode('utf-8'))
    except KeyboardInterrupt:
        print("\n[Python Monitor] Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

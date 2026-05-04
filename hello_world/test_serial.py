import serial
import time

print("Opening COM3 at 115200 baud...")
try:
    ser = serial.Serial('COM3', 115200, timeout=0.1)
    time.sleep(5)
    
    print("=== Serial Monitor (waiting 45 seconds) ===")
    start_time = time.time()
    buffer = ""
    json_count = 0
    other_count = 0
    
    while time.time() - start_time < 45:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        if line.startswith('{'):
                            json_count += 1
                            print(f"[JSON {json_count}] {line}")
                            if json_count >= 30:
                                break
                        else:
                            other_count += 1
                            if other_count <= 10:
                                print(f"[LOG {other_count}] {line}")
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.5)
    
    ser.close()
    print(f"\n=== Done ===")
    print(f"JSON lines: {json_count}")
    print(f"Other lines: {other_count}")
except Exception as e:
    print(f"Failed to open COM3: {e}")

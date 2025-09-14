import serial
import time

PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

try:
    # Wait to ensure port is free
    time.sleep(2)
    
    # Open serial connection
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print(f"Connected to {PORT}. Press Ctrl+C to exit.\n")

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            values = line.split(',')
            
            if len(values) == 9:
                print("\n--- Sensor Data ---")
                print(f"Temperature: {values[0]} °C")
                print(f"Gas: {values[1]}")
                print(f"Soil Humidity: {values[2]}%")
                print(f"Soil Temp: {values[3]} °C")
                print(f"Conductivity: {values[4]} µS/cm")
                print(f"pH: {values[5]}")
                print(f"N: {values[6]} | P: {values[7]} | K: {values[8]} (mg/kg)")
            else:
                print(f"Error: Expected 9 values, got {len(values)}")

        time.sleep(0.1)

except serial.SerialException as e:
    print(f"Error: {e}")
    print("Try: 1) Unplug/plug Arduino, 2) Close other programs, 3) Run `sudo kill -9 $(sudo lsof -t /dev/ttyUSB0)`")
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close() if 'ser' in locals() else None

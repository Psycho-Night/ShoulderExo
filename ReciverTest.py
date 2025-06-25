import serial

import serial

def main():
    # Set up serial connection (adjust port and baudrate as needed)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Change '/dev/ttyUSB0' to your port
    print("Connected to Arduino")
    
    try:
        while True:
            if ser.in_waiting > 0:
                # line = ser.readline().decode('utf-8').strip()
                data = ser.readline().decode('utf-8', errors='ignore').strip()  # Read incoming data
                print(data)
                data_array = data.split(';')  # Split received data by semicolon
                print(f"Received: {data_array}")
                
                # ser.write((data + '\n').encode('utf-8'))  # Send data back
                # print(f"Sent: {data}")
    
    except KeyboardInterrupt:
        print("Closing connection")
        ser.close()

if __name__ == "__main__":
    main()
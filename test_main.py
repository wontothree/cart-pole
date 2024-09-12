import serial

# set serial port
ser = serial.Serial('COM5', 38400, timeout=0.05)

try:
    while True:
        # is data in buffer
        if ser.in_waiting > 0:
            # read data
            ang = ser.readline().decode('utf-8').strip()
            if ang:
                data = ang.split(',')
                if len(data) == 3:
                    Degree, Rev, RPM = data
                    print(f"Degree: {Degree}, Rev: {Rev}, RPM: {RPM}")

except serial.SerialException as e:
    print(f"SerialException: {e}")
finally:
    if ser.is_open:
        ser.close()

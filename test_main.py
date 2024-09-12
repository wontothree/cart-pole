import serial

# 시리얼 포트 설정 (아두이노와 연결된 포트를 설정)
ser = serial.Serial('/dev/cu.usbmodem34B7DA62EAFC2', 38400, timeout=1)

try:
    while True:
        # 시리얼 데이터 읽기
        line = ser.readline().decode('utf-8').strip()  # 데이터를 읽고 줄바꿈 제거
        if line:
            # Degree 값 출력
            degree = float(line)  # 수신된 데이터를 float로 변환
            print(f"Degree: {degree}")
finally:
    ser.close()  # 시리얼 포트 닫기

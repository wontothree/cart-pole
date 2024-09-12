// 주의사항
// 전압이 부족하면 센서가 다운되기 때문에 아두이노 전원은 아답터로 연결해야 한다.
// RX핀이 업로드를 방해할 수 있기 때문에 코드 업로드 시 아두이노 RX 핀을 연결하지 않고 업로드 완료 후 RX 핀을 연결해야 한다.

float Degree = 0;
float Rev = 0;
float RPM = 0;

void setup() {
  Serial.begin(38400);   // USB 시리얼 통신 초기화
  Serial1.begin(38400);  // Serial1 통신 초기화
}

void loop() {
  static String inString = "";  // 전체 문자열을 저장할 변수

  // Serial1에서 데이터가 들어오면
  while (Serial1.available()) { 
    char c = Serial1.read();  // 하나의 문자를 읽음
    if (c == '\n') {  // 줄바꿈 문자로 데이터의 끝을 확인
      // 문자열 파싱 및 데이터 출력
      int index1 = inString.indexOf(',');
      int index2 = inString.indexOf(',', index1 + 1);
      int index3 = inString.indexOf(',', index2 + 1);
      int index4 = inString.length();

      // 데이터 파싱 (라벨 제거 후)
      float Degree = inString.substring(index1 + 1, index2).toFloat();
      float Rev = inString.substring(index2 + 1, index3).toFloat();
      int RPM = inString.substring(index3 + 1, index4).toInt();

      // print in sertial monitor
      Serial.print("Degree: ");
      Serial.print(Degree, 1);  // 소수점 1자리까지
      Serial.print(", Rev: ");
      Serial.print(Rev, 3);  // 소수점 3자리까지
      Serial.print(", RPM: ");
      Serial.println(RPM);

      // 문자열 초기화
      inString = "";
    } else {
      inString += c;  // 읽어온 문자를 문자열에 추가
    }
  }
}
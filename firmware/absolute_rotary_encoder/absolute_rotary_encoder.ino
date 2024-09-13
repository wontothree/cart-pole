// 주의사항
// 전압이 부족하면 센서가 다운되기 때문에 아두이노 전원은 아답터로 연결해야 한다.
// RX핀이 업로드를 방해할 수 있기 때문에 코드 업로드 시 아두이노 RX 핀을 연결하지 않고 업로드 완료 후 RX 핀을 연결해야 한다.

float Degree = 0;
// float Rev = 0;
// float RPM = 0;

void setup() {
  // initialize USB Serial communication 
  Serial.begin(38400);
  // initialize Serial1 communication
  Serial1.begin(38400);
}

void loop() {
  static String inString = "";

  // is data in Serial1
  while (Serial1.available()) { 
    // read input
    char encoder_data = Serial1.read();

    if (encoder_data == '\n') {
      //  parsing string 
      int index1 = inString.indexOf(',');
      int index2 = inString.indexOf(',', index1 + 1);
      // int index3 = inString.indexOf(',', index2 + 1);
      // int index4 = inString.length();

      // remove label and parse data
      float Degree = inString.substring(index1 + 1, index2).toFloat();
      // float Rev = inString.substring(index2 + 1, index3).toFloat();
      // int RPM = inString.substring(index3 + 1, index4).toInt();

      // print in sertial monitor
      // Serial.print("Degree: ");
      Serial.print(Degree, 1);  // 1 decimal place
      // Serial.print(", Rev: ");
      // Serial.print(Rev, 3);  // 3 decimal place
      // Serial.print(", RPM: ");
      // Serial.println(RPM);

      // initialize string
      inString = "";
    } else {
      inString += encoder_data;
    }
  }
}
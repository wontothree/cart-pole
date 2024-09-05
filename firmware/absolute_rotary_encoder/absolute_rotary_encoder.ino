// en25-absolute-2.ino
// 주의사항
// 전압이 부족하면 센서가 다운되서 아두이노 전원은 아답터로 연결하기를 권장한다.
// 코드 업로드 시 아두이노 RX 핀을 연결하지 않고 업로드를 완료 후 RX 핀을 연결한다.

float Degree = 0;
float Rev = 0;
int RPM = 0;

void setup() {
  Serial.begin(38400);
}

void loop() {
  if (Serial.available()) {
    String inString = Serial.readStringUntil('\n');

    int index1 = inString.indexOf(',');
    int index2 = inString.indexOf(',', index1 + 1);
    int index3 = inString.indexOf(',', index2 + 1);
    int index4 = inString.length();

    float Degree = inString.substring(index1 + 1, index2).toFloat();
    float Rev = inString.substring(index2 + 1, index3).toFloat();
    int RPM = inString.substring(index3 + 1, index4).toInt();

    Serial.print("Degree: ");
    Serial.print(Degree, 1);
    Serial.print(',');
    Serial.print("Rev");
    Serial.print(Rev, 3);
    Serial.print(',');
    Serial.print("RPM");
    Serial.println(RPM);
  }
}

// 주의사항
// 전압이 부족하면 센서가 다운되기 때문에 아두이노 전원은 아답터로 연결해야 한다.
// RX핀이 업로드를 방해할 수 있기 때문에 코드 업로드 시 아두이노 RX 핀을 연결하지 않고 업로드 완료 후 RX 핀을 연결해야 한다.

float previous_angle = 0;
float current_angle = 0;
unsigned long previous_time = 0;
unsigned long current_time = 0;
float angular_velocity = 0;

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

      // remove label and parse data
      float current_angle = inString.substring(index1 + 1, index2).toFloat();

      current_time = millis();

      if (current_time - previous_time > 0) {
        angular_velocity = (current_angle - previous_angle) / ((current_time - previous_time) / 1000.0);
      }

      // print
      Serial.print(current_angle, 1);  // 1 decimal place
      Serial.print(", ");
      Serial.print(angular_velocity, 2);  // 2 decimal places
      Serial.print("\n");

      // update previous angle and time
      previous_angle = current_angle;
      previous_time = current_time;

      // initialize string
      inString = "";
    } else {
      inString += encoder_data;
    }
  }
}
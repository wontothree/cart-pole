#define LEFT_PHOTO_INTERRUPTER_SENSOR_PIN A0
#define RIGHT_PHOTO_INTERRUPTER_SENSOR_PIN A1
int left_photo_interrupter_sensor_data;
int right_photo_interrupter_sensor_data;

void setup()
{
  Serial.begin(19200);
}

void loop()
{
  left_photo_interrupter_sensor_data = analogRead(LEFT_PHOTO_INTERRUPTER_SENSOR_PIN);
  right_photo_interrupter_sensor_data = analogRead(RIGHT_PHOTO_INTERRUPTER_SENSOR_PIN);
  Serial.println("\nleft_photo_interrupter_sensor_data: ");
  Serial.println(left_photo_interrupter_sensor_data);
  Serial.println("\nright_photo_interrupter_sensor_data: ");
  Serial.println(right_photo_interrupter_sensor_data);
  delay(50);
}
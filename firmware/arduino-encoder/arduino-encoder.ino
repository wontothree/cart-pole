#define PI 3.141592
#define RADIAN_PER_DEGREE 0.0174533

float previous_angle = 0;
float current_angle = 0;
unsigned long previous_time = 0;
unsigned long current_time = 0;
float angular_velocity = 0;

float targetAngle;
float angleError;
float kp, ki, kd;

float cumulativeAngleError;
float lastAngleError;

float lastTime = 0;

float action = 0;

void setPID(float KP, float KI, float KD)
{
  kp = KP;
  ki = KI;
  kd = KD;
}

void setTargetAngle(float targetAngle)
{
  targetAngle = targetAngle;
}

float controlByPID(float angle)
{
  // proportional term
  angleError = angle - targetAngle;

  float currentTime = millis();

  float timeInterval = currentTime - lastTime;

  // integral term
  cumulativeAngleError += angleError * timeInterval;

  // derivative term
  float derivativeAngleError = (angleError - lastAngleError) / timeInterval;

  float output = kp * angleError + ki * cumulativeAngleError + kd * derivativeAngleError;

  // update last values
  lastAngleError = angleError;
  lastTime = currentTime;

  return output;
}

void setup()
{
  // initialize USB Serial communication
  Serial.begin(38400);
  // initialize Serial1 communication
  Serial1.begin(38400);
}

void loop()
{
  static String inString = "";

  // is data in Serial1
  while (Serial1.available())
  {
    // read input
    char encoder_data = Serial1.read();

    if (encoder_data == '\n')
    {
      //  parsing string
      int index1 = inString.indexOf(',');
      int index2 = inString.indexOf(',', index1 + 1);

      // remove label and parse data
      float current_angle = inString.substring(index1 + 1, index2).toFloat();

      current_angle *= RADIAN_PER_DEGREE; // degree to radian
      current_angle -= PI;                // 0 ~ 2pi to -pi ~ pi
      current_angle *= -1;

      current_time = millis();

      // calculate angular velocity
      if (current_time - previous_time > 0)
      {
        angular_velocity = (current_angle - previous_angle) / ((current_time - previous_time) / 1000.0);
      }

      setPID(1, 0, 0);
      setTargetAngle(0);
      action = controlByPID(current_angle) * 100;

      if (action > 20) {
        action = 20;
      } else if (action < -20) {
        action = - 20;
      }

      Serial.print(action, 1);
      Serial.print("\n");

      // print
      // Serial.print(current_angle, 1); // 1 decimal placex
      // Serial.print(", ");
      // Serial.print(angular_velocity, 1);
      // Serial.print("\n");

      // update previous angle and time
      previous_angle = current_angle;
      previous_time = current_time;

      // initialize string
      inString = "";
    }
    else
    {
      inString += encoder_data;
    }
  }
}
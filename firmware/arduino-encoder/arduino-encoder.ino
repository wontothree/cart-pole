void setup()
{
  // initialize USB Serial communication
  Serial.begin(38400);
  Serial1.begin(38400);
}

void loop()
{
  static String inString = "";

  // is data in Serial1
  while (Serial1.available())
  {
    // read input
    char encoder_raw_data = Serial1.read();

    if (encoder_raw_data == '\n')
    {
      //  parsing string
      int index1 = inString.indexOf(',');
      int index2 = inString.indexOf(',', index1 + 1);

      float angle = inString.substring(index1 + 1, index2).toFloat();

      Serial.println(angle);

      // initialize string
      inString = "";
    }
    else
    {
      inString += encoder_raw_data;
    }
  }
}
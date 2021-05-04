#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

VL53L0X left_sensor;
VL53L0X right_sensor;

void set_address(int first_pin, int second_pin)
{
  digitalWrite(first_pin, LOW);
  digitalWrite(second_pin, LOW);

  delay(500);

  digitalWrite(first_pin, HIGH);
  delay(150);
  left_sensor.init(true);
  delay(100);
  left_sensor.setAddress((uint8_t)02);

  digitalWrite(second_pin, HIGH);
  delay(150);
  right_sensor.init(true);
  delay(100);
  right_sensor.setAddress((uint8_t)03);

  Serial.println("addresses set");

  left_sensor.startContinuous();
  right_sensor.startContinuous();

  left_sensor.setMeasurementTimingBudget(20000);
  right_sensor.setMeasurementTimingBudget(20000);
}

float calc_angle(int left_dis, int right_dis) //returns the angle in rads
{
  const int DIS_BETWEEN_SENSORS = 30000; //distance in millimeters

  return atan(DIS_BETWEEN_SENSORS / (left_dis - right_dis)); //could be (right - left)
}
float calc_dis(int left_dis, float angle) //returns the distance between the left sensor and wall
{
  return sin(angle) * left_dis;
}

byte read_from_roborio()
{ // reads from roborio and returns the value
  if (Serial.available() > 0)
    return Serial.read();
  else
    return 0;
}

void send_to_roborio(int data)
{
  Serial.write(data);
}

void setup()
{
  const byte LEFT_SENSOR_PIN = 0;  //set later
  const byte RIGHT_SENSOR_PIN = 0; //set later

  pinMode(LEFT_SENSOR_PIN, OUTPUT);
  pinMode(RIGHT_SENSOR_PIN, OUTPUT);

  set_address(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN);

  Wire.begin();
  Serial.begin(9600);
}

void loop()
{
  int left_dis = left_sensor.readRangeContinuousMillimeters();
  int right_dis = right_sensor.readRangeContinuousMillimeters();

  const byte SHOOTING_SIGN = 0; //sign from roborio to send position while shooting

  if (read_from_roborio() == SHOOTING_SIGN)
  {
    send_to_roborio(calc_angle(left_dis, right_dis));
  }
}
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define ECHO_PIN 11
#define TRIG_PIN 12
#define TEMP_PIN A3

double getDistance(double);
double microsecondsToCentimeter(long, double);
double getTemperature();

double distance = 0;
double distanceTotal = 0;
double temperature = 0;
double temperatureTotal = 0;
int tempCount = 0;
int distCount = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  lcd.init();
  // lcd.backlight();
  Serial.begin(9600);
  temperature = getTemperature();
}

void loop()
{
  tempCount++;
  temperatureTotal += getTemperature();
  if (tempCount == 25)
  {
    temperature = temperatureTotal / tempCount;
    temperatureTotal = 0;
    tempCount = 0;
  }
  lcd.home();
  lcd.print("TEMP:         C");
  lcd.setCursor(6, 0);
  lcd.print(temperature);
  Serial.println(temperature);

  distCount++;
  distanceTotal += getDistance(temperature);
  if (distCount == 5)
  {
    distance = distanceTotal / distCount;
    distanceTotal = 0;
    distCount = 0;
  }
  lcd.setCursor(0, 1);
  lcd.print("DIST:         cm");
  lcd.setCursor(6, 1);
  lcd.print(distance);
  Serial.print(distance, 4);
  Serial.println("cm");
  lcd.home();
  delay(200);
}

double getDistance(double temp)
{
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(ECHO_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return microsecondsToCentimeter(duration, temp);
}

double microsecondsToCentimeter(long duration, double temp)
{
  double d = (double)duration * 0.5 * (331.5 + 0.61 * temp) * 0.0001;
  return d;
}

double getTemperature()
{
  int analogVal = analogRead(TEMP_PIN);
  double inputVolt = double(analogVal) * (5.0 / 1024.0);

  double thermistorVal = (5.0 / inputVolt - 1) * 10000;
  double temp = 1 / (log(thermistorVal / 10000) / 3435 + (1 / (25 + 273.15))) - 273.15;

  return temp;
}

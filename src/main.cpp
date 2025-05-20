#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define ECHO_PIN 11
#define TRIG_PIN 12
#define TEMP_PIN 7

double getDistance(double);
double microsecondsToCentimeter(long, double);
double getTemperature();

double distance = 0;
double temperature = 0;
double temperatureTotal = 0;
int count = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  delay(500);
  temperature = getTemperature();
}

void loop()
{
  temperatureTotal += getTemperature();
  count++;
  if (count == 10)
  {
    temperature = temperatureTotal / count;
    temperatureTotal = 0;
    count = 0;
  }
  lcd.home();
  lcd.print("TEMP:         C");
  lcd.setCursor(6, 0);
  lcd.print(temperature);
  Serial.println(temperature);

  lcd.setCursor(0, 1);
  distance = getDistance(temperature);
  lcd.print("DIST:         cm");
  lcd.setCursor(6, 1);
  lcd.print(distance);
  Serial.print(distance, 4);
  Serial.println("cm");
  lcd.home();
  delay(500);
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
  double du = (double)duration * 0.5;
  double d = du * (331.5 + 0.6 * temp) * 0.0001;
  return d;
}

double getTemperature()
{
  int tempReading = analogRead(TEMP_PIN);
  analogRead(2);

  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK);
  double tempC = tempK - 273.15;
  return tempC;
}

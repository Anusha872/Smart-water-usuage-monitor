#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define TRIG_PIN 2
#define ECHO_PIN 3
#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define BUZZER_PIN 5
#define POTENTIOMETER_PIN A0

Servo flowControlServo;

float echoDuration, waterLevelDistance;
float totalWaterFlowLiters = 0;
unsigned long previousMillis = 0;
const unsigned long updateInterval = 1000;
const float tankCapacityLiters = 20.0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);

  flowControlServo.attach(11);
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.print("Water System");
  delay(2000);
  lcd.clear();
  flowControlServo.write(0);
}

void loop() {
  waterLevelDistance = measureWaterLevel();

  if (waterLevelDistance < 20 && totalWaterFlowLiters>100) {
    digitalWrite(BUZZER_PIN, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Tank Full");
    lcd.setCursor(0, 1);
    lcd.print("Overflowing!!");
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Tank filling");
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
    lcd.setCursor(0, 1);
    
  }
  delay(500);
  lcd.clear();

  int potentiometerValue = analogRead(POTENTIOMETER_PIN);
  int servoAngle = map(potentiometerValue, 0, 1023, 0, 180);
  flowControlServo.write(servoAngle);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    float flowRateLitersPerSecond = servoAngle * 0.05;

    // Only increment if totalWaterFlowLiters is 100 or less
    if (totalWaterFlowLiters <= 100) {
        totalWaterFlowLiters += flowRateLitersPerSecond;
    }

    Serial.print("Water Flow Rate: ");
    Serial.print(flowRateLitersPerSecond);
    Serial.println(" L/s");
    Serial.print("Total Water Flow: ");
    Serial.print(totalWaterFlowLiters);
    Serial.println(" L");

    lcd.setCursor(0, 0);
    lcd.print("Liters: ");
    lcd.print(totalWaterFlowLiters);
    lcd.print("  ");
    delay(800);
    lcd.clear();
}

}

float measureWaterLevel() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  echoDuration = pulseIn(ECHO_PIN, HIGH);
  waterLevelDistance = (echoDuration * 0.034) / 2;

  Serial.print("Water Level: ");
  Serial.print(waterLevelDistance);
  Serial.println(" cm");

  return waterLevelDistance;
}

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>  /

VL53L0X sensor;

// Pin motor
#define ENA 15
#define IN1 18
#define IN2 5
#define IN3 4
#define IN4 2
#define ENB 19

// Servo
#define SERVO_PIN 25  
Servo gripperServo;
const int SERVO_OPEN_POS = 60;
const int SERVO_CLOSE_POS = 98;
const int SERVO_INITIAL_POS = 110;

unsigned long lastCommandTime = 0;             
const unsigned long commandTimeout = 500;      

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inisialisasi sensor
  if (!sensor.init()) {
    Serial.println("Sensor gagal!");
    while (1);
  }
  sensor.startContinuous();

  // Inisialisasi motor
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // Inisialisasi servo
  gripperServo.attach(SERVO_PIN);
  delay(300);
  gripperServo.write(SERVO_INITIAL_POS);  
}

void loop() {
  // 1. Baca sensor dan kirim ke Raspberry Pi
  int distance = sensor.readRangeContinuousMillimeters();
  Serial.println("DIST:" + String(distance));  

  // 2. Baca perintah dari Raspberry Pi
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("MOTOR:")) {
      executeMotorCommand(command);
      lastCommandTime = millis();
    } else if (command == "STOP") {
      stopMotors();
      lastCommandTime = 0;
    } else if (command == "OPEN") {
      gripperServo.write(SERVO_OPEN_POS);
    } else if (command == "CLOSE") {
      gripperServo.write(SERVO_CLOSE_POS);
    }
  }

  // 3. Cek timeout
  if (millis() - lastCommandTime > commandTimeout) {
    stopMotors();
  }

  delay(50);
}

void executeMotorCommand(String cmd) {
  int commaIndex = cmd.indexOf(',');
  int left_speed = cmd.substring(6, commaIndex).toInt();
  int right_speed = cmd.substring(commaIndex + 1).toInt();

  digitalWrite(IN1, (right_speed >= 0) ? HIGH : LOW);
  digitalWrite(IN2, (right_speed >= 0) ? LOW : HIGH);
  analogWrite(ENA, constrain(abs(right_speed), 0, 255));

  digitalWrite(IN3, (left_speed >= 0) ? HIGH : LOW);
  digitalWrite(IN4, (left_speed >= 0) ? LOW : HIGH);
  analogWrite(ENB, constrain(abs(left_speed), 0, 255));
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
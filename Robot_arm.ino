#include <Wire.h>      // Library for I2C communication (for MPU6050 and LCD)
#include <Servo.h>     // Library for controlling servo motors
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(9600);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1);
    }
    Serial.println("MPU6050 Found!");
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Accel X: "); Serial.print(a.acceleration.x);
    Serial.print(", Y: "); Serial.print(a.acceleration.y);
    Serial.print(", Z: "); Serial.println(a.acceleration.z);

    delay(500);
}

#include <Wire.h>                // For I2C communication

LiquidCrystal_I2C lcd(0x27, 16, 2); // Address 0x27, 16x2 LCD

void setup() {
    lcd.begin();
    lcd.backlight();
    lcd.print("Hello, World!");
}

void loop() {
    // Add LCD updates here
}
// Servo motor objects
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoGripper;

// Flex sensor pins
const int flex1Pin = A0;
const int flex2Pin = A1;
const int flex3Pin = A2;
const int flex4Pin = A3;
const int flex5Pin = A4;

// LED pins
const int statusLED = 13; // Status indicator (on-board LED)

// MPU6050 variables
MPU6050 mpu;
float angleX, angleY;

// Servo motor pins
const int basePin = 3;
const int shoulderPin = 5;
const int elbowPin = 6;
const int wristPin = 9;
const int gripperPin = 10;

// LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach servo motors to pins
  servoBase.attach(basePin);
  servoShoulder.attach(shoulderPin);
  servoElbow.attach(elbowPin);
  servoWrist.attach(wristPin);
  servoGripper.attach(gripperPin);

  // Initialize LED
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected!");
    digitalWrite(statusLED, HIGH); // Turn on LED if MPU is connected
  } else {
    Serial.println("Failed to connect to MPU6050!");
    digitalWrite(statusLED, LOW);
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Read flex sensor values
  int flex1Value = analogRead(flex1Pin);
  int flex2Value = analogRead(flex2Pin);
  int flex3Value = analogRead(flex3Pin);
  int flex4Value = analogRead(flex4Pin);
  int flex5Value = analogRead(flex5Pin);

  // Map flex sensor values to servo angles
  int baseAngle = map(flex1Value, 600, 1023, 0, 180);   // Adjust these ranges as needed
  int shoulderAngle = map(flex2Value, 600, 1023, 0, 180);
  int elbowAngle = map(flex3Value, 600, 1023, 0, 180);
  int wristAngle = map(flex4Value, 600, 1023, 0, 180);
  int gripperAngle = map(flex5Value, 600, 1023, 0, 180);

  // Write angles to servos
  servoBase.write(baseAngle);
  servoShoulder.write(shoulderAngle);
  servoElbow.write(elbowAngle);
  servoWrist.write(wristAngle);
  servoGripper.write(gripperAngle);

  // Read MPU6050 data for orientation
  angleX = mpu.getAccelerationX() / 16384.0 * 90;  // Convert raw data to angle
  angleY = mpu.getAccelerationY() / 16384.0 * 90;

  // Use angle data for base movement (optional)
  int mpuBaseAngle = map(angleX, -90, 90, 0, 180);
  servoBase.write(mpuBaseAngle);

  // Update LCD display
  lcd.setCursor(0, 0);
  lcd.print("Base: ");
  lcd.print(baseAngle);
  lcd.print(" Gripper: ");
  lcd.print(gripperAngle);

  lcd.setCursor(0, 1);
  lcd.print("MPU X: ");
  lcd.print((int)angleX);
  lcd.print(" Y: ");
  lcd.print((int)angleY);

  // Print values for debugging
  Serial.print("Flex1: "); Serial.print(flex1Value);
  Serial.print(" Flex2: "); Serial.print(flex2Value);
  Serial.print(" Flex3: "); Serial.print(flex3Value);
  Serial.print(" Flex4: "); Serial.print(flex4Value);
  Serial.print(" Flex5: "); Serial.println(flex5Value);
  Serial.print("MPU Base Angle: "); Serial.println(mpuBaseAngle);

  // LED indication for system status
  if (mpuBaseAngle > 90) {
    digitalWrite(statusLED, HIGH); // Turn on LED when the base is tilted significantly
  } else {
    digitalWrite(statusLED, LOW);  // Turn off LED otherwise
  }

  // Small delay for stability
  delay(100);
}
    

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);


// Joystick connections
int joyX = A1;
int joyY = A0;
int buttonPin = A2;

// Create a start time and flight duration
unsigned long startTime;
unsigned long airspeedDuration = 60000;

// Motor A connections
int motorEnableA = 11;  // enable A
int motorAPin1 = 12;    // input 1
int motorAPin2 = 13;    // input 2

// Motor B connections
int motorEnableB = 10;  // enable B
int motorBPin1 = 8;     // input 3
int motorBPin2 = 9;     // input 4

// Motor C connections
int motorEnableC = 5;    // enable C
int motorCPin1 = 7;      // input 3
int motorCPin2 = 1;      // input 4

// Motor D connections
int motorEnableD = 3;    // enable D
int motorDPin1 = 4;      // input 1
int motorDPin2 = 2;      // input 2

// Dead zone threshold
int deadZone = 50;

// Servo Declarations
Servo myservo;
int ServoPin = 6;


void setup() {

  lcd.begin(16, 2);
  lcd.init(); // initialize the lcd
  lcd.backlight();
 
  // Set motor control pins to OUTPUT
  pinMode(motorEnableB, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(motorEnableD, OUTPUT);
  pinMode(motorDPin1, OUTPUT);
  pinMode(motorDPin2, OUTPUT);
  pinMode(motorEnableA, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorEnableC, OUTPUT);
  pinMode(motorCPin1, OUTPUT);
  pinMode(motorCPin2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Set servo control pin
  myservo.attach(ServoPin);

  // Initialize serial communication
  Serial.begin(9600); 
}




void loop() {

  // Read joystick values
  int joyValueX = analogRead(joyX);
  int joyValueY = analogRead(joyY);
  int buttonState = digitalRead(buttonPin);

  // Start the flight upon the button being pressed
  if (buttonState == LOW) {
    startTime = millis();
    lcd.clear();
    Serial.println("Button pressed!");
    lcd.print("Start Flying!");
    lcd.clear();
    delay(10);
  }
  // Start calculating the elapsed time
  unsigned long elapsedTime = millis() - startTime;

  // End the flight after 60 seconds, stop all motors
  if (elapsedTime > 60000){
    lcd.setCursor(0,0);
    lcd.clear();
    lcd.print("Mission Success!");
    lcd.setCursor(0,1);
    analogWrite(motorEnableB, 0);
    analogWrite(motorEnableD, 0);
    analogWrite(motorEnableA, 0);
    analogWrite(motorEnableC, 0);
  }
  

  // map the airspeed over the elapsed time linearly decreasing
  int airspeed = map(elapsedTime, 0, airspeedDuration, 17400, 250);
  lcd.setCursor(0,0);
  lcd.print("Time: ");
  lcd.print(elapsedTime/1000);
  lcd.print("s");
  
  lcd.setCursor(0,1);
  lcd.print("Speed: ");
  lcd.print(airspeed);
  lcd.print(" mph");

  // Map elapsed time to servo value linearly
  int servoValue1 = map(elapsedTime, 0, airspeedDuration, 35, 60);
  int servoValue2 = map(elapsedTime, 0, airspeedDuration, 25, 0);


  // Display X and Y joystick values (debugging/setup)
  Serial.print("X: ");
  Serial.print(joyValueX);
  Serial.print("\tY: ");
  Serial.println(joyValueY);

  // Apply dead zone to joystick X and Y axis values
  if (abs(joyValueX - 512) < deadZone && abs(joyValueY - 512) < deadZone) {
    // Joystick is in the dead zone, stop motors
    Serial.println("Dead Zone - Stopping Motors");
    analogWrite(motorEnableB, 0);
    analogWrite(motorEnableD, 0);
    analogWrite(motorEnableA, 0);
    analogWrite(motorEnableC, 0);
    myservo.write(30); // Elevons in null position
  } else {
    // Map joystick X/Y-axis values to motor speed
    int motorSpeedX = map(joyValueX, 0, 512, 255, 0);
    int motorSpeedX2 = map(joyValueX, 1023, 512, 255, 0);
    int motorSpeedY = map(joyValueY, 0, 512, 255, 0);
    int motorSpeedY2 = map(joyValueY, 1023, 512, 255, 0);

    // Display mapped motor speed values
    Serial.print("Mapped Speed X: ");
    Serial.print(motorSpeedX);
    Serial.print("\t Y: ");
    Serial.println(motorSpeedY);

    // Case 1 Pitching down
    if (joyValueX > 512 && abs(joyValueY - 512) < deadZone) {
      Serial.println("Case 1 - Pitch down B and D Negative, A and C Positive");
      analogWrite(motorEnableB, motorSpeedX2);
      digitalWrite(motorBPin1, HIGH);
      digitalWrite(motorBPin2, LOW);

      analogWrite(motorEnableD, motorSpeedX2);
      digitalWrite(motorDPin1, HIGH);
      digitalWrite(motorDPin2, LOW);


      analogWrite(motorEnableA, motorSpeedX2);
      digitalWrite(motorAPin1, LOW);
      digitalWrite(motorAPin2, HIGH);


      analogWrite(motorEnableC, motorSpeedX2);
      digitalWrite(motorCPin1, LOW);
      digitalWrite(motorCPin2, HIGH);
    }
    // Case 2 Pitching up
    else if (joyValueX < 512 && abs(joyValueY - 512) < deadZone) {
      Serial.println("Case 2 - Pitch up B and D Positive, A and C Negative");
      analogWrite(motorEnableB, motorSpeedX);
      digitalWrite(motorBPin1, LOW);
      digitalWrite(motorBPin2, HIGH);

      analogWrite(motorEnableD, motorSpeedX);
      digitalWrite(motorDPin1, LOW);
      digitalWrite(motorDPin2, HIGH);

      analogWrite(motorEnableA, motorSpeedX);
      digitalWrite(motorAPin1, HIGH);
      digitalWrite(motorAPin2, LOW);

      analogWrite(motorEnableC, motorSpeedX);
      digitalWrite(motorCPin1, HIGH);
      digitalWrite(motorCPin2, LOW);
    }
    // Case 3 Rolling right 
    else if (joyValueY > 512 && abs(joyValueX - 512) < deadZone) {
      Serial.println("Case 3 Roll right - D and C Negative, B and A Positive");
      analogWrite(motorEnableD, motorSpeedY2);
      digitalWrite(motorDPin1, HIGH);
      digitalWrite(motorDPin2, LOW);

      analogWrite(motorEnableC, motorSpeedY2);
      digitalWrite(motorCPin1, HIGH);
      digitalWrite(motorCPin2, LOW);

      analogWrite(motorEnableA, motorSpeedY2);
      digitalWrite(motorAPin1, LOW);
      digitalWrite(motorAPin2, HIGH);

      analogWrite(motorEnableB, motorSpeedY2);
      digitalWrite(motorBPin1, LOW);
      digitalWrite(motorBPin2, HIGH);
      myservo.write(servoValue1);//put the servo into position based on time
    }
    // Case 4
    else if (joyValueY < 512 && abs(joyValueX - 512) < deadZone) {
      Serial.println("Case 4 Roll left - B and C Positive, B and A Negative");
      analogWrite(motorEnableB, motorSpeedY);
      digitalWrite(motorBPin1, HIGH);
      digitalWrite(motorBPin2, LOW);

      analogWrite(motorEnableC, motorSpeedY);
      digitalWrite(motorCPin1, LOW);
      digitalWrite(motorCPin2, HIGH);

      analogWrite(motorEnableA, motorSpeedY);
      digitalWrite(motorAPin1, HIGH);
      digitalWrite(motorAPin2, LOW);

      analogWrite(motorEnableD, motorSpeedY);
      digitalWrite(motorDPin2, HIGH);
      digitalWrite(motorDPin1, LOW);

      myservo.write(servoValue2);// put the servo into position based on time
    }
  }

  delay(100); 
}
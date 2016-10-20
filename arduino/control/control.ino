/*
  HG7881_Motor_Driver_Example - Arduino sketch

  This example shows how to drive a motor with using HG7881 (L9110) Dual
  Channel Motor Driver Module.  For simplicity, this example shows how to
  drive a single motor.  Both channels work the same way.

  This example is meant to illustrate how to operate the motor driver
  and is not intended to be elegant, efficient or useful.

  Connections:

    Arduino digital output D10 to motor driver input B-IA.
    Arduino digital output D11 to motor driver input B-IB.
    Motor driver VCC to operating voltage 5V.
    Motor driver GND to common ground.
    Motor driver MOTOR B screw terminals to a small motor.

  Related Banana Robotics items:

    BR010038 HG7881 (L9110) Dual Channel Motor Driver Module
    https://www.BananaRobotics.com/shop/HG7881-(L9110)-Dual-Channel-Motor-Driver-Module

  https://www.BananaRobotics.com
*/

#include <Servo.h>

// wired connections
#define H_BRIDGE_1_A_IA 6 // D6 --> Motor 1A Input A --> MOTOR A +
#define H_BRIDGE_1_A_IB 2 // D2 --> Motor 1A Input B --> MOTOR A -

#define H_BRIDGE_1_B_IA 11 // D11 --> Motor 1B Input A --> MOTOR B +
#define H_BRIDGE_1_B_IB 4 // D4 --> Motor 1B Input B --> MOTOR B -

#define H_BRIDGE_2_A_IA 3 // D3 --> Motor 2A Input A --> MOTOR B +
#define H_BRIDGE_2_A_IB 8 // D8 --> Motor 2A Input B --> MOTOR B -

#define H_BRIDGE_2_B_IA 5 // D5 --> Motor 2B Input A --> MOTOR B +
#define H_BRIDGE_2_B_IB 7 // D7 --> Motor 2B Input B --> MOTOR B -

// functional connections
#define MOTOR_1A_PWM H_BRIDGE_1_A_IA // Motor 1A PWM Speed
#define MOTOR_1A_DIR H_BRIDGE_1_A_IB // Motor 1A Direction
#define MOTOR_1B_PWM H_BRIDGE_1_B_IA // Motor 1B PWM Speed
#define MOTOR_1B_DIR H_BRIDGE_1_B_IB // Motor 1B Direction
#define MOTOR_2A_PWM H_BRIDGE_2_A_IA // Motor 2A PWM Speed
#define MOTOR_2A_DIR H_BRIDGE_2_A_IB // Motor 2A Direction
#define MOTOR_2B_PWM H_BRIDGE_2_B_IA // Motor 2B PWM Speed
#define MOTOR_2B_DIR H_BRIDGE_2_B_IB // Motor 2B Direction

#define MOTOR_1A 1
#define MOTOR_1B 2
#define MOTOR_2A 3
#define MOTOR_2B 4

#define TRIGGER 12
#define ECHO    13

#define BUMPER_FRONT1 A4
#define BUMPER_FRONT2 A0
#define BUMPER_REAR1  A1
#define BUMPER_REAR2  A2
#define BUMPER_REAR3  A3

#define GRIP 9 // pin for grip servo motor
#define MIN_SERVO_GRIP_ANGLE 74
#define MAX_SERVO_GRIP_ANGLE 130
#define SERVO_GRIP_ANGLE_CLOSE 80
#define SERVO_GRIP_ANGLE_OPEN 120

#define ULTRASONIC_POLLING_TIME 200 // milliseconds between ultrasonic sensor distance calculation
#define ULTRASONIC_BUMPER_DISTANCE_CM 5

// the actual values for "fast" and "slow" depend on the motor
#define PWM_SLOW 100  // arbitrary slow speed PWM duty cycle
#define PWM_FAST 255 // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 200 // brief delay for abrupt motor changes

#define FORWARD HIGH
#define REVERSE LOW
#define INVERT_DIRECTION false

Servo servoGrip;  // create servo object to control grip

void setup()
{
  Serial.begin( 9600 );
  pinMode( MOTOR_1A_DIR, OUTPUT );
  pinMode( MOTOR_1A_PWM, OUTPUT );
  pinMode( MOTOR_1B_DIR, OUTPUT );
  pinMode( MOTOR_1B_PWM, OUTPUT );
  pinMode( MOTOR_2A_DIR, OUTPUT );
  pinMode( MOTOR_2A_PWM, OUTPUT );
  pinMode( MOTOR_2B_DIR, OUTPUT );
  pinMode( MOTOR_2B_PWM, OUTPUT );

  pinMode( BUMPER_FRONT1, INPUT );
  pinMode( BUMPER_FRONT2, INPUT );
  pinMode( BUMPER_REAR1, INPUT );
  pinMode( BUMPER_REAR2, INPUT );
  pinMode( BUMPER_REAR3, INPUT );

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);

  servoGrip.attach(GRIP);  // attaches the servo on pin GRIP to the servo object

  stopAllMotors();
}

void stopAllMotors()
{
  digitalWrite( MOTOR_1A_DIR, LOW );
  digitalWrite( MOTOR_1A_PWM, LOW );
  digitalWrite( MOTOR_1B_DIR, LOW );
  digitalWrite( MOTOR_1B_PWM, LOW );
  digitalWrite( MOTOR_2A_DIR, LOW );
  digitalWrite( MOTOR_2A_PWM, LOW );
  digitalWrite( MOTOR_2B_DIR, LOW );
  digitalWrite( MOTOR_2B_PWM, LOW );
}

void motorControl(byte byteMotor, boolean boolDirection, byte byteSpeed)
{
  byte byteSpeed_;

  if(INVERT_DIRECTION)
    boolDirection = !boolDirection;

  if(boolDirection == FORWARD)
    byteSpeed_ = 255 - byteSpeed;
  else
    byteSpeed_ = byteSpeed;

  switch(byteMotor)
  {
    case MOTOR_1A:
      digitalWrite( MOTOR_1A_DIR, boolDirection);
      analogWrite( MOTOR_1A_PWM, byteSpeed_);
      break;
    case MOTOR_1B:
      digitalWrite( MOTOR_1B_DIR, boolDirection);
      analogWrite( MOTOR_1B_PWM, byteSpeed_);
      break;
    case MOTOR_2A:
      digitalWrite( MOTOR_2A_DIR, boolDirection);
      analogWrite( MOTOR_2A_PWM, byteSpeed_);
      break;
    case MOTOR_2B:
      digitalWrite( MOTOR_2B_DIR, boolDirection);
      analogWrite( MOTOR_2B_PWM, byteSpeed_);
      break;
    default:
      Serial.println( "Incorrect motor selection" );
      break;
  }
}

void allMotorsFastForward()
{
  // set the motors speed and direction
  motorControl(MOTOR_1A, FORWARD, PWM_FAST);
  motorControl(MOTOR_1B, FORWARD, PWM_FAST);
  motorControl(MOTOR_2A, FORWARD, PWM_FAST);
  motorControl(MOTOR_2B, FORWARD, PWM_FAST);
}


void loop()
{
  boolean isValidInput;
  boolean isBumpersForwardActive = true;
  boolean isBumpersReverseActive = true;
  boolean isGoingForward = false;
  boolean isGoingReverse = false;
  unsigned long currentMillis;
  String strSensorsWord = "";
  char c;
  char last_c = 0x00;

  long duration;
  unsigned long previousMillis = 0;
  float ultrasonicDistanceCms = 1000; // in cms
  int angle = 0;    // variable to store the servo position

  do
  {
    currentMillis = millis();

      // Every ULTRASONIC_POLLING_TIME milliseconds calculate distance ahead
    if(currentMillis - previousMillis > ULTRASONIC_POLLING_TIME)
    {
      previousMillis = currentMillis;

      // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
      // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      digitalWrite(TRIGGER, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIGGER, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER, LOW);

      // Read the signal from the sensor: a HIGH pulse whose
      // duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      duration = pulseIn(ECHO, HIGH, 11640); // timeout equals 2 meters (max distance of interest)

      // convert the time into a distance
      ultrasonicDistanceCms = duration / 58.2;

      strSensorsWord += ultrasonicDistanceCms ;
      strSensorsWord += ";" ;
      strSensorsWord += !digitalRead(BUMPER_FRONT1);
      strSensorsWord += ";" ;
      strSensorsWord += !digitalRead(BUMPER_FRONT2);
      strSensorsWord += ";" ;
      strSensorsWord += !digitalRead(BUMPER_REAR1);
      strSensorsWord += ";" ;
      strSensorsWord += !digitalRead(BUMPER_REAR2);
      strSensorsWord += ";" ;
      strSensorsWord += !digitalRead(BUMPER_REAR3);

      Serial.println(strSensorsWord);
      strSensorsWord = "";
    }

    // get the next character from the serial port
    while( !Serial.available() )
    {
      if((isBumpersForwardActive && (!digitalRead(BUMPER_FRONT1) || !digitalRead(BUMPER_FRONT2) || (ultrasonicDistanceCms < 4 && ultrasonicDistanceCms > 0)) && isGoingForward) ||
         (isBumpersReverseActive && (!digitalRead(BUMPER_REAR1) || !digitalRead(BUMPER_REAR2) || !digitalRead(BUMPER_REAR3)) && isGoingReverse))
      {
        stopAllMotors();
        last_c = 0x00;
      }
    }

    c = Serial.read();

    // Only consider command if different from previous one
    if(c == last_c)
    {
      last_c = c;
      continue;
    }
    else
      last_c = c;


    // execute the menu option based on the character recieved
    switch( c )
    {
      case '1': // 1) Fast forward
        Serial.println( "Fast forward..." );
        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        motorControl(MOTOR_1A, FORWARD, PWM_FAST);
        motorControl(MOTOR_1B, FORWARD, PWM_FAST);
        motorControl(MOTOR_2A, FORWARD, PWM_FAST);
        motorControl(MOTOR_2B, FORWARD, PWM_FAST);
        //allMotorsFastForward();
        isValidInput = true;
        isGoingForward = true;
        isGoingReverse = false;
        break;

      case '2': // 2) Forward
        Serial.println( "Forward..." );
        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_1A, FORWARD, PWM_SLOW);
        motorControl(MOTOR_1B, FORWARD, PWM_SLOW);
        motorControl(MOTOR_2A, FORWARD, PWM_SLOW);
        motorControl(MOTOR_2B, FORWARD, PWM_SLOW);

        isValidInput = true;
        isGoingForward = true;
        isGoingReverse = false;
        break;

      case '3': // 3) Soft stop (preferred)
        Serial.println( "Soft stop (coast)..." );
        stopAllMotors();
        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = false;
        break;

      case '4': // 4) Reverse
        Serial.println( "Reverse..." );
        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_1A, REVERSE, PWM_SLOW);
        motorControl(MOTOR_1B, REVERSE, PWM_SLOW);
        motorControl(MOTOR_2A, REVERSE, PWM_SLOW);
        motorControl(MOTOR_2B, REVERSE, PWM_SLOW);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = true;
        break;

      case '5': // 5) Fast reverse
        Serial.println( "Fast reverse..." );

        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_1A, REVERSE, PWM_FAST);
        motorControl(MOTOR_1B, REVERSE, PWM_FAST);
        motorControl(MOTOR_2A, REVERSE, PWM_FAST);
        motorControl(MOTOR_2B, REVERSE, PWM_FAST);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = true;
        break;

      case '6': // Turn right
        Serial.println( "Turning right..." );

        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2A, FORWARD, PWM_FAST);
        motorControl(MOTOR_1B, FORWARD, PWM_FAST);
        motorControl(MOTOR_1A, FORWARD, PWM_SLOW);
        motorControl(MOTOR_2B, FORWARD, PWM_SLOW);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = false;
        break;

      case '7': // Turn left
        Serial.println( "Turning left..." );

        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2A, FORWARD, PWM_SLOW);
        motorControl(MOTOR_1B, FORWARD, PWM_SLOW);
        motorControl(MOTOR_1A, FORWARD, PWM_FAST);
        motorControl(MOTOR_2B, FORWARD, PWM_FAST);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = false;
        break;

      case '8': // Turn hard right
        Serial.println( "Turning hard right..." );

        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2A, FORWARD, PWM_FAST);
        motorControl(MOTOR_1B, FORWARD, PWM_FAST);
        motorControl(MOTOR_1A, REVERSE, PWM_FAST);
        motorControl(MOTOR_2B, REVERSE, PWM_FAST);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = false;
        break;

      case '9': // Turn hard left
        Serial.println( "Turning hard left..." );

        // // always stop motors briefly before abrupt changes
        // stopAllMotors();
        // delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2A, REVERSE, PWM_FAST);
        motorControl(MOTOR_1B, REVERSE, PWM_FAST);
        motorControl(MOTOR_1A, FORWARD, PWM_FAST);
        motorControl(MOTOR_2B, FORWARD, PWM_FAST);

        isValidInput = true;
        isGoingForward = false;
        isGoingReverse = false;
        break;

      case 'a': // 1A FF
        // always stop motors briefly before abrupt changes
        stopAllMotors();
        delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_1A, FORWARD, PWM_FAST);

        isValidInput = true;
        break;

      case 'b': // 1B FF
        // always stop motors briefly before abrupt changes
        stopAllMotors();
        delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_1B, FORWARD, PWM_FAST);

        isValidInput = true;
        break;

      case 'c': // 2A FF
        // always stop motors briefly before abrupt changes
        stopAllMotors();
        delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2A, FORWARD, PWM_FAST);

        isValidInput = true;
        break;

      case 'd': // 2B FF
        // always stop motors briefly before abrupt changes
        stopAllMotors();
        delay( DIR_DELAY );

        // set the motor speed and direction
        motorControl(MOTOR_2B, FORWARD, PWM_FAST);

        isValidInput = true;
        break;

      case 'e': // toggle bumpers forward
        isBumpersForwardActive = !isBumpersForwardActive;
        Serial.println( "Toggle bumpers forward detection" );
        break;

      case 'f': // toggle bumpers reverse
        isBumpersReverseActive = !isBumpersReverseActive;
        Serial.println( "Toggle bumpers reverse detection" );
        break;

      case 'g': // close grip
        angle = constrain(SERVO_GRIP_ANGLE_CLOSE, MIN_SERVO_GRIP_ANGLE, MAX_SERVO_GRIP_ANGLE);
        servoGrip.write(angle);
        Serial.println("Close grip" );
        break;

      case 'h': // open grip
        angle = constrain(SERVO_GRIP_ANGLE_OPEN, MIN_SERVO_GRIP_ANGLE, MAX_SERVO_GRIP_ANGLE);
        servoGrip.write(angle);
        Serial.println("Open grip");
        break;

      default:
        Serial.println( "Unknown command" );
        isValidInput = false;
        break;
    }
  } while(true);
}
/*EOF*/

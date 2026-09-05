/*
  ESP SERVO PWM CONTROLLER
  ------------------------
  Raspberry Pi:
    - USB camera
    - Python/OpenCV target tracking
    - Calculates final PAN/TILT angles
    - Sends: A,PAN,TILT\n over USB serial

  ESP:
    - Only handles servo PWM
    - Receives PAN/TILT angles from Raspberry Pi
    - Servos are powered from an external regulated 5V supply
    - ESP GND and external servo-supply GND must be common

  Expected serial examples:
    A,90.00,90.00
    A,105.50,84.25
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// -------------------- SERVO PINS --------------------
// Change these two pins if your wiring is different.
const int PAN_SERVO_PIN  = 18;
const int TILT_SERVO_PIN = 19;

// -------------------- SERVO LIMITS --------------------
const float PAN_MIN  = 0.0;
const float PAN_MAX  = 180.0;
const float TILT_MIN = 0.0;
const float TILT_MAX = 180.0;

// Startup/center position
const float PAN_CENTER  = 90.0;
const float TILT_CENTER = 90.0;

// Typical servo pulse range.
// Adjust only if your particular servos require different limits.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// -------------------- OBJECTS --------------------
Servo panServo;
Servo tiltServo;

// Current commanded positions
float panAngle = PAN_CENTER;
float tiltAngle = TILT_CENTER;

// -------------------- SERIAL BUFFER --------------------
String serialBuffer;

// Clamp an angle to the allowed range.
float clampAngle(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

// Move servos to the requested angles.
void setServoAngles(float pan, float tilt)
{
  pan = clampAngle(pan, PAN_MIN, PAN_MAX);
  tilt = clampAngle(tilt, TILT_MIN, TILT_MAX);

  panAngle = pan;
  tiltAngle = tilt;

  panServo.write((int)round(panAngle));
  tiltServo.write((int)round(tiltAngle));
}

// Parse the Raspberry Pi command:
//
// A,92.50,88.70
//
// Returns true only when a valid command was received.
bool parseCommand(String command)
{
  command.trim();

  if (command.length() == 0)
    return false;

  if (command.charAt(0) != 'A')
    return false;

  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0)
    return false;

  String panString =
      command.substring(firstComma + 1, secondComma);

  String tiltString =
      command.substring(secondComma + 1);

  panString.trim();
  tiltString.trim();

  if (panString.length() == 0 || tiltString.length() == 0)
    return false;

  float pan = panString.toFloat();
  float tilt = tiltString.toFloat();

  setServoAngles(pan, tilt);

  return true;
}

void setup()
{
  Serial.begin(115200);

  // ESP32Servo uses ESP32 hardware PWM internally.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // Start centered.
  setServoAngles(PAN_CENTER, TILT_CENTER);

  delay(500);

  Serial.println("ESP SERVO CONTROLLER READY");
  Serial.println("Expected: A,PAN,TILT");
}

void loop()
{
  // Read complete lines from Raspberry Pi.
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\n')
    {
      if (parseCommand(serialBuffer))
      {
        // Optional acknowledgement.
        // Python does not require this response.
        Serial.print("OK,");
        Serial.print(panAngle, 2);
        Serial.print(",");
        Serial.println(tiltAngle, 2);
      }

      serialBuffer = "";
    }
    else if (c != '\r')
    {
      serialBuffer += c;

      // Prevent an accidentally corrupted/huge serial line.
      if (serialBuffer.length() > 80)
      {
        serialBuffer = "";
      }
    }
  }
}

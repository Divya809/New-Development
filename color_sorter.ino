#include <ESP32Servo.h> // Use ESP32Servo library for ESP32 compatibility
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_TCS34725.h>

// OLED (1-Page Buffer saves RAM)
U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorSensorOK = false;

// ESP32 PIN DEFINITIONS
#define I2C_SDA 21
#define I2C_SCL 22
#define IR_SENSOR_PIN 33
#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define ENA 25
#define IN1 26
#define IN2 27

// SERVO POSITIONS & SPEED
#define SERVO_HOME 0
#define SERVO1_SORT 60
#define SERVO2_SORT 60
#define MOTOR_SPEED 150

Servo servo1;
Servo servo2;

// COLOR SETTINGS (CHANGE THESE TO TUNE SENSITIVITY)
#define ORANGE_DOMINANCE 3   // Lowered to 3 because Orange has both Red and Green
#define GREEN_DOMINANCE  5   // Sensitivity for Green
#define BLUE_DOMINANCE   5   // Sensitivity for Blue

#define MIN_CLEAR_LEVEL   20
#define WHITE_CLEAR_LEVEL 180
#define WHITE_DIFFERENCE  8

// COLOR SENSOR TIMEOUT
#define COLOR_TIMEOUT 1000

// COUNTERS
unsigned long orangeCount = 0;
unsigned long greenCount = 0;
unsigned long blueCount = 0;

// INTERRUPT FLAG & STATE
volatile bool objectDetected = false;

enum SystemState
{
  MOTOR_RUNNING,
  COLOR_SENSING
};

volatile SystemState state = MOTOR_RUNNING;

unsigned long colorStartTime = 0;

// IR INTERRUPT (INSTANT HARDWARE BRAKE)
// IRAM_ATTR ensures this runs fast from RAM on ESP32
void IRAM_ATTR IR_ISR()
{
  if (state == MOTOR_RUNNING)
  {
    // Short motor terminals together for instant brake
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);

    objectDetected = true;
  }
}

// MOTOR START
void motorStart()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_SPEED); // analogWrite is natively supported in ESP32 Core v3+
}

// MOTOR STOP (ACTIVE ELECTROMAGNETIC BRAKE)
void motorStop()
{
  // 1. Force hard electromagnetic brake (IN1=LOW, IN2=LOW, ENA=HIGH)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, HIGH);

  // Hold brake for 80ms to completely kill belt momentum
  delay(80);

  // 2. Cut enable power so the driver does not overheat while waiting
  analogWrite(ENA, 0);
}

// OLED - SHOW COUNTS (DEFAULT SCREEN)
void updateDisplay()
{
  display.firstPage();
  do
  {
    display.setFont(u8g2_font_6x10_tf);

    display.drawStr(0, 12, "COLOR SORTING");

    display.drawStr(0, 30, "ORANGE:");
    display.setCursor(48, 30);
    display.print(orangeCount);

    display.drawStr(0, 42, "GREEN:");
    display.setCursor(48, 42);
    display.print(greenCount);

    display.drawStr(0, 54, "BLUE:");
    display.setCursor(48, 54);
    display.print(blueCount);
  } while (display.nextPage());
}

// OLED - SHOW DETECTED COLOR IN CENTER
void showColorName(const char* colorName)
{
  display.firstPage();
  do
  {
    display.setFont(u8g2_font_ncenB14_tr);

    // Calculate horizontal & vertical center on 128x64 OLED
    int x = (128 - display.getStrWidth(colorName)) / 2;
    int y = 38;

    display.drawStr(x, y, colorName);
  } while (display.nextPage());
}

// SERVO 1 - ORANGE
void activateServo1()
{
  servo1.write(SERVO1_SORT);
  delay(1000);

  servo1.write(SERVO_HOME);
  delay(300);
}

// SERVO 2 - GREEN
void activateServo2()
{
  servo2.write(SERVO2_SORT);
  delay(1500);

  servo2.write(SERVO_HOME);
  delay(300);
}

// COLOR DETECTION
int detectColor()
{
  if (!colorSensorOK)
  {
    return 0;
  }

  uint16_t r, g, b, c;

  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("R: ");
  Serial.print(r);
  Serial.print("  G: ");
  Serial.print(g);
  Serial.print("  B: ");
  Serial.print(b);
  Serial.print("  C: ");
  Serial.println(c);

  // TOO DARK
  if (c < MIN_CLEAR_LEVEL)
  {
    return 0;
  }

  uint16_t maxRGB = max(r, max(g, b));
  uint16_t minRGB = min(r, min(g, b));

  // WHITE
  if (c >= WHITE_CLEAR_LEVEL &&
      (maxRGB - minRGB) <= WHITE_DIFFERENCE)
  {
    Serial.println("WHITE");
    return 0;
  }

  // 1. ORANGE (Servo 1)
  if (r > g + ORANGE_DOMINANCE &&
      r > b + ORANGE_DOMINANCE)
  {
    Serial.println("ORANGE DETECTED");

    orangeCount++;
    showColorName("ORANGE");

    return 1;
  }

  // 2. GREEN (Servo 2)
  if (g > r + GREEN_DOMINANCE &&
      g > b + GREEN_DOMINANCE)
  {
    Serial.println("GREEN DETECTED");

    greenCount++;
    showColorName("GREEN");

    return 2;
  }

  // 3. BLUE (No Servo)
  if (b > r + BLUE_DOMINANCE &&
      b > g + BLUE_DOMINANCE)
  {
    Serial.println("BLUE DETECTED");

    blueCount++;
    showColorName("BLUE");

    return 3;
  }

  return 0;
}

// SETUP
void setup()
{
  Serial.begin(115200); // Standard higher baud rate for ESP32 debugging

  // IR
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);

  // MOTOR
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // ESP32 SERVO SETUP
  // Allows ESP32Servo to allocate proper PWM timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50); // Standard 50hz servo
  servo2.setPeriodHertz(50);

  servo1.attach(SERVO1_PIN, 500, 2400); // Attach with standard min/max widths
  servo2.attach(SERVO2_PIN, 500, 2400);

  servo1.write(SERVO_HOME);
  servo2.write(SERVO_HOME);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL); // Explicitly configure SDA and SCL pins

  // OLED
  display.begin();

  display.firstPage();
  do
  {
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 20, "COLOR SORTING");
    display.drawStr(0, 40, "SYSTEM START");
  } while (display.nextPage());

  delay(500);

  // COLOR SENSOR
  if (tcs.begin())
  {
    colorSensorOK = true;
    Serial.println("TCS34725 FOUND");
  }
  else
  {
    colorSensorOK = false;
    Serial.println("TCS34725 NOT FOUND");
  }

  updateDisplay();

  // IR INTERRUPT
  attachInterrupt(
    digitalPinToInterrupt(IR_SENSOR_PIN),
    IR_ISR,
    FALLING
  );

  // MOTOR START
  motorStart();

  Serial.println();
  Serial.println("==============================");
  Serial.println("MOTOR RUNNING CONTINUOUSLY");
  Serial.println("==============================");
}

// LOOP
void loop()
{
  // STATE 1 - MOTOR RUNNING
  if (state == MOTOR_RUNNING)
  {
    // CHECK IR INTERRUPT FIRST (Before motorStart!)
    if (objectDetected)
    {
      // 1. Lock brake immediately before any Serial printing
      motorStop();

      objectDetected = false;

      // 2. Print status after motor is completely stopped
      Serial.println();
      Serial.println("==============================");
      Serial.println("OBJECT DETECTED");
      Serial.println("==============================");
      Serial.println("MOTOR STOPPED");

      // 3. Start color sensor timer
      colorStartTime = millis();

      state = COLOR_SENSING;

      Serial.println("COLOR SENSOR ACTIVE");
      Serial.println("TIMEOUT = 1 SECOND");
    }
    else
    {
      // Keep motor running only when no object is detected
      motorStart();
    }
  }

  // STATE 2 - COLOR SENSING
  else if (state == COLOR_SENSING)
  {
    // TRY TO DETECT COLOR
    int detectedColor = detectColor();

    // COLOR FOUND
    if (detectedColor != 0)
    {
      Serial.println();
      Serial.println("COLOR DETECTED");

      // START MOTOR FIRST
      motorStart();

      Serial.println("MOTOR RESTARTED");

      // SORT (OLED holds color name until servo is home)
      if (detectedColor == 1)
      {
        Serial.println("ORANGE -> SERVO 1");
        activateServo1();
      }
      else if (detectedColor == 2)
      {
        Serial.println("GREEN -> SERVO 2");
        activateServo2();
      }
      else if (detectedColor == 3)
      {
        Serial.println("BLUE -> NO SERVO");
        delay(500);
      }

      // SERVO IS BACK HOME -> SHOW COUNTS AGAIN
      updateDisplay();

      // RETURN TO MOTOR RUNNING
      motorStart();

      // Clear false IR triggers from delay bouncing
      objectDetected = false;

      state = MOTOR_RUNNING;

      Serial.println("MOTOR RUNNING CONTINUOUSLY");
      Serial.println("READY FOR NEXT OBJECT");
      Serial.println();
    }

    // TIMEOUT
    else if (millis() - colorStartTime >= COLOR_TIMEOUT)
    {
      Serial.println();
      Serial.println("COLOR TIMEOUT");
      Serial.println("NO COLOR DETECTED");

      // MOTOR START
      motorStart();

      Serial.println("MOTOR RESTARTED");
      Serial.println("READY FOR NEXT OBJECT");

      // Clear false IR triggers from the timeout
      objectDetected = false;

      state = MOTOR_RUNNING;

      Serial.println();
    }
  }
}

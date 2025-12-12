#include <Servo.h>
#include <IRremote.h>
#include <Adafruit_NeoPixel.h>

#define IR_RECEIVE_PIN 9 // Define the pin connected to the IR receiver
#define LED_PIN 4 
#define LED_COUNT 1 
Adafruit_NeoPixel rgb(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); 



// motors
#define AIN_1 8 // AIN1
#define BIN_1 7 // BIN1
#define BIN_PWM 6 // B_PWM
#define AIN_PWM 5 // A_PWM
#define standby 3

//remote 0xValue
#define IR_top 0xB946FF00
#define IR_bottom 0xEA15FF00
#define IR_left 0xBB44FF00
#define IR_right 0xBC43FF00
#define IR_stop 0xBF40FF00

#define IR_speed_down  0xBD42FF00 // button (*)
#define IR_speed_up  0xB54AFF00 // button (#)

#define IR_IR 0xE916FF00 // button (1)
#define IR_OBJECT 0xE619FF00  // button (2)
#define IR_TRACKING 0xF20DFF00  // button (3)
#define IR_MANUAL 0xF30CFF00  // button (4)

enum MODE {
  IR,
  OBJECT,
  TRACKING,
  MANUAL,
  SLEEP
};

MODE currentMode = SLEEP;
unsigned long modeVal = 0;

// Line tracking
#define L3 A2
#define M4 A1
#define R5 A0

// OBSTICAL AVOIDANCE
#define servo 10// Servo
Servo myservo;
#define ULTecho 12 // Ultrasonic
#define ULTtrig 13

const int PROXIMITY_THRESHOLD = 25;
long distance_cm;
enum pos {
  straight,
  scan_left,
  scan_right
};
int angle = 90;
pos state = straight;
String lastDirection = "STOP";

int motorSpeed = 200;

//Direction lastDirection = STOP;

void setup() {
  // put your setup code here, to run once:
  int Motor_array[7] = {AIN_1, BIN_1, BIN_PWM, AIN_PWM, standby, ULTtrig};
  for ( int Motor = 0; Motor < 7; Motor++) {
    pinMode(Motor_array[Motor], OUTPUT);
  }
  pinMode(ULTecho, INPUT);
      Serial.begin(9600);
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Initialize the receiver
    myservo.attach(10);
    myservo.write(angle);

  rgb.begin();
  rgb.show();
  rgb.setBrightness(50);
}

void loop() {
  // put your main code here, to run repeatedly:
if (IrReceiver.decode()) {
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      modeVal = IrReceiver.decodedIRData.decodedRawData;
    }
    IrReceiver.resume();
    }

if (modeVal == IR_IR) {
  Stop();
  currentMode = IR;
  modeVal = 0;
}

else if (modeVal == IR_OBJECT) { 
Stop();
 currentMode = OBJECT; 
 modeVal = 0;
}

else if (modeVal == IR_TRACKING) { 
Stop(); 
currentMode = TRACKING;
modeVal = 0;
}

else if (modeVal == IR_MANUAL) {
 Stop();
 delay(1000);
currentMode = MANUAL;
modeVal = 0;
}

else if (modeVal == IR_stop){
  currentMode = SLEEP;
  modeVal = 0;
}

switch(currentMode) {
  case IR:
rgb.setPixelColor(0, rgb.Color(128,128,0));
rgb.show();
  IR_remote ();
  break;

case OBJECT:
rgb.setPixelColor(0,rgb.Color(128,0,128));
rgb.show();
object_avoidance();
break;

case TRACKING:
rgb.setPixelColor(0, rgb.Color(0,128,128));
rgb.show();
 light_tracking(); 
break;
case MANUAL:
rgb.setPixelColor(0, rgb.Color(0, 255, 0));
rgb.show();
Manual();
 break;
case SLEEP:
rgb.setPixelColor(0, rgb.Color(255, 0, 0));
rgb.show();
 Stop(); 
break;
}
}

void object_avoidance() {
  // 1. Read Distance
  distance_cm = calculateDistance();
  Serial.print("State: ");

  switch(state) {
    // --- CASE 1: LOOKING STRAIGHT ---
    case straight:
      Serial.print("STRAIGHT");
      myservo.write(90); 
      Forward(120);
      // LOGIC: If path is blocked (< 25cm), switch state to scan LEFT
      if (distance_cm < PROXIMITY_THRESHOLD && distance_cm > 0) {
        Serial.println(" -> Blocked! Checking LEFT.");
        state = scan_left;
        Stop();
      } else {
        // [cite: 6] If clear, Robot would normally move "Forward" here
        Serial.println(" -> Path Clear (Forward)");
      }
      break;
    // --- CASE 2: SCANNING LEFT ---
    case scan_left:
      Serial.print("SCANNING LEFT");
      myservo.write(180); // Look fully Left
      delay(500); // Wait for servo to reach position
      distance_cm = calculateDistance(); // Read sensor at new angle
      // LOGIC: If Left is clear, go back to Straight (Robot turns Left) [cite: 7]
      if (distance_cm >= PROXIMITY_THRESHOLD) {
         state = straight;
         Serial.println(" -> Left is Clear. Turning Left.");
         Left(120);
         delay(450);
      }
      // LOGIC: If Left is BLOCKED, switch state to scan RIGHT
      else {
         state = scan_right;
         Serial.println(" -> Left Blocked! Checking RIGHT.");
      }
      break;
    // --- CASE 3: SCANNING RIGHT ---
    case scan_right:
      Serial.print("SCANNING RIGHT");
      myservo.write(0); // Look fully Right
      delay(500); // Wait for servo to reach position
      distance_cm = calculateDistance(); // Read sensor at new angle
      // LOGIC: If Right is clear, go back to Straight (Robot turns Right) [cite: 16]
      if (distance_cm >= PROXIMITY_THRESHOLD) {
         state = straight;
         Serial.println(" -> Right is Clear. Turning Right.");
         Right(120);
         delay(450);
      }
      // LOGIC: If Right is ALSO blocked, Robot must reverse [cite: 25]
      else if (distance_cm < PROXIMITY_THRESHOLD && distance_cm > 0) {
         state = straight; // Reset to straight to try again (or add a 'reverse' state)
         Serial.println(" -> All Blocked! Reversing.");
         myservo.write(90); // Look fully Right
        delay(500); // Wait for servo to reach position
         Backward(120);
         delay(450);
         Left(120);
         delay(1300);
      }
      break;
  }
  Serial.print(" | Dist: ");
  Serial.println(distance_cm);
  delay(200); // Small delay for stability
}

long calculateDistance() {
  digitalWrite(ULTtrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTtrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTtrig, LOW);
  long duration = pulseIn(ULTecho, HIGH);
  return duration * 0.034 / 2;
}

void light_tracking() {
  // --- LINE TRACKING ---

  int leftStatus = analogRead(L3);
  int midStatus = analogRead(M4);
  int rightStatus = analogRead(R5);

  // Convert analog to digital based on THRESHOLD
  int l = leftStatus; // Assuming Line is Dark (Low Value)
  int m = midStatus;
  int r = rightStatus;

  if (l && m && r > 1000) Stop();
  else if (m > 700) Forward(60);
  else if (l > 700) Left(60);
  else if (r > 700) Right(60);

  else Right(60);

}

void Forward(int speedVal) {
  digitalWrite(AIN_1, 1);
  digitalWrite(BIN_1, 1);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void Backward(int speedVal) {
  digitalWrite(AIN_1, 0);
  digitalWrite(BIN_1, 0);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void Right(int speedVal) {
  digitalWrite(AIN_1, 1);
  digitalWrite(BIN_1, 0);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void Left(int speedVal) {
  digitalWrite(AIN_1, 0);
  digitalWrite(BIN_1, 1);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void LeftForward(int speedVal) {
  digitalWrite(AIN_1, 0);
  digitalWrite(BIN_1, 1);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal/2);

  digitalWrite(standby, 1);
}

void RightForward(int speedVal) {
  digitalWrite(AIN_1, 1);
  digitalWrite(BIN_1, 1);

  analogWrite(AIN_PWM, speedVal/2);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void LeftBackward(int speedVal) {
  digitalWrite(AIN_1, 0);
  digitalWrite(BIN_1, 0);

  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal/2);

  digitalWrite(standby, 1);

}

void RightBackward(int speedVal) {
  digitalWrite(AIN_1, 0);
  digitalWrite(BIN_1, 0);

  analogWrite(AIN_PWM, speedVal/2);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(standby, 1);
}

void Stop () {
  analogWrite(AIN_PWM, 0);
  analogWrite(BIN_PWM, 0);

  digitalWrite(standby, 1);
}

void turnLeft180(int speedVal) {
  int leftStatus = digitalRead(L3);
  int midStatus = digitalRead(M4);
  int rightStatus = digitalRead(R5);
  // 1. Set Speed (0-255)
  // 180 turns need lower speed to prevent overshooting the line
  analogWrite(AIN_PWM, speedVal);
  analogWrite(BIN_PWM, speedVal);

  digitalWrite(AIN_1, LOW);
  digitalWrite(BIN_1, HIGH);
  delay(150);
}

void Manual() {

  Forward(200); delay(4700);
  Stop();      delay(1000);
  Right(100);   delay(550);
  Stop();      delay(1020);  
  Forward(200); delay(4700);
  Stop();      delay(1000);
  Right(100);   delay(500);
  Stop();      delay(1000);  
  Forward(200); delay(4300);  
  Stop();      delay(1000);
  Right(100);   delay(550);  
  Stop();      delay(1000);
  Forward(200); delay(2500);
  Stop();      delay(1000);
  Right(100);   delay(550);  
  Stop();      delay(1000);
}

void IR_remote () {
 if (modeVal == 0) return;

      switch (modeVal) {
      case IR_top:
       Forward(motorSpeed);
       lastDirection = "Forward";
      break;
      case IR_bottom:
        Backward(motorSpeed);
        lastDirection = "Backward";
      break;
      case IR_left:
        Left(motorSpeed);
        lastDirection = "Left";
      break;
      case IR_right:
        Right(motorSpeed);
        lastDirection = "Right";
      break;
      case IR_speed_up:
             motorSpeed += 20;
             if(motorSpeed > 255) motorSpeed = 255;
             applyLastDirection();
             break;
          case IR_speed_down:
             motorSpeed -= 20;
             if(motorSpeed < 0) motorSpeed = 0;
              applyLastDirection();
             break;
      /*case IR_stop:
        Stop();
      break;*/
    }
modeVal = 0;
}

void applyLastDirection() {
  if (lastDirection == "Forward") Forward(motorSpeed);
  else if (lastDirection == "Backward") Backward(motorSpeed);
  else if (lastDirection == "Left") Left(motorSpeed);
  else if (lastDirection == "Right") Right(motorSpeed);
}
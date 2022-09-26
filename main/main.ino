class Wheels {
public:
  // Sets motor control pins for the wheels to OUTPUT, initializes members which refer to wheel motor control pins. 
  Wheels(const byte left_forward, const byte left_backward, const byte right_forward, const byte right_backward, const byte motor_speed) : 
         pin_left_fw(left_forward), pin_left_bw(left_backward), pin_right_fw(right_forward), pin_right_bw(right_backward), pin_motor_speed(motor_speed) {
    pinMode(left_forward, OUTPUT);
    pinMode(left_backward, OUTPUT);
    pinMode(right_forward, OUTPUT);
    pinMode(right_backward, OUTPUT);
    pinMode(motor_speed, OUTPUT);
  }
  void go();
  void stop(); // Need to investigate breaking. Currently just sending voltage to the wheels.
  void reverse();
  void turn_right();
  void turn_left();
  void set_speed(byte new_speed);
  const byte FAST = 255;
  const byte MEDIUM = 180;
  const byte SLOW = 100;
private:
  byte speed = Wheels::MEDIUM;
  byte pin_left_fw;
  byte pin_left_bw;
  byte pin_right_fw;
  byte pin_right_bw;
  byte pin_motor_speed;
};

void Wheels::go() {
  Serial.println("Going!");
  digitalWrite(pin_left_bw, LOW);
  digitalWrite(pin_right_bw, LOW);
  digitalWrite(pin_left_fw, HIGH);
  digitalWrite(pin_right_fw, HIGH);
  analogWrite(pin_motor_speed, speed);
}

void Wheels::stop() {
  Serial.println("Stopped.");
  digitalWrite(pin_left_bw, LOW);
  digitalWrite(pin_right_bw, LOW);
  digitalWrite(pin_left_fw, LOW);
  digitalWrite(pin_right_fw, LOW);
  analogWrite(pin_motor_speed, 0);
}

void Wheels::reverse() {
  Serial.println("Backing up.");
  digitalWrite(pin_left_fw, LOW);
  digitalWrite(pin_right_fw, LOW);
  digitalWrite(pin_left_bw, HIGH);
  digitalWrite(pin_right_bw, HIGH);
  analogWrite(pin_motor_speed, speed);
}

void Wheels::turn_right() {
  Serial.println("Right Turn");
  stop();
  digitalWrite(pin_right_bw, HIGH);
  digitalWrite(pin_left_fw, HIGH);
  analogWrite(pin_motor_speed, SLOW);
}

void Wheels::turn_left() {
  Serial.println("Left Turn");
  stop();
  digitalWrite(pin_left_bw, HIGH);
  digitalWrite(pin_right_fw, HIGH);
  analogWrite(pin_motor_speed, SLOW);
}

void Wheels::set_speed(byte new_speed) {
  if (speed < 0 || speed > 255) { // Invalid speed
    return;
  }

  if (speed == 0) { // If not currently moving.
    speed = new_speed;
    return;
  }

  analogWrite(pin_motor_speed, speed);
  speed = new_speed;
}

// ========================================================
// ========================================================
// ========================================================
// ========================================================
const int ledPin = 2; // temporary for testing

void setup() {
  Serial.begin(9600); // Needed to print to Serial Monitor.
  pinMode(ledPin, OUTPUT);    // { temporary
  digitalWrite(ledPin, HIGH); //   for testing }
  Serial.println("\nSetup complete.\n");
}

// Need to figure out what the actual pins will be.
#define PIN_LEFT_FW 2
#define PIN_LEFT_BW 4
#define PIN_RIGHT_FW 7
#define PIN_RIGHT_BW 8
#define PIN_WHEELS_SPEED 3

void loop() {
  static const Wheels wheels(PIN_LEFT_FW, PIN_LEFT_BW, PIN_RIGHT_FW, PIN_RIGHT_BW, PIN_WHEELS_SPEED);
  // Test script {
  wheels.go();
  delay(3000);
  wheels.reverse();
  delay(3000);
  wheels.turn_right();
  delay(3000);
  wheels.turn_left();
  delay(3000);
  wheels.stop();
  delay(5000);
  // }
}

// Pin definitions for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// Motor 1 control pins
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR1_PWM 11

// Motor 2 control pins
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR2_PWM 3

// Motor 3 control pins
#define MOTOR3_A 5
#define MOTOR3_B 7 
#define MOTOR3_PWM 6

// Motor 4 control pins 
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR4_PWM 5

// Define movements 
#define FORWARD 1
#define REVERSE 2
#define RELEASE 3
#define BRAKE 4
#define LEFT_FORWARD 5
#define RIGHT_FORWARD 6
#define LEFT_REVERSE 7
#define RIGHT_REVERSE 8

void setup() {
  Serial.begin(9600);
  Serial.println("Starting cockroach");

  // Initialize the shift register
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  // Set default values for the shift register
  digitalWrite(MOTORDATA, LOW);
  digitalWrite(MOTORLATCH, LOW);
  digitalWrite(MOTORCLK, LOW);
  digitalWrite(MOTORENABLE, LOW);
}

void loop() {
  drive_motor(1, FORWARD, 150);
  drive_motor(2, FORWARD, 150);
  drive_motor(3, FORWARD, 150);
  drive_motor(4, FORWARD, 150);
  delay(2000); // Run for 2 seconds

}

void drive_motor(int motor_num, int command, int speed) {
  int motorA, motorB, motorPWM;
  switch (motor_num) {
    case 1:
      motorA = MOTOR1_A;
      motorB = MOTOR1_B;
      motorPWM = MOTOR1_PWM;
      break;
    case 2:
      motorA = MOTOR2_A;
      motorB = MOTOR2_B;
      motorPWM = MOTOR2_PWM;
      break;
    case 3:
      motorA = MOTOR3_A;
      motorB = MOTOR3_B;
      motorPWM = MOTOR3_PWM;
      break;
    case 4:
      motorA = MOTOR4_A;
      motorB = MOTOR4_B;
      motorPWM = MOTOR4_PWM;
      break;
  }

  switch (command) {
    case FORWARD: 
      shiftWrite(motorA, HIGH);
      shiftWrite(motorB, LOW);
      analogWrite(motorPWM, speed);
  }

}

void shiftWrite(int output, int high_low) {
  static int latch_copy;
  static int shift_register_initialized = false;

  // Initialize the shift register on first use
  if (!shift_register_initialized) {
    latch_copy = 0;
    shift_register_initialized = true;
  }

  // Update the latch copy with the new output state
  bitWrite(latch_copy, output, high_low);

  // Shift out the data to the shift register
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);

  // Latch the data to the output pins
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // Small delay for safety
  digitalWrite(MOTORLATCH, LOW);
}
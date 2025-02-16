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
  // Move Motor M1 forward at full speed
  shiftWrite(MOTOR3_A, HIGH); 
  shiftWrite(MOTOR3_B, LOW);  
  analogWrite(MOTOR1_PWM, 255); 
  delay(2000); // Run for 2 seconds

  // Move Motor M4 forward at full speed
  shiftWrite(MOTOR4_A, HIGH); 
  shiftWrite(MOTOR4_B, LOW);  
  analogWrite(MOTOR2_PWM, 255); 
  delay(2000); 

  // Stop Motor M3
  shiftWrite(MOTOR3_A, LOW); 
  shiftWrite(MOTOR3_B, LOW); 
  analogWrite(MOTOR3_PWM, 0);
  delay(1000); 

  // Stop Motor M4
  shiftWrite(MOTOR4_A, LOW); 
  shiftWrite(MOTOR4_B, LOW); 
  analogWrite(MOTOR4_PWM, 0);
  delay(1000);

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
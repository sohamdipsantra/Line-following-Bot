// Motor A
const int ENA = 11; // PWM pin for Motor A
const int IN1 = 10;  // IN1 pin for Motor A
const int IN2 = 9;  // IN2 pin for Motor A
#define s 100 //base speed
#define t 130//turning speed


// Motor B
const int ENB = 6; // PWM pin for Motor B
const int IN3 = 8;  // IN3 pin for Motor B
const int IN4 = 7;  // IN4 pin for Motor B

// IR Sensors
const int IRSensorL1 = A1;   // Left IR sensor
const int IRSensorL2 = A2;
const int IRSensorC = A3; // CENTRAL
const int IRSensorR1 = A4;  // Right IR sensor
const int IRSensorR2 = A5;

void setup() {
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set sensor pins as inputs
  pinMode(IRSensorL1, INPUT);
  pinMode(IRSensorL2, INPUT);
  pinMode(IRSensorC, INPUT);
  pinMode(IRSensorR1, INPUT);
  pinMode(IRSensorR2, INPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  bool leftSensor1 = digitalRead(IRSensorL1);
  bool leftSensor2 = digitalRead(IRSensorL2);
  bool centralSensor = digitalRead(IRSensorC);
  bool rightSensor1 = digitalRead(IRSensorR1);
  bool rightSensor2 = digitalRead(IRSensorR2);
  
  
  // Print sensor values
  Serial.print("Left Sensor 2: ");
  Serial.print(leftSensor2);
  Serial.print("| Left Sensor 1: ");
  Serial.print(leftSensor1);
  Serial.print("| Central Sensor: ");
  Serial.print(centralSensor);
  Serial.print(" | Right Sensor1: ");
  Serial.println(rightSensor1);
  Serial.print(" | Right Sensor2: ");
  Serial.println(rightSensor2);
  delay(500);
//white==0(low)
//black==1(high)
  // Forward movement
  if (leftSensor1 == LOW && centralSensor== HIGH && rightSensor1 == LOW) { //BLACK LINE ON THE CENTRAL SENSOR
    moveForward();
  }
  //CURVE TURNS
  // Turn right
  else if (leftSensor1 == LOW && centralSensor== LOW && rightSensor1 == HIGH) { //BLACK LINE ON THE RIGHT SENSOR
    turnRight();
  }
  // Turn left
  else if (leftSensor1 == HIGH && centralSensor== LOW && rightSensor1 == LOW) {
    turnLeft();
  }
  //90 DEG TURNS
  // Turn right
  else if (leftSensor2 == LOW && leftSensor1 == LOW && centralSensor== HIGH && rightSensor1 == HIGH && rightSensor2 == HIGH) { //BLACK LINE ON THE RIGHT 1,2 SENSORS
    turnRightEx();
  }
  // Turn left
  else if (leftSensor2 == HIGH && leftSensor1 == HIGH && centralSensor== HIGH && rightSensor1 == LOW && rightSensor2 == LOW) {
    turnLeftEx();
  }
  //FOR DIAMOND SHAPRE TRACK =🔷= L1,C,R1 HIGH GO LEFT OR RIGHT 
  //IN THE LATER PART OF GETTING OUT OF THE 🔷 TRACK ITS 135DEG SO IT WILL 1ST TURN L/R THEN TURN R/L ///SPEED SLOW AS BASE SPEED
  // Stop
  else {
    stopMotors();
  }

  delay(100); // Add a short delay to make the print readable
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, s); // Adjust speed here

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, s); // Adjust speed here
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, t); // Adjust speed here

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, t); // Adjust speed here
}
void turnRightEx() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, t); // Adjust speed here

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, t); // Adjust speed here
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, t); // Adjust speed here

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, t); // Adjust speed here
}
void turnLeftEx() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, t); // Adjust speed here

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, t); // Adjust speed here
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Stop motor

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0); // Stop motor
}



// Motor pins
const int motorLeftForward = 5;
const int motorLeftBackward = 6;
const int motorRightForward = 9;
const int motorRightBackward = 10;

// IR sensor pins
const int leftIRSensor = A0;
const int centerIRSensor = A1;
const int rightIRSensor = A2;

// Ultrasonic sensor pins
const int trigPin = 7;
const int echoPin = 8;

// Obstacle detection threshold (in cm)
const int obstacleThreshold = 20;

void setup() {
  // Motor pins setup
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);

  // Sensor pins setup
  pinMode(leftIRSensor, INPUT);
  pinMode(centerIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);
  
  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Read IR sensor values
  int leftIRValue = analogRead(leftIRSensor);
  int centerIRValue = analogRead(centerIRSensor);
  int rightIRValue = analogRead(rightIRSensor);

  // Read distance from ultrasonic sensor
  int distance = getDistance();

  // Obstacle avoidance logic
  if (distance < obstacleThreshold) {
    stopRobot();
    Serial.println("Obstacle detected, stopping.");
    delay(1000); // Stop for 1 second before deciding how to move
    avoidObstacle();
  } else {
    followLine(leftIRValue, centerIRValue, rightIRValue);
  }
}

void followLine(int left, int center, int right) {
  // Follow line based on sensor values
  if (center < 500) { // Center sensor sees the line
    moveForward();
  } else if (left < 500) { // Left sensor sees the line
    turnLeft();
  } else if (right < 500) { // Right sensor sees the line
    turnRight();
  } else { // If none see the line, stop (could be off the track)
    stopRobot();
  }
}

void moveForward() {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
}

void turnLeft() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
}

void turnRight() {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
}

void stopRobot() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, LOW);
}

void avoidObstacle() {
  // Simple logic to avoid an obstacle, e.g., turn left and move forward
  turnLeft();
  delay(1000);
  moveForward();
  delay(1000);
}

int getDistance() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time taken for the echo to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  int distance = duration * 0.034 / 2;

  return distance;
}

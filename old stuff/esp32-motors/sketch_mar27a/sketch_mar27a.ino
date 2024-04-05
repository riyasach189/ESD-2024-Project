//TODO : check the error manipulation for backward, left and right

// PIN DEFINITIONS
// ===============================================================================================

int error = 0;

const int speedControlPinLeft = 12;          // Speed and direction control pins for the motor driver
const int speedControlPinRight = 12;
const int directionPinLeft1 = 14;
const int directionPinLeft2 = 27;
const int directionPinRight1 = 14;
const int directionPinRight2 = 27;

const int analogPinLeft1 = 25;               // Analog pulse outputs from the motor encoders
const int analogPinLeft2 = 26;
const int analogPinRight1 = 25;
const int analogPinRight2 = 26;

volatile int pulseCountLeft1 = 0;       // Pulse counter variables which get periodically updated
volatile int pulseCountLeft2 = 0;       // using system interrupts
volatile int pulseCountRight1 = 0; 
volatile int pulseCountRight2 = 0; 

// ================================================================================================

void setup() {
  Serial.begin(115200);
  pinMode(speedControlPinLeft, OUTPUT);
  pinMode(speedControlPinRight, OUTPUT);
  pinMode(directionPinLeft1, OUTPUT);
  pinMode(directionPinLeft2, OUTPUT);
  pinMode(directionPinRight1, OUTPUT);
  pinMode(directionPinRight2, OUTPUT);

  pinMode(analogPinLeft1, INPUT);
  pinMode(analogPinLeft2, INPUT);
  pinMode(analogPinRight1, INPUT);
  pinMode(analogPinRight2, INPUT);

  attachInterrupt(digitalPinToInterrupt(analogPinLeft1), pulseLeft1, RISING);     // Defining the system
  attachInterrupt(digitalPinToInterrupt(analogPinLeft2), pulseLeft2, RISING);     // interrupts to periodically
  attachInterrupt(digitalPinToInterrupt(analogPinRight1), pulseRight1, RISING);   // update the pulse counters
  attachInterrupt(digitalPinToInterrupt(analogPinRight2), pulseRight2, RISING);
}

void resetPulses(){           // Function which will be called by the movement functions to reset the pulse counters
  pulseCountLeft1 = 0;
  pulseCountLeft2 = 0;
  pulseCountRight1 = 0;
  pulseCountRight2 = 0;
}

// MOVEMENT FUNCTIONS
// ============================================

void forward(){

  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, HIGH);
  digitalWrite(directionPinLeft2, LOW);
  digitalWrite(directionPinRight1, HIGH);
  digitalWrite(directionPinRight2, LOW);

  int pulseLeft = 0;
  int pulseRight = 0;
  resetPulses();
  delay(100);
  pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
  pulseRight = (pulseCountRight1+pulseCountRight2)/2;
  int currentError = pulseLeft - pulseRight;

  while(currentError > error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }

  while(currentError < -error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }
}

void backward(){

  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, LOW);
  digitalWrite(directionPinLeft2, HIGH);
  digitalWrite(directionPinRight1, LOW);
  digitalWrite(directionPinRight2, HIGH);

  int pulseLeft = 0;
  int pulseRight = 0;
  resetPulses();
  delay(100);
  pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
  pulseRight = (pulseCountRight1+pulseCountRight2)/2;
  int currentError = pulseLeft - pulseRight;

  while(currentError > error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }

  while(currentError < -error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }
}



void left(){

  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, HIGH);
  digitalWrite(directionPinLeft2, LOW);
  digitalWrite(directionPinRight1, LOW);
  digitalWrite(directionPinRight2, HIGH);

  int pulseLeft = 0;
  int pulseRight = 0;
  resetPulses();
  delay(100);
  pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
  pulseRight = (pulseCountRight1+pulseCountRight2)/2;
  int currentError = pulseLeft - pulseRight;

  while(currentError > error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }

  while(currentError < -error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }
}



void right(){

  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, LOW);
  digitalWrite(directionPinLeft2, HIGH);
  digitalWrite(directionPinRight1, HIGH);
  digitalWrite(directionPinRight2, LOW);

  int pulseLeft = 0;
  int pulseRight = 0;
  resetPulses();
  delay(100);
  pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
  pulseRight = (pulseCountRight1+pulseCountRight2)/2;
  int currentError = pulseLeft - pulseRight;

  while(currentError > error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }

  while(currentError < -error){
    currentSpeedLeft -= 10;
    currentSpeedRight -= 10;
    analogWrite(speedControlPinLeft, currentSpeedLeft);
    analogWrite(speedControlPinRight, currentSpeedRight);
    int pulseLeft = 0;
    int pulseRight = 0;
    resetPulses();
    delay(100);
    pulseLeft = (pulseCountLeft1+pulseCountLeft2)/2;
    pulseRight = (pulseCountRight1+pulseCountRight2)/2;
    currentError = pulseLeft - pulseRight;
  }
}

// ============================================
void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(analogRead(analogPinLeft1));
  forward();

  if (analogRead(analogPinLeft1) == 0)
  {
    Serial.print(0);
  }

  else 
  {
    Serial.print(1);
  }

  Serial.print(",");
  
  if (analogRead(analogPinLeft2) == 0)
  {
    Serial.print(0);
  }

  else 
  {
    Serial.print(1);
  }

  Serial.println("");
  delay(1);
}


/*
 ===============================================
| The system interrupts which will periodically  | 
| update the pulse counter variables             |
 ===============================================
*/

void pulseLeft1(){
  pulseCountLeft1++;
}

void pulseLeft2(){
  pulseCountLeft2++;
}

void pulseRight1(){
  pulseCountRight1++;
}

void pulseRight2(){
  pulseCountRight2++;
}
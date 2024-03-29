//ultrasonic
const int trigPinLeft = 22;
const int echoPinLeft = 23;
const int trigPinStraight = 18;
const int echoPinStraight = 19;
const int trigPinRight = 0;
const int echoPinRight = 4;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long durationLeft, durationStraight, durationRight;
float distanceCmLeft, distanceCmStraight, distanceCmRight;

//motors
const int speedControlPinLeft = 34;          
const int speedControlPinRight = 35;
const int directionPinLeft1 = 32;
const int directionPinLeft2 = 33;
const int directionPinRight1 = 25;
const int directionPinRight2 = 26;

//LSR logic
int LSR = 0;
// LSR is a 3 digit binary number left-straight-right
// 1 means there is a wall
// 0 means no wall

void setup() 
{
  //ultrasonic
  Serial.begin(115200); 
  pinMode(trigPinLeft, OUTPUT); 
  pinMode(echoPinLeft, INPUT); 
  pinMode(trigPinStraight, OUTPUT); 
  pinMode(echoPinStraight, INPUT);
  pinMode(trigPinRight, OUTPUT); 
  pinMode(echoPinRight, INPUT);

  //motors
  pinMode(speedControlPinLeft, OUTPUT);
  pinMode(speedControlPinRight, OUTPUT);
  pinMode(directionPinLeft1, OUTPUT);
  pinMode(directionPinLeft2, OUTPUT);
  pinMode(directionPinRight1, OUTPUT);
  pinMode(directionPinRight2, OUTPUT);
}

void forward()
{
  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, HIGH);
  digitalWrite(directionPinLeft2, LOW);
  digitalWrite(directionPinRight1, HIGH);
  digitalWrite(directionPinRight2, LOW);
}

void left()
{
  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, HIGH);
  digitalWrite(directionPinLeft2, LOW);
  digitalWrite(directionPinRight1, LOW);
  digitalWrite(directionPinRight2, HIGH);
}

void right()
{
  int currentSpeedLeft = 255;
  int currentSpeedRight = 255;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, LOW);
  digitalWrite(directionPinLeft2, HIGH);
  digitalWrite(directionPinRight1, HIGH);
  digitalWrite(directionPinRight2, LOW);
}

void turnAround()
{
  right();
  right();
}

void stop()
{
  int currentSpeedLeft = 0;
  int currentSpeedRight = 0;
  
  analogWrite(speedControlPinLeft, currentSpeedLeft);
  analogWrite(speedControlPinRight, currentSpeedRight);

  digitalWrite(directionPinLeft1, LOW);
  digitalWrite(directionPinLeft2, LOW);
  digitalWrite(directionPinRight1, LOW);
  digitalWrite(directionPinRight2, LOW);
}

void loop() 
{
  //start the bot
  forward();

  //ultrasonic
  LSR = 0;

  // Clears the trigPin
  digitalWrite(trigPinLeft, LOW);
  digitalWrite(trigPinStraight, LOW);
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationLeft = pulseIn(echoPinLeft, HIGH); 
  
  digitalWrite(trigPinStraight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinStraight, LOW);
  durationStraight = pulseIn(echoPinStraight, HIGH); 

  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
  durationRight = pulseIn(echoPinRight, HIGH); 
  
  // Calculate the distance
  distanceCmLeft = durationLeft * SOUND_SPEED/2;
  distanceCmStraight = durationStraight * SOUND_SPEED/2;
  distanceCmRight= durationRight * SOUND_SPEED/2;

  Serial.print(distanceCmLeft);
  Serial.print(",");
  Serial.print(distanceCmStraight);
  Serial.print(",");
  Serial.print(distanceCmRight);
  Serial.println();

  if (distanceCmLeft < 5)
  {
    LSR = 1;
  }

  else 
  {
    LSR = 0;
  }

  if (distanceCmStraight < 5)
  {
    LSR = (LSR * 10) + 1;
  }

  else 
  {
    LSR = (LSR * 10) + 0;
  }

  if (distanceCmRight < 5)
  {
    LSR = (LSR * 10) + 1;
  }

  else 
  {
    LSR = (LSR * 10) + 0;
  }

  //LSR logic
  switch (LSR) 
  {
    case 000:
      stop();
      break;
    case 001:
      left();
      break;
    case 010:
      left();
      break;
    case 011:
      left();
      break;
    case 100:
      forward();
      break;
    case 101:
      forward();
      break;
    case 110:
      right();
      break;
    case 111:
      turnAround();
      break;
    default:
      forward();
      break;
  }
  
  delay(500);
}

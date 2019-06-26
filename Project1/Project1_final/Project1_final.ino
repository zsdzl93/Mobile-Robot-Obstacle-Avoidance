#include <RedBot.h>
RedBotMotors motors; // define motors
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotEncoder encoder = RedBotEncoder(A2, 10); // initialize left encouder object on A2, right encounder on 10
RedBotAccel accelerometer;
#define LINETHRESHOLD 800 //Line value
#define SPEED 60  // set the nominal speed. Set to any number from 0 - 255.
int leftSpeed;   // variable used to store the leftMotor speed
int rightSpeed;  // variable used to store the rightMotor speed

int buttonPin = 12; // set buttton as 12
int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

float wheelDiam = 2.56;  // diam = 65mm / 25.4 mm/in
float wheelCirc = PI*wheelDiam;  // Redbot wheel circumference = pi*D

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP); // initialize button as input_pullup 
  Serial.begin(9600); // initialize serial
//  while (digitalRead(buttonPin) == HIGH);
//  driveDistance(50);  //drive 50 inches
}

void loop(void)
{
  // drive on button press.
  accelerometer.read(); // updates the x, y, and z axis readings on the acceleromter

  int n;
  float X=accelerometer.x;
  Serial.print( X,2 );
  Serial.print("\n");

  for( n=0;n<100;n++ )
  {
    X = X+accelerometer.x;
  }
  X=X/100;
  
  if (  X < -3000 )   //   digitalRead(buttonPin) == LOW &&
  {
    driveDistance(50);  //drive 50 inches
  }
}

void driveDistance(float distance)
{
  long stopCount = 0;
  long lCount = 0; // left count value
  long rCount = 0; // right eount value
  float numRev; //goal encoder value
  numRev = distance / wheelCirc; //goal encoude value = goal distance / wheel circumference
  encoder.clearEnc(BOTH);  // clear the encoder count
  while (lCount+rCount < 2*numRev*countsPerRev) // while average count value < goal
  {
    if(center.read() > LINETHRESHOLD) // if center sensor is above the line, go straight
    {
      leftSpeed = -SPEED; 
      rightSpeed = SPEED;
    } 
    else if(right.read() > LINETHRESHOLD) // if right sensor is above the line, turn left
    {
      leftSpeed = -( 100);
      rightSpeed =  - 100;
    }
    else if(left.read() > LINETHRESHOLD) // if left sensor is above the line, urn right
    {
      leftSpeed = -(- 100);
      rightSpeed =  100;
    }
    if((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD)) // if all sensor is above the line, stop
    {
      if(stopCount > 500) motors.stop();
      else stopCount ++;
    }
    else //else, adjust speed
    {
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed); 
      stopCount = 0;
      delay(5);
    }
    lCount = abs(encoder.getTicks(LEFT)); // get left encoder value
    rCount = abs(encoder.getTicks(RIGHT)); //get right encoder value
    Serial.print("driveDistance "); // print drived distance
    Serial.print((lCount + rCount)*wheelCirc/2/countsPerRev);
    Serial.print("\"\n"); // print drived distance
  }
  motors.brake();//stop
  Serial.print("=====================================\n");
  Serial.print("driveDistance "); // print drived distance
  Serial.print((lCount + rCount)*wheelCirc/2/countsPerRev);
  Serial.print("\"\n"); // print drived distance
  delay(1000);
}


#include <RedBot.h>
#include <math.h>
#define LED 13
#define BUTTON 12
#define LINETHRESHOLD 800
#define LEFTSPEED 60  // sets the nominal speed. Set to any number from 0 - 255.
#define RIGHTSPEED 60
#define countsPerRev 192
#define wheelDiam 65
#define wheelCirc (PI*wheelDiam)
#define delayTime 5
#define DELAYTIME 10
#define wheelDistance 160
#define OFFSET 1
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);
void get_q(float q[2], float x, float y);
void get_g(float g[3][3], float angle = 0, float x = 0, float y = 0);
void inv_g(float ginv[3][3], float g[3][3]);
void gq(float result[2], float g[3][3], float q[2]);
void driveLine(float * q);
void setup()
{
  // put your setup code here, to run once:
  int leftSpeed;   // variable used to store the leftMotor speed
  int rightSpeed;  // variable used to store the rightMotor speed
  long lCount, rCount, lDiff, rDiff, prevlCount, prevrCount;
  float lDistance, rDistance;
  float qA[2] = { 920, 920};
  float qB[2], gB1B2[3][3], gB2B1[3][3], gBA[3][3], gAB[3][3];
  float angle;
  Serial.begin(9600);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  get_g(gAB, 0);//angle, x, y
  inv_g(gBA, gAB);
  gq(qB, gBA, qA);
  while(digitalRead(BUTTON) == HIGH);  
  Serial.println(qB[0]);
  Serial.println(qB[1]);
  Serial.print("********\n");
  while(abs(qB[0])>50 || abs(qB[1])>50)//has reached?
  {
    if (qB[1]>0 && abs(atan(qB[0]/qB[1]))<0.08)// if in the front, then go straight
    {
      // go straight
      leftSpeed = -LEFTSPEED; 
      rightSpeed = RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
      encoder.clearEnc(BOTH);
      lDistance = 0;
      rDistance = 0; 
      prevlCount = 0;
      prevrCount = 0;
//      delay(delayTime);
//      lCount = encoder.getTicks(LEFT);
//      rCount = encoder.getTicks(RIGHT);
//      lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
//      rDistance = abs(float(rCount)) * wheelCirc / countsPerRev; 
//      qB[1] = sqrt(qB[1]*qB[1]+qB[0]*qB[0]) - (lDistance+rDistance)/2;
//      qB[0] = 0;

      while(left.read()<=LINETHRESHOLD && center.read()<=LINETHRESHOLD && right.read()<=LINETHRESHOLD)
      {
        delay(delayTime);
        lCount = encoder.getTicks(LEFT);
        rCount = encoder.getTicks(RIGHT);
        lCount = abs(lCount);
        rCount = abs(rCount);
        lDistance = float(lCount) * wheelCirc / countsPerRev;
        rDistance = float(rCount) * wheelCirc / countsPerRev;
        if (lDistance + rDistance >= 2*qB[1]) break;
        lDiff = (lCount - prevlCount);
        rDiff = (rCount - prevrCount);
        prevlCount = lCount;
        prevrCount = rCount;
        if (lDiff > rDiff) //has turned right, should turn left
        {
          leftSpeed += OFFSET;
          rightSpeed += OFFSET;
        }
        // if right is faster than the left, speed up the left / slow down right
        else if (lDiff < rDiff) //has turned left, should turn right
        {
          leftSpeed -= OFFSET;
          rightSpeed -= OFFSET;
        }
        motors.leftMotor(-leftSpeed);
        motors.rightMotor(-rightSpeed);
      }
      qB[1] -= (lDistance + rDistance)/2;
    }
    else
    {
      if(qB[0] > 0)//if on the right, turn right
      {
        // turn right
        if(qB[1] == 0) angle = PI/2;
        else if(qB[1]>0) angle = atan(qB[0]/qB[1]);
        else angle = PI + atan(qB[0]/(qB[1]));
        leftSpeed = -LEFTSPEED; 
        rightSpeed = -RIGHTSPEED;
        encoder.clearEnc(BOTH);
        motors.leftMotor(-leftSpeed);
        motors.rightMotor(-rightSpeed);
        do
        {
          delay(1);
          lCount = encoder.getTicks(LEFT);
          rCount = encoder.getTicks(RIGHT);
          lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
          rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;           
        }while (rDistance+lDistance < wheelDistance*angle);
      }
      else//otherwise, turn left
      {
        // turn left
        if(qB[1] == 0) angle = PI/2;
        else if(qB[1]>0) angle = atan(-qB[0]/qB[1]);
        else angle = PI + atan(-qB[0]/qB[1]);
        leftSpeed = LEFTSPEED; 
        rightSpeed = RIGHTSPEED;
        encoder.clearEnc(BOTH);
        motors.leftMotor(-leftSpeed);
        motors.rightMotor(-rightSpeed);
        do
        {
          delay(1);
          lCount = encoder.getTicks(LEFT);
          rCount = encoder.getTicks(RIGHT);
          lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
          rDistance = abs(float(rCount)) * wheelCirc / countsPerRev; 
        }while (rDistance+lDistance < wheelDistance*angle);
      }
      qB[1] = sqrt(qB[1]*qB[1]+qB[0]*qB[0]);
      qB[0] = 0;
    }
    /********** if hit an obstacle ***********/
    if( (center.read()> LINETHRESHOLD) )  //||(left.read()> LINETHRESHOLD)||(right.read()> LINETHRESHOLD)
    {
      //turn left
//      digitalWrite(LED, HIGH);
      driveLine(qB);
//      digitalWrite(LED, LOW);
    }
  }
  motors.brake();
}
void loop()
{
  // put your main code here, to run repeatedly:
}
void get_q(float q[2], float x, float y)
{
  q[0] = x;
  q[1] = y;
}
void get_g(float g[3][3], float angle, float x, float y)
{
  g[0][0] = cos(angle);
  g[0][1] = -sin(angle);
  g[0][2] = x;
  g[1][0] = sin(angle);
  g[1][1] = cos(angle);
  g[1][2] = y;
  g[2][0] = 0;
  g[2][1] = 0;
  g[2][2] = 1;
}
void inv_g(float ginv[3][3], float g[3][3])
{
  float gg[3][3];
  unsigned char i, j;
  for(i=0; i<3; i++)
  {
    for(j = 0; j<3; j++)
    {
      gg[i][j] = g[i][j];
    }
  }
  ginv[0][0] = gg[0][0];
  ginv[0][1] = gg[1][0];
  ginv[0][2] = -gg[0][0]*gg[0][2] - gg[1][0]*gg[1][2];
  ginv[1][0] = gg[0][1];
  ginv[1][1] = gg[1][1];
  ginv[1][2] = -gg[0][1]*gg[0][2] - gg[1][1]*gg[1][2];
  ginv[2][0] = 0;
  ginv[2][1] = 0;
  ginv[2][2] = 1;
}
void gq(float result[2], float g[3][3], float q[2])
{
  float qq[2] = {q[0], q[1]};
  result[0] = g[0][2] + g[0][0]*qq[0] + g[0][1]*qq[1];
  result[1] = g[1][2] + g[1][0]*qq[0] + g[1][1]*qq[1]; 
}

void driveLine(float * q)
{
  
  int hit_state_now=0,hit_state_last=0;
  int count=0;
  int leftSpeed,rightSpeed;
  long lCount, rCount;
  float lDistance, rDistance;
  float g[3][3], g1[3][3];
  float theta;

  digitalWrite(LED,HIGH);
//  Serial.println(left.read());
//  Serial.println(center.read());
//  Serial.println(right.read());
//  Serial.println(q[0]);
//  Serial.println(q[1]);
//  Serial.print("=============\n");
  /************** turn left when hit an abstacle ****************/
  // if the left sensor hit the line first, then a big left turn is needed.
  encoder.clearEnc(BOTH);
  if( left.read()>LINETHRESHOLD&&center.read()<LINETHRESHOLD )
  {

    while( count<2 )
    {

      leftSpeed = LEFTSPEED;
      rightSpeed = RIGHTSPEED;
     
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);

      if( hit_state_now>hit_state_last )
      {
        count++;
      }
      hit_state_last=hit_state_now;
      if( center.read()<LINETHRESHOLD*0.9 )
        hit_state_now=0;
      if( center.read()>LINETHRESHOLD )
        hit_state_now=1;
     // Serial.print(hit_state_now);
    }
  }

  // If enter straight in, or the right sensor hit the line first, no need for a big left turn

  leftSpeed = LEFTSPEED;
  rightSpeed = RIGHTSPEED;
  motors.leftMotor(-leftSpeed);
  motors.rightMotor(-rightSpeed);
  
  while(center.read() > LINETHRESHOLD);// 
  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);
  lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
  rDistance = abs(float(rCount)) * wheelCirc / countsPerRev; 

  get_g(g, (lDistance+rDistance)/wheelDistance);
  inv_g(g1, g);
  gq(q, g1, q);
//  Serial.println(q[0]);
//  Serial.println(q[1]);
//  Serial.print("=============\n");
//  motors.brake();
//  while(1);

  while(q[0] > 0 || q[1] < 0)
  {
    if(center.read() > LINETHRESHOLD) // if center sensor is above the line, go straight
    {
      leftSpeed = -LEFTSPEED; 
      rightSpeed = RIGHTSPEED;
    } 
    else if(left.read() > LINETHRESHOLD) // if left sensor is above the line, urn right
    {
      leftSpeed = -(LEFTSPEED - 50);
      rightSpeed = RIGHTSPEED + 50;
    }

    else if(right.read() > LINETHRESHOLD) // if right sensor is above the line, turn left
    {
      leftSpeed = -(LEFTSPEED + 50);
      rightSpeed = RIGHTSPEED - 50;
    }

    encoder.clearEnc(BOTH);     // clear encoder
    if(left.read()<LINETHRESHOLD || center.read()<LINETHRESHOLD || right.read()<LINETHRESHOLD)
    {
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed); 
    }


    delay(DELAYTIME);
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
    rDistance = abs(float(rCount)) * wheelCirc / countsPerRev; 
    theta = (rDistance - lDistance) / wheelDistance;
    get_g(g, theta, -(lDistance+rDistance)*sin(theta/2)/2, (lDistance+rDistance)*cos(theta/2)/2);
    inv_g(g1, g);
    gq(q, g1, q);
 
  }
  digitalWrite(LED,LOW);
  Serial.println(q[0]);
  Serial.println(q[1]);
  Serial.print("=============\n");
}

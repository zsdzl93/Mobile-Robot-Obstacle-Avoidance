#include <RedBot.h>
#include <math.h>
#define LED 13
#define BUTTON 12      // button to be pressed to start up
#define LINETHRESHOLD 800
#define LEFTSPEED 60  // sets the nominal speed. Set to any number from 0 - 255.
#define RIGHTSPEED 60
#define TURNSPEED 50
#define countsPerRev 192
#define wheelDiam 65
#define wheelCirc (PI*wheelDiam)
#define delayTime 5
#define DELAYTIME 10
#define wheelDistance 160
#define OFFSET 1
#define PHITTHRESHOLD 150  //permit error that might happen
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);
void get_q(float q[2], float x, float y);    // the function to get coordinates of the goal point in car frame
void get_g(float g[3][3], float angle = 0, float x = 0, float y = 0);// the function to get the g as gab
void inv_g(float ginv[3][3], float g[3][3]);// the function to get the g inverse: g1
void gq(float result[2], float g[3][3], float q[2]);// the function to get q' which equals g1*q
void driveLine(float * q); // the function to circumnavigate the obstacle in bug1
void driveDistance(float distance);// use project#1 algrithms to drive for a certain distance

void setup() {
  // put your setup code here, to run once:
  int leftSpeed;   // variable used to store the leftMotor speed
  int rightSpeed;  // variable used to store the rightMotor speed
  long lCount, rCount, lDiff, rDiff, prevlCount, prevrCount;
  float lDistance, rDistance;
  float qA[2] = { 0, 500};
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
  while(abs(qB[0])>50 || abs(qB[1])>50)// while not at the goal
  {
    if (qB[1]>0 && abs(atan(qB[0]/qB[1]))<0.08)// if the goal is almost straight ahead
    {
      // then go straight
      leftSpeed = -LEFTSPEED; 
      rightSpeed = RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
      encoder.clearEnc(BOTH);
      lDistance = 0;
      rDistance = 0; 
      prevlCount = 0;
      prevrCount = 0;

      //while not hit obstacle, go strictly straight towards the goal
      while(left.read()<=LINETHRESHOLD && center.read()<=LINETHRESHOLD && right.read()<=LINETHRESHOLD)
      {
        delay(delayTime);
        lCount = encoder.getTicks(LEFT);
        rCount = encoder.getTicks(RIGHT);
        lCount = abs(lCount);
        rCount = abs(rCount);
        lDistance = float(lCount) * wheelCirc / countsPerRev;
        rDistance = float(rCount) * wheelCirc / countsPerRev;
        if (lDistance + rDistance >= 2*qB[1]) break;//if reaches the goal, break
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
      
      qB[1] -= (lDistance + rDistance)/2;//compute the coordinates of the goal q in car frame
    }
    else
    {
      if(qB[0] > 0)//if the goal is on the right, then turn right
      {
        //compute the directon of the goal
        if(qB[1] == 0) angle = PI/2;
        else if(qB[1]>0) angle = atan(qB[0]/qB[1]);
        else angle = PI + atan(qB[0]/(qB[1]));
        // turn right until the car face towards the goal point q
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
      else  //otherwise, turn left
      {
        //compute the directon of the goal
        if(qB[1] == 0) angle = PI/2;
        else if(qB[1]>0) angle = atan(-qB[0]/qB[1]);
        else angle = PI + atan(-qB[0]/qB[1]);
        // turn left until the car face towards the goal point q
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

      //compute the goal point coordinates expressed in car frame
      qB[1] = sqrt(qB[1]*qB[1]+qB[0]*qB[0]);
      qB[0] = 0;
    }
    /********** if hit an obstacle ***********/
    if( (center.read()> LINETHRESHOLD) )
    {
      //turn left
      digitalWrite(LED, HIGH);
      driveLine(qB);
      digitalWrite(LED, LOW);
    }
  }
  motors.brake();//if reaches the goal, brake.
}


void loop() {
  // put your main code here, to run repeatedly:
}

/*********** the specific function of get_q get_g inv_g gq ************/
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

/*********************** driveLine ************************/
void driveLine(float*q)
{
  int hit_state_now=0,hit_state_last=0;
  int count0=0;
  int leftSpeed,rightSpeed;
  long lCount, rCount;
  float lDistance, rDistance;
  float g[3][3], g1[3][3];
  float theta;
  float S,S1;
  float Dmin;
  float Phit[2]={ 0,0 };//hit point
  int count=0;
  /************** turn left when hit an abstacle ****************/
  //if the left sensor hits the obstacle first, then the car needs a big left turn
  encoder.clearEnc(BOTH);
  if( left.read()>LINETHRESHOLD&&center.read()<LINETHRESHOLD )//if the left sensor hits the obstacle first
  {
    while( count0<2 )//if the center sensor hits obstacle twice, then the big left turn completes, break
    {
      leftSpeed = LEFTSPEED;
      rightSpeed = RIGHTSPEED;
     
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);

      //hit_state from 0 to 1, means hit the obstacle once.
      if( hit_state_now>hit_state_last )
      {
        count0++;
      }
      hit_state_last=hit_state_now;
      if( center.read()<LINETHRESHOLD*0.9 )
        hit_state_now=0;
      if( center.read()>LINETHRESHOLD )
        hit_state_now=1;
    }
  }
  
  //if hit the obstacle in a proper direction (doesn't need a big left turn)
  //then just turn left slightly
  leftSpeed = LEFTSPEED;
  rightSpeed = RIGHTSPEED;
  motors.leftMotor(-leftSpeed);
  motors.rightMotor(-rightSpeed);
  while(center.read() > LINETHRESHOLD);

  //After car adjusts the position, compute the goal point in car frame.
  //Phit won't change because of the pure rotation. Here just transform q
  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);
  lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
  rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;
  get_g(g, (lDistance+rDistance)/wheelDistance);
  inv_g(g1, g);
  gq(q, g1, q);

  // Before circumnavigating the obstacle, initialize the S and Dmin
  S=0;//the distance that the car has driven
  Dmin=sqrt(q[1]*q[1]+q[0]*q[0]);//the minimum distance to the goal

  //then start circumnavigating the obstacle
  /* Because the car start from the Phit(hit point), it can't return to the Phit in a short time.
     We set "count" to avoid the car reaching the Phit(hit point) immediately by mistake */
  while( count<300 || (abs(Phit[0])>PHITTHRESHOLD || abs(Phit[1])>PHITTHRESHOLD) )// while not return to the hit point
  {
    // Tracking
    if(center.read() > LINETHRESHOLD) // if center sensor is above the line, go straight
    {
      leftSpeed = -LEFTSPEED; 
      rightSpeed = RIGHTSPEED;
    }
    else if(left.read() > LINETHRESHOLD) // if left sensor is above the line, urn right
    {
      leftSpeed = -(LEFTSPEED - TURNSPEED);
      rightSpeed = RIGHTSPEED + TURNSPEED;
    }
    else if(right.read() > LINETHRESHOLD) // if right sensor is above the line, turn left
    {
      leftSpeed = -(LEFTSPEED + TURNSPEED);
      rightSpeed = RIGHTSPEED - TURNSPEED;
    }

    encoder.clearEnc(BOTH);     //clear the encoder data
    if(left.read()<LINETHRESHOLD || center.read()<LINETHRESHOLD || right.read()<LINETHRESHOLD)
    {
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed); 
    }

    //compute the coordinates of the goal point q
    delay(DELAYTIME);
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
    rDistance = abs(float(rCount)) * wheelCirc / countsPerRev; 
    theta = (rDistance - lDistance) / wheelDistance;

    get_g(g, theta, -(lDistance+rDistance)*sin(theta/2)/2, (lDistance+rDistance)*cos(theta/2)/2); 
    inv_g(g1, g);
    gq(q, g1, q);
    //then compute the coordinates of Phit(hit point)
    gq(Phit, g1, Phit);

    //update the s first, then see if the Dmin should be updated as well as S1
    S= S+(lDistance+rDistance)/2;
    if( sqrt(q[1]*q[1]+q[0]*q[0])<Dmin )
    {
      Dmin=sqrt(q[1]*q[1]+q[0]*q[0]);
      S1=S;
    }

    
    /* Because the car start from the Phit(hit point), it can't return to the Phit in a short time.
       We set "count" to avoid the car reaching the Phit(hit point) immediately by mistake */
    if(count<500)
      count++;
  }


  encoder.clearEnc(BOTH);     //clear the encoder data
  //if the distance in current direction to the "P leave" is greater than half of obstacle primeter, 
  //then move to the gaol in inverse direction for (S-S1)
  if(2*S1>S)
  {
    //turn by 180°
    while( ((lDistance+rDistance)/wheelDistance)<PI )
    {
      leftSpeed = LEFTSPEED;
      rightSpeed = RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
      lCount = encoder.getTicks(LEFT);
      rCount = encoder.getTicks(RIGHT);
      lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
      rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;
    }
    // circumnavigate the obstacle for (S-S1)
    driveDistance(S-S1);
    //when arrives the "P leave", move towards the goal
    encoder.clearEnc(BOTH);     //clear the encoder data
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
    rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;
    //turn
    while( ((lDistance+rDistance)/wheelDistance)<(PI/2) )
    {
      leftSpeed = LEFTSPEED;
      rightSpeed = RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed); 
    }
    
    encoder.clearEnc(BOTH);     //clear the encoder data
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
    rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;
    //move towards
    while( (lDistance+rDistance)<(Dmin*2) )
    {
      leftSpeed = -LEFTSPEED;
      rightSpeed = RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
    }
    
  }
  else  // Otherwise, move to the goal by circumnavigating the obstacle in the current deriction
  {
    //circumnavigate the obstacle for S1
    driveDistance(S1);
    //when arrives the "P leave", move towards the goal
    encoder.clearEnc(BOTH);     //clear the encoder data
    while( ((lDistance+rDistance)/wheelDistance)<(PI/2) )
    {
      leftSpeed = -LEFTSPEED;
      rightSpeed = -RIGHTSPEED;
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed);
      lCount = encoder.getTicks(LEFT);
      rCount = encoder.getTicks(RIGHT);
      lDistance = abs(float(lCount)) * wheelCirc / countsPerRev;
      rDistance = abs(float(rCount)) * wheelCirc / countsPerRev;
    }
  }
}

/***************** project1 circumnavigate the obstacle for a certain distance ****************/
void driveDistance(float distance)
{
  int leftSpeed,rightSpeed;
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
      leftSpeed = -LEFTSPEED; 
      rightSpeed = RIGHTSPEED;
    } 
        else if(left.read() > LINETHRESHOLD) // if left sensor is above the line, urn right
    {
      leftSpeed = -(LEFTSPEED - TURNSPEED);
      rightSpeed = RIGHTSPEED + TURNSPEED;
    }

    else if(right.read() > LINETHRESHOLD) // if right sensor is above the line, turn left
    {
      leftSpeed = -(LEFTSPEED + TURNSPEED);
      rightSpeed = RIGHTSPEED - TURNSPEED;
    }

    if(left.read()<LINETHRESHOLD || center.read()<LINETHRESHOLD || right.read()<LINETHRESHOLD)
    {
      motors.leftMotor(-leftSpeed);
      motors.rightMotor(-rightSpeed); 
    }
    
    lCount = abs(encoder.getTicks(LEFT)); // get left encoder value
    rCount = abs(encoder.getTicks(RIGHT)); //get right encoder value

  }

}






















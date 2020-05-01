/*
BalanceCar.h BalanceCar.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/

#include "BalanceCar.h"

float BalanceCar::turnspin(bool turnleftflag,bool turnrightflag,bool spinleftflag,bool spinrightflag, int speeds)
{
  int spinonce = 0;
  float turnspeed = 0;
	float rotationratio = 0;
	
  if (turnleftflag || turnrightflag || spinleftflag || spinrightflag)
  {
    if (spinonce == 0)
    {
      turnspeed = float(speeds);
	  spinonce++;
    }

    if (turnspeed < 0)
    {
      turnspeed = -turnspeed;

    }
    if(turnleftflag || turnrightflag)
    {
     turnmax=3;
     turnmin=-3;
    }
    if(spinleftflag||spinrightflag)
    {
      turnmax=10;
      turnmin=-10;
    }
    rotationratio = 5 / turnspeed;
    if (rotationratio < 0.5)rotationratio = 0.5;
    if (rotationratio > 5)rotationratio = 5;
  }
  else
  {
    rotationratio = 0.5;
    spinonce = 0;
    turnspeed = 0;
  }
  if (turnleftflag || spinleftflag)
  {
    turnout += rotationratio;
  }
  else if (turnrightflag || spinrightflag)
  {
    turnout -= rotationratio;
  }
  else turnout = 0;
  if (turnout > turnmax) turnout = turnmax;
  if (turnout < turnmin) turnout = turnmin;

	return turnout;
}

void BalanceCar::pwma(double speedoutput,float rotationoutput,float angle,float angle6,bool const turnleftflag, bool const turnrightflag, bool const spinleftflag, bool const spinrightflag,
	int const direction,float accelz,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB)
{

  pwm1 = -angleoutput - speedoutput - rotationoutput; //Left PWM
  pwm2 = -angleoutput - speedoutput + rotationoutput;//Right PWM

  if (pwm1 > 255) pwm1 = 255;
  if (pwm1 < -255) pwm1 = -255;
  if (pwm2 > 255) pwm2 = 255;
  if (pwm2 < -255) pwm2 = -255;

  if (angle > 30 || angle < -30)
  {
    pwm1 = 0;
    pwm2 = 0;
  }

		  if (angle6 > 10 || angle6 < -10 && !turnleftflag && !turnrightflag && !spinleftflag && !spinrightflag && direction == 0)
  {
           if(stopl + stopr > 1500||stopl + stopr < -3500)
   {
    pwm1 = 0;
    pwm2 = 0;
	flag1=1;

	}
  }
  else 
  {
   stopl=stopr=0;
   flag1=0;

  }

if (pwm1 >= 0) {
  digitalWrite(Pin2, 0);
  digitalWrite(Pin1, 1);
  analogWrite(PinPWMA, pwm1);
} else {
  digitalWrite(Pin2, 1);
  digitalWrite(Pin1, 0);
  analogWrite(PinPWMA, -pwm1);
}

if (pwm2 >= 0) {
  digitalWrite(Pin4, 0);
  digitalWrite(Pin3, 1);
  analogWrite(PinPWMB, pwm2);
} else {
  digitalWrite(Pin4, 1);
  digitalWrite(Pin3, 0);
  analogWrite(PinPWMB, -pwm2);
}

}
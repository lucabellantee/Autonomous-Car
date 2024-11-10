#include "servo_motor.h"



void servo_motor(int angolo)
{
	float tic;
	tic = 0.02;
	  float ccr;
	  int conv_angolo;

	  if(angolo < MIN_ANGOLO)

		   angolo = MIN_ANGOLO;

	  else if (angolo > MAX_ANGOLO)

		   angolo = MAX_ANGOLO;

	  conv_angolo = angolo + 92;

	  ccr=(((0.01111*conv_angolo)+0.5)/tic);
	  TIM1->CCR1=ccr;
}

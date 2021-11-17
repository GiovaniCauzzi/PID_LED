
int analog = A0;
int setpoint_pin= A1;
int ledPin = 9;
int out = 0;

#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

/* Controller parameters */
#define PID_KP  0.50f
#define PID_KI  2.0f
#define PID_KD  0.5000f

#define PID_TAU 0.02f

#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX 255.0f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT  255.0f

#define n 20
int VetorMediaMovel[n];

//#define SAMPLE_TIME_S 0.01f



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  double tempauxatual=0,tempauxanterior=0;
  int atual = 0;
  float setpoint = 20, TempoAtual = 0, TempoAnterior = 0,PotenciaLED=0;
  float SAMPLE_TIME_S = 0.1;

  PIDController pid = { PID_KP, PID_KI, PID_KD,
                        PID_TAU,
                        PID_LIM_MIN, PID_LIM_MAX,
                        PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                        SAMPLE_TIME_S
                      };

  PIDController_Init(&pid);



  while (1) {
    atual = -0.32*analogRead(analog)+336.8;
    atual = MediaMovel(atual);

    TempoAtual = micros();
    SAMPLE_TIME_S = (TempoAtual - TempoAnterior) / 1000;
    TempoAnterior = TempoAtual;

    PotenciaLED = PIDController_Update(&pid, setpoint, atual);
    analogWrite(ledPin, PotenciaLED);

    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(atual);
    Serial.print(",");
    Serial.println((PotenciaLED*100)/255);
  



   /*tempauxatual=millis();
   if(tempauxatual-tempauxanterior>3000){
    setpoint=setpoint+10;
    tempauxanterior=tempauxatual; 
   }

  
   if(setpoint>100){
    setpoint=0;
    }*/
    //setpoint=70;
    //setpoint=(analogRead(setpoint_pin)*100)/1024;
    setpoint=analogRead(setpoint_pin);
    setpoint=map(setpoint,0,1023,0,100);
  }
}

long MediaMovel(int UltimaLeitura)
{
  //desloca os elementos do vetor de média móvel
  for (int i = n - 1; i > 0; i--) VetorMediaMovel[i] = VetorMediaMovel[i - 1];

  VetorMediaMovel[0] = UltimaLeitura; //posição inicial do vetor recebe a leitura original

  long somador = 0;          //acumulador para somar os pontos da média móvel

  for (int i = 0; i < n; i++) somador += VetorMediaMovel[i]; //faz a somatória do número de pontos


  return somador / n; //retorna a média móvel
} //end moving_average


float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
  /*
    Error signal
  */
  float error = setpoint - measurement;
  /*
    Proportional
  */
  float proportional = pid->Kp * error;
  /*
    Integral
  */
  pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

  /* Anti-wind-up via integrator clamping */
  if (pid->integrator > pid->limMaxInt) {
    pid->integrator = pid->limMaxInt;
  } else if (pid->integrator < pid->limMinInt) {
    pid->integrator = pid->limMinInt;
  }
  /*
    Derivative (band-limited differentiator)
  */
  pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

  /*
    Compute output and apply limits
  */
  pid->out = proportional + pid->integrator + pid->differentiator;
  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }

  /* Store error and measurement for later use */
  pid->prevError       = error;
  pid->prevMeasurement = measurement;

  /* Return controller output */
  return pid->out;
}

void PIDController_Init(PIDController *pid) {

  /* Clear controller variables */
  pid->integrator = 0.0f;
  pid->prevError  = 0.0f;

  pid->differentiator  = 0.0f;
  pid->prevMeasurement = 0.0f;

  pid->out = 0.0f;

}

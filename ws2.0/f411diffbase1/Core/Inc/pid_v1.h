/*
 * pid_v1.h
 *
 *  Created on: Feb 5, 2026
 *      Author: inaciose
 */

#ifndef INC_PID_V1_H_
#define INC_PID_V1_H_

#include <stdint.h>

/* Constants */
#define AUTOMATIC  1
#define MANUAL     0
#define DIRECT     0
#define REVERSE    1
#define P_ON_M     0
#define P_ON_E     1

typedef struct
{
    /* User tunings */
    double dispKp;
    double dispKi;
    double dispKd;

    double kp;
    double ki;
    double kd;

    int controllerDirection;
    int pOn;
    uint8_t pOnE;

    double *myInput;
    double *myOutput;
    double *mySetpoint;

    uint32_t lastTime;
    double outputSum;
    double lastInput;

    uint32_t SampleTime;
    double outMin;
    double outMax;

    uint8_t inAuto;

} PID_t;

/* API */
void PID_Init(PID_t *pid,
              double *Input, double *Output, double *Setpoint,
              double Kp, double Ki, double Kd,
              int POn, int ControllerDirection);

void PID_InitSimple(PID_t *pid,
                    double *Input, double *Output, double *Setpoint,
                    double Kp, double Ki, double Kd,
                    int ControllerDirection);

uint8_t PID_Compute(PID_t *pid);

void PID_SetTunings(PID_t *pid, double Kp, double Ki, double Kd, int POn);
void PID_SetTuningsSimple(PID_t *pid, double Kp, double Ki, double Kd);

void PID_SetSampleTime(PID_t *pid, int NewSampleTime);
void PID_SetOutputLimits(PID_t *pid, double Min, double Max);
void PID_SetMode(PID_t *pid, int Mode);
void PID_SetControllerDirection(PID_t *pid, int Direction);

/* Getters */
double PID_GetKp(PID_t *pid);
double PID_GetKi(PID_t *pid);
double PID_GetKd(PID_t *pid);
int    PID_GetMode(PID_t *pid);
int    PID_GetDirection(PID_t *pid);


#endif /* INC_PID_V1_H_ */

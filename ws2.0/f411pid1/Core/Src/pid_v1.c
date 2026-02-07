/*
 * pid_v1.c
 *
 *  Created on: Feb 5, 2026
 *      Author: inaciose
 */


#include "pid_v1.h"
#include "stm32f4xx_hal.h" // adjust to mcu

static void PID_Initialize(PID_t *pid);

void PID_Init(PID_t *pid,
              double *Input, double *Output, double *Setpoint,
              double Kp, double Ki, double Kd,
              int POn, int ControllerDirection)
{
    pid->myInput = Input;
    pid->myOutput = Output;
    pid->mySetpoint = Setpoint;
    pid->inAuto = 0;

    PID_SetOutputLimits(pid, 0, 255);
    pid->SampleTime = 100;

    PID_SetControllerDirection(pid, ControllerDirection);
    PID_SetTunings(pid, Kp, Ki, Kd, POn);

    pid->lastTime = HAL_GetTick() - pid->SampleTime;
}

void PID_InitSimple(PID_t *pid,
                    double *Input, double *Output, double *Setpoint,
                    double Kp, double Ki, double Kd,
                    int ControllerDirection)
{
    PID_Init(pid, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection);
}

uint8_t PID_Compute(PID_t *pid)
{
    if(!pid->inAuto) return 0;

    uint32_t now = HAL_GetTick();
    uint32_t timeChange = (now - pid->lastTime);

    if(timeChange >= pid->SampleTime)
    {
        double input = *(pid->myInput);
        double error = *(pid->mySetpoint) - input;
        double dInput = (input - pid->lastInput);

        pid->outputSum += (pid->ki * error);

        if(!pid->pOnE)
            pid->outputSum -= pid->kp * dInput;

        if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;

        double output;
        if(pid->pOnE) output = pid->kp * error;
        else output = 0;

        output += pid->outputSum - pid->kd * dInput;

        if(output > pid->outMax) output = pid->outMax;
        else if(output < pid->outMin) output = pid->outMin;

        *(pid->myOutput) = output;

        pid->lastInput = input;
        pid->lastTime = now;

        return 1;
    }
    return 0;
}

void PID_SetTunings(PID_t *pid, double Kp, double Ki, double Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    pid->pOn = POn;
    pid->pOnE = (POn == P_ON_E);

    pid->dispKp = Kp;
    pid->dispKi = Ki;
    pid->dispKd = Kd;

    double SampleTimeInSec = ((double)pid->SampleTime) / 1000.0;
    pid->kp = Kp;
    pid->ki = Ki * SampleTimeInSec;
    pid->kd = Kd / SampleTimeInSec;

    if(pid->controllerDirection == REVERSE)
    {
        pid->kp = -pid->kp;
        pid->ki = -pid->ki;
        pid->kd = -pid->kd;
    }
}

void PID_SetTuningsSimple(PID_t *pid, double Kp, double Ki, double Kd)
{
    PID_SetTunings(pid, Kp, Ki, Kd, pid->pOn);
}

void PID_SetSampleTime(PID_t *pid, int NewSampleTime)
{
    if(NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)pid->SampleTime;
        pid->ki *= ratio;
        pid->kd /= ratio;
        pid->SampleTime = (uint32_t)NewSampleTime;
    }
}

void PID_SetOutputLimits(PID_t *pid, double Min, double Max)
{
    if(Min >= Max) return;

    pid->outMin = Min;
    pid->outMax = Max;

    if(pid->inAuto)
    {
        if(*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
        else if(*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;

        if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
    }
}

void PID_SetMode(PID_t *pid, int Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto && !pid->inAuto)
    {
        PID_Initialize(pid);
    }
    pid->inAuto = newAuto;
}

static void PID_Initialize(PID_t *pid)
{
    pid->outputSum = *(pid->myOutput);
    pid->lastInput = *(pid->myInput);

    if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
    else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
}

void PID_SetControllerDirection(PID_t *pid, int Direction)
{
    if(pid->inAuto && Direction != pid->controllerDirection)
    {
        pid->kp = -pid->kp;
        pid->ki = -pid->ki;
        pid->kd = -pid->kd;
    }
    pid->controllerDirection = Direction;
}

/* Getters */
double PID_GetKp(PID_t *pid){ return pid->dispKp; }
double PID_GetKi(PID_t *pid){ return pid->dispKi; }
double PID_GetKd(PID_t *pid){ return pid->dispKd; }
int    PID_GetMode(PID_t *pid){ return pid->inAuto ? AUTOMATIC : MANUAL; }
int    PID_GetDirection(PID_t *pid){ return pid->controllerDirection; }

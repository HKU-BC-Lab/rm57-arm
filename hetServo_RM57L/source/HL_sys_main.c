/** @file HL_sys_main.c
*   @brief Application main file
*   @date 28.Aug.2015
*   @version 04.05.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/*
* Copyright (C) 2009-2015 Texas Instruments Incorporated - www.ti.com
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */

#include "HL_het.h"
#include "HL_ecap.h"

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

/*----------------------------------------------------------------------------*/
/* Choose timer */
#define PMU_Cycle true

/* Variables for Timing */
volatile unsigned int loop_count_prep, loop_count_prep_max=1;
volatile unsigned int loop_count, loop_count_max=1;

#if PMU_Cycle
/* Performance Measurement Unit */
#include "HL_sys_pmu.h"
#endif //PMU_Cycle


/*----------------------------------------------------------------------------*/
/* Macros */
#define ECAP2 true

#define DEBUG false
#define TESTSPEED false

#define f_HCLK (float) 330.0 //f in [MHz], HCLK (depends on device setup)
#define PI (double) 3.1415926535897932
#define FACTOR (float) 10.0 //0.01


/*----------------------------------------------------------------------------*/
/* Data types*/

typedef struct pwmSig
{
	double duty;   /**< Duty cycle in % of the period  */
	double period; /**< Period in [us]                   */
} pwmSIG_t;


/*----------------------------------------------------------------------------*/
/* Global Variables */

/* Variables for Devices */
static const uint32 s_het1pwmPolarity[8U] = {3U, 3U, 3U, 3U, 3U, 3U, 3U, 3U};

/* Variables for main function */
float currentPos[6]; //current Cartesian position after interpolation
uint16 form = 1;     //choosing interpolation method. 1 stands for linear, and 2 stands for cubic.
uint16 curIndex = 0; //current start frame index
float curtime, tElapsed, reltime, duration; //in [ms]

/* Designated time breaks and path points at corresp. time points */
float via_breaks[11] = {0*FACTOR, 12*FACTOR, 24*FACTOR, 36*FACTOR, 48*FACTOR, 50*FACTOR, 62*FACTOR, 74*FACTOR, 86*FACTOR, 98*FACTOR, 120*FACTOR};
//float via_breaks[11] = {0, 12, 24, 36, 48, 50, 62, 74, 86, 98, 120};
float via_frames[6][11] =
{
    {0.14  , 0.140 , 0.1400 , 0.1400 , 0.1400 , 0.1400 , 0.1400 , 0.1400 , 0.1400 , 0.1400 , 0.1400   } ,
    {0     , 0.11  , 0.11   , 0      , -0.11  , -0.11  , 0      , 0.11   , 0.11   , 0      , -0.11000 } ,
    {0.1   , 0.1   , 0.15   , 0.15   , 0.15   , 0.1    , 0.1    , 0.1    , 0.15   , 0.15   , 0.15     } ,
    {0     , 0     , 0      , 0      , 0      , 0      , 0      , 0      , 0      , 0      , 0        } ,
    //{-PI/2 , -PI/2 , -PI/2  , -PI/2  , -PI/2  , -PI/2  , -PI/2  , -PI/2  , -PI/2  , -PI/2  , -PI/2    } ,
    {PI/2  , PI/2  , PI/2   , PI/2   , PI/2   , PI/2   , PI/2   , PI/2   , PI/2   , PI/2   , PI/2     } ,
    {0     , 0     , 0      , 0      , 0      , 0      , 0      , 0      , 0      , 0      , 0        }
};


/*----------------------------------------------------------------------------*/
/* Functions */

void genServoPwm(pwmSIG_t * sig, const double angle);
void getCurrentPos(double * pos, double * rpy, const int form);
void rotmat2rpy(double * rpy, double * drpy, const double * m, const double * dm);
void setPwmPulse(hetRAMBASE_t * hetRAM, const uint32 pwm, const pwmSIG_t * sig);
void setJoints(hetRAMBASE_t * hetRAM, const double * q, const int n);
void solveJoints0(double * q, const double * pos, const double * rpy);

float tic(void);
float toc(const float timerVal);


/*----------------------------------------------------------------------------*/

/* USER CODE END */

void main(void)
{
/* USER CODE BEGIN (3) */

	/* Initialize HET */
	hetInit();

#if ECAP2
	/* Configure ECAP2 (Software Oscilloscope) */
	ecapSetCaptureEvent1(ecapREG2, RISING_EDGE, RESET_DISABLE);
	ecapSetCaptureEvent2(ecapREG2, FALLING_EDGE, RESET_DISABLE);
	ecapSetCaptureEvent3(ecapREG2, RISING_EDGE, RESET_ENABLE);
	ecapSetCaptureMode(ecapREG2, CONTINUOUS, CAPTURE_EVENT3);
	ecapStartCounter(ecapREG2);
	ecapEnableCapture(ecapREG2);
	ecapEnableInterrupt(ecapREG2, ecapInt_CEVT3);
#endif

	/* Intialize Servo positions to neutral */
	double q[6] = {PI/2., PI/2., PI/2., PI/2., PI/2., PI/2.};
	//double q[6] = {PI/1., PI/1., PI/1., PI/1., PI/1., PI/1.};
    //int n = sizeof(q)/sizeof(*q);
    int n = 6;
    printf("\n");
    setJoints(hetRAM1, q, n);

    /* Steer Roboarm to draw a frame */
    //double *pos, *rpy;
    //pos = (double *) malloc(3*sizeof(double));
    //rpy = (double *) malloc(3*sizeof(double));
    double pos[3]; // = {0.0, 0.0, 0.0};
    double rpy[3]; // = {0.0, 0.0, 0.0};

    /* Interpolation method */
	form = 1; //linear

    /* Duration between succeeding via_frames */
    duration = via_breaks[1]-via_breaks[0];

    /* Start timing */
#if PMU_Cycle
    /* Configure PMU */
    _pmuInit_();
    _pmuEnableCountersGlobal_(); //enable all PMU counters
    _pmuResetCounters_(); //reset the cycle counter and all event counters

    /* Configure event counters */
    _pmuSetCountEvent_(pmuCOUNTER0, PMU_CYCLE_COUNT); //select CPU cycles (Event 0x11) to be counted
#endif //PMU_Cycle

	/* Run forever */
	while (true)
    {
        /* Get t */
#if PMU_Cycle
        curtime = tic();
        printf("# Current time is %fus\n", curtime);
#endif //PMU_Cycle

        if (curIndex < 11 && via_breaks[curIndex+1] <= curtime)
        {
            curIndex = curIndex+1;
            duration = via_breaks[curIndex+1] - via_breaks[curIndex];
        }

        getCurrentPos(pos, rpy, form);

#if DEBUG
        printf("%-20s%-20s%-20s%-20s%-20s%-20s\n",
                "pos[x]", "pos[y]", "pos[z]", "roll", "pitch", "yaw");
        /*printf("%-20.2f%-20.2f%-20.2f%-20.2f%-20.2f%-20.2f\n", , curIndex, duration);*/
        int i;
        for (i = 0; i < 3; ++i) { printf("%-20.2f", *(pos+i)); }
        for (i = 0; i < 3; ++i) { printf("%-20.2f", *(rpy+i)); }
        printf("\n");
#endif //DEBUG


#if TESTSPEED

#if PMU_Cycle
        curtime = tic();
#endif //PMU_Cycle

        for(loop_count = 0; loop_count < loop_count_max; ++loop_count)
        {
            solveJoints0(q, pos, rpy);
        }

#if PMU_Cycle
        tElapsed = toc(curtime);
#endif //PMU_Cycle

#else
        solveJoints0(q, pos, rpy);
#endif //TESTSPEED


        setJoints(hetRAM1, q, 6);

    }

/* USER CODE END */
}

/* USER CODE BEGIN (4) */

/** @fn void genServoPwm(pwmSIG_t * sig, const double angle)
*   @brief Set period
*   @param[in] sig Pointer to pwmSIG_t
*              - period period in [us].
*              - duty cycle in %.
*   @param[in] angle double
*              - 0 to PI
*
*   Generates PWM pulse for Servo
*/
void genServoPwm(pwmSIG_t * sig, const double angle)
{
    double period = 20000.;

    /* Linear map: angle (0 - pi) to pulse width in microeconds (1000us - 2000us) */
    double pulseWidth = angle*180./PI * (2000.-1000.)/180. + 1000.; //in [us]
    double duty = pulseWidth*100./period; //in %

#if DEBUG
    printf("%-20.2f%-20.2f%-20.2f\n", period/1000., pulseWidth/1000., duty);
#endif

    /* Set PWM signal */
    sig->period = period; // in [us]
    sig->duty   = duty;   // in %

}


/** @fn void getCurrentPos(double * pos, double * rpy, const int form)
*   @brief Get position of end-effector
*   @param[in] form, Integer indicator of the form of frame
*
*   Computes current position of end-effector
*/
void getCurrentPos(double * pos, double * rpy, const int form)
{
    int i;
    double a0,a2,a3; //a1,

    reltime = curtime - via_breaks[curIndex];
#if DEBUG
    printf("%-20s%-20s%-20s\n", "reltime [ms]", "curIndex", "duration [ms]");
    printf("%-20.2f%-20d%-20.2f\n", reltime, curIndex, duration);
#endif

    switch (form)
    {
    case 1:
        for (i = 0; i < 6; i++)
        {
            currentPos[i] = (via_breaks[curIndex+1] - curtime) /
                duration*via_frames[i][curIndex]
                + reltime / duration*via_frames[i][curIndex+1];
        }
        break;
    case 2:
        for (i = 0; i < 6; i++)
        {
            a0 = via_frames[i][curIndex];
            //a1 = 0;
            a2 = 3.0/(duration*duration)*(via_frames[i][curIndex+1]
                    - via_frames[i][curIndex]);
            a3 = -2.0/(duration*duration*duration)*(via_frames[i][curIndex+1]
                    - via_frames[i][curIndex]);
            currentPos[i] = a3*reltime*reltime*reltime + a2*reltime*reltime + a0;
#if DEBUG
            printf("# currentPos[%d] = %f\n", i, currentPos[i]);
#endif
        }
        break;
    }

    for (i = 0; i < 3; ++i)
    {
        *(pos+i) = currentPos[i];
    }

    for (i = 0; i < 3; ++i)
    {
        *(rpy+i) = currentPos[3+i];
    }

}


/** @fn void rotmat2rpy(double * rpy, double * drpy, const double * m, const double * dm)
*   @brief Converts rotation matrix to rpy
*   @param[in] m Pointer to rotation matrix
*   @param[in] dm Pointer to gradient of rotation matrix
*   @param[in] rpy, Pointer to rpy
*   @param[in] drpy, Pointer to gradient of rpy
*
*   Converts rotation matrix to rpy
*/
void rotmat2rpy(double * rpy, double * drpy, const double * m, const double * dm)
{
    bool compute_gradient;
    int rows = 3;

    if (m == NULL)
    {
        printf("# Rotation matrix is not declared!\n");
        exit(1);
    }

    if (rpy == NULL)
    {
        printf("# The rpy is not declared!\n");
        exit(1);
    }

    if (dm == NULL)
    {
        compute_gradient = false;
        if (drpy == NULL)
        {
            printf("# Gradient of rpy is not declared!\n");
            exit(1);
        }
    }
    else
    {
        compute_gradient = true;
    }

    /* Compute rpy and drpy */
    if (compute_gradient)
    {
        dm = NULL;
        printf("# NOT IMPLEMENTED!\n");
        exit(1);
    }

    *rpy = atan2(m[2+1*rows], m[2+2*rows]);
    *(rpy+1) = atan2(-m[2+0*rows],
            sqrt(m[2+1*rows]*m[2+1*rows] + m[2+2*rows]*m[2+2*rows]));
    *(rpy+2) = atan2(m[1+0*rows], m[0+0*rows]);

}


/** @fn void setJoints(hetRAMBASE_t * hetRAM, const double * q, const int n)
*   @brief Steers joints to speicified generalized coordinates
*   @param[in] hetRAM Pointer to HET RAM
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] q, Pointer to generalized coordinates
*   @param[in] n Number of Degree of Freedoms
*
*   Steers joints to speicified generalized coordinates
*/
void setJoints(hetRAMBASE_t * hetRAM, const double * q, const int n)
{
    if (q == NULL)
    {
        printf("# No degree of freedom is defined!\n");
        exit(1);
    }

    if (n > 8)
    {
        printf("# The Number of DOFs is greater than available PWM registers of one HET RAM!\n");
        exit(1);
    }

#if DEBUG
    printf("%-20s%-20s%-20s\n", "period [ms]", "pulseWidth [ms]", "duty (%)");
#endif
    pwmSIG_t *sig;
    sig = (pwmSIG_t *) malloc(sizeof(pwmSIG_t));
    int i;
    for (i = 0; i < n; ++i)
    {
#if DEBUG
        printf("# Set q[%d]:=%.2f\n", i, *(q+i));
#endif
        double val = *(q + i);
        if (val <= PI/2. && val >= -PI/2.)
        {
            genServoPwm(sig, *(q + i));
            setPwmPulse(hetRAM, (uint32)(pwm0+i), sig);
        }
        else
        {
            printf("# %d.th DOF (%.2f in degree) is out of range.\n", i, val*180./PI);
        }
    }

}


/** @fn void setPwmPulse(hetRAMBASE_t * hetRAM, const uint32 pwm, const pwmSIG_t * sig)
*   @brief Sets a new pwm signal
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] sig Pointer to pwmSIG_t
*              - duty cycle in %.
*              - period period in [us].
*
*   Sets a new pwm signal
*/
void setPwmPulse(hetRAMBASE_t * hetRAM, const uint32 pwm, const pwmSIG_t * sig)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    double pwmPeriod = 0.;

    if(hetRAM == hetRAM1)
    {
        pwmPeriod = (sig->period * 1000.) / 853.333F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }

    if (sig->duty == 0.)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (sig->duty >= 100U)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control)
            & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32)(pwmPeriod * sig->duty)) / 100U) << 7U ) + 128U;
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32)pwmPeriod << 7U) - 128U;

}


/** @fn void solveJoints0(double * q, const double * pos, const double * rpy)
*   @brief Computes generalized coordinates of joints
*   @param[in] pos, Pointer to Cartesian coordinates (position) of end-effector
*   @param[in] rpy, Pointer to rpy
*   @param[in] q, Pointer to generalized coordinates of joints
*
*   Computes generalized coordinates of joints
*/
void solveJoints0(double * q, const double * pos, const double * rpy)
{
    if (pos == NULL || rpy == NULL || q == NULL)
    {
        printf("# Some input arguments are not declared!\n");
        exit(1);
    }

    double x,y,z;
    x = *pos;
    y = *(pos+1);
    z = *(pos+2);

    double roll,pitch,yaw;
    roll = *rpy;
    pitch = *(rpy+1);
    yaw = *(rpy+2);

    double r11,r12,r13,r21,r22,r23,r33,cos_r,sin_r,cos_p,sin_p,cos_y,sin_y;
    double c0,s0,c1,c2,c4,c5,s1,s2,s4,s5,c123,s123,tmp;
    double q0,q1,q2,q3,q4,q5,q123;
    double a = 0.078,b = 0.105,m,n;
    //double c12,c3,c6,q12,s12,s3,s6,a1,r31,r32,t0;

    cos_r = cos(roll);
    sin_r = sin(roll);
    cos_p = cos(pitch);
    sin_p = sin(pitch);
    cos_y = cos(yaw);
    sin_y = sin(yaw);

    if (abs(cos_p)<0.0000000001)
        cos_p  =  cos_p + 0.000000001;

    r11 = cos_y*cos_p;
    r12 = cos_y*sin_p*sin_r-sin_y*cos_r;
    r13 = cos_y*sin_p*cos_r+sin_y*sin_r;
    r21 = sin_y*cos_p;
    r22 = sin_y*sin_p*sin_r+cos_y*cos_r;
    r23 = sin_y*sin_p*cos_r-cos_y*sin_r;
    //r31 = -sin_p;
    //r32 = cos_p*sin_r;
    r33 = cos_p*cos_r;

    //t0 = y/x;
    tmp = sqrt(x*x+y*y);
    s0 = y/tmp;
    c0 = x/tmp;
    q0 = atan2(s0,c0);

    s4 = r13*s0-r23*c0;
    q4 = asin(s4);
    c4 = cos(q4);;

    c5 = (r22*c0-r12*s0)/c4;
    s5 = (r21*c0-r11*s0)/c4;
    q5 = atan2(s5,c5);

    s123 = (r13*c0+r23*s0)/c4;
    c123 = r33/c4;
    q123 = atan2(s123,c123);
    if (!s123 && c123<0) q123 = PI;

    m  = x*c0+y*s0-0.075*s123-0.01;
    n  = (z-0.075*c123);
    c2 = (m*m+n*n-a*a-b*b)/(2*a*b);
    q2 = acos(c2);
    s2 = sin(q2);

    s1 = (m*a*c2-n*a*s2+m*b)/(m*m+n*n);
    q1 = asin(s1);
    c1 = cos(q1);

    q3 = q123-q1-q2;

    *q = q0;
    *(q+1) = q1;
    *(q+2) = q2;
    *(q+3) = q3;
    *(q+4) = q4;
    *(q+5) = q5;

#if DEBUG
    printf("%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s%-10s\n",
            "s2", "c2", "q2",
            "s1", "c1", "q1",
            "m", "n",
            "s123", "c123", "q123",
            "s4", "c4", "q4");
    printf("%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f%-10.2f\n",
            s2, c2, q2,
            s1, c1, q1,
            m, n,
            s123, c123, q123,
            s4, c4, q4);
#endif

}


/** @fn float tic(void)
*   @brief Starts stopwatch timer and gets the current time
*   @param[out] Current timer value in [us]
*
*   Starts stopwatch timer and gets the current time
*   NOTE: All 3 event counters 0-2 are reset
*/
float tic(void)
{
    uint32 cycles;
    _pmuStartCounters_(pmuCYCLE_COUNTER);
    cycles = _pmuGetCycleCount_();
    return (float)cycles / (f_HCLK);
}


/** @fn float toc(const float timerVal)
*   @brief Reads elapsed time from stopwatch
*   @param[in] timerVal, Initial timer value in [us]
*   @param[out] tElapsed, Elapsed time in [us]
*
*   Reads elapsed time from stopwatch
*/
float toc(const float timerVal)
{
    uint32 cycles;
    float tElapsed;
    _pmuStopCounters_(PMU_CYCLE_COUNT);
    cycles = _pmuGetCycleCount_();
    tElapsed = (float)cycles / (f_HCLK) - timerVal;
    return tElapsed;
}


/* USER CODE END */

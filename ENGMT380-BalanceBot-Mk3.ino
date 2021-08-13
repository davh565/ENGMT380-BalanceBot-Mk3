#include <Filters.h>
// #include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include "MeMegaPi.h"
#include "utility.h"

#define UPRIGHT -.53  //deg
#define TILTBAK -8  //deg
#define LPF_FRQ 0.25  //Hz
#define ANGPOT_RANGE 5 //deg
#define SPDPOT_RANGE 50 //rpm
#define SAMPLETIME 20 //ms
#define DUAL_LOOP true

//Angle Control Params
double angSetPointDeg = UPRIGHT;
double angKp = 7.5;
double angKi = 15;
double angKd = 0;
double angMaxDeg = 25;
double angMinDeg = -angMaxDeg;
//Angle Control Vars
double angYdeg;
double angYscaled;
double angOffsetDeg;
double angOffset;
double angSetPointScaled = scale(angSetPointDeg,angMaxDeg,angMinDeg);
double angSP = angSetPointScaled;
double angIn; // 0-255
double angOut;  //0-255

#if DUAL_LOOP
//Speed Control Params
double spdSetPointRPM = 0;
double spdKp = 1;
double spdKi = 0;
double spdKd = 0;
double spdMaxRPM = 170;
double spdMinRPM = -spdMaxRPM;
//Speed Control Vars
double spdMotor1;
double spdMotor2;
double spdMotorAvg;
double spdMotorAvgLPF;
double spdOffsetRPM;
double spdOffset;
double spdSetPointScaled = scale(spdSetPointRPM,spdMaxRPM,spdMinRPM);
double spdSP = spdSetPointScaled;
double spdIn; // 0-255
double spdOut;  //0-255
double pidBias = 1.0; // 0 angle control, 1 speed control, 0.5 equal mix
double angInMixed;
#endif
//Class Instantiation
    // MeUltrasonicSensor ultraSensor(PORT_7);
MeGyro gyro;
MePotentiometer angPot(PORT_7);
MePotentiometer spdPot(PORT_8);
MeEncoderOnBoard motor1(SLOT1);
MeEncoderOnBoard motor2(SLOT2);
FilterOnePole spdLPF( LOWPASS, LPF_FRQ);
PID angPID(&angIn, &angOut, &angSP, angKp, angKi, angKd, DIRECT);
#if DUAL_LOOP
PID spdPID(&spdIn, &spdOut, &spdSP, spdKp, spdKi, spdKd, DIRECT);
#endif

void isr_process_encoder1(void){
    if(digitalRead(motor1.getPortB()) == 0) motor1.pulsePosMinus();
    else motor1.pulsePosPlus();
}
void isr_process_encoder2(void){
    if(digitalRead(motor2.getPortB()) == 0) motor2.pulsePosMinus();
    else motor2.pulsePosPlus();
}

void setup(){
    #if DUAL_LOOP
    spdMotorAvg = 0;
    spdIn = 0;
    spdPID.SetSampleTime(SAMPLETIME); //ms
    spdPID.SetOutputLimits(-255,255);
    spdPID.SetMode(AUTOMATIC);

    #endif
    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);
    //Set Motor PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    Serial.begin(115200);

    gyro.begin();
    angYdeg = gyro.getAngleY();
    angYscaled = scale(angYdeg,angMaxDeg,angMinDeg);
    angIn = angYscaled;
    angPID.SetSampleTime(SAMPLETIME); //ms
    angPID.SetOutputLimits(-255,255);
    angPID.SetMode(AUTOMATIC);
}

void loop(){
    motor1.updateSpeed();
    motor2.updateSpeed();
    #if DUAL_LOOP
    //Speed Control
        //modify setpoint by pot value
    spdOffsetRPM = scale(spdPot.read(),972,0,SPDPOT_RANGE,-SPDPOT_RANGE);
    spdOffset = scale(spdOffsetRPM,spdMaxRPM,spdMinRPM,255,-255);
    spdSP = spdSetPointScaled +spdOffset;
    spdMotor1 = motor1.getCurrentSpeed();
    spdMotor2 = motor2.getCurrentSpeed();
    spdMotorAvg = (spdMotor1+spdMotor2)/2;
    spdLPF.input(spdMotorAvg);
    spdMotorAvgLPF = spdLPF.output();
    spdIn = scale(spdMotorAvgLPF,spdMaxRPM,spdMinRPM);
    spdPID.Compute();
    #endif
    //Angle Control
    gyro.update();
        //modify setpoint by pot value
    angOffsetDeg = scale(angPot.read(),972,0,ANGPOT_RANGE,-ANGPOT_RANGE);
    angOffset = scale(angOffsetDeg,angMaxDeg,angMinDeg,255,-255);
    #if DUAL_LOOP
    //cascade PID, speed outer loop, angle inner loop
    angSP = mix(angSetPointScaled +angOffset,spdOut,pidBias);
    #else
    angSP = angSetPointScaled +angOffset;
    #endif
    angYdeg = gyro.getAngleY();
    angIn = 255-scale(angYdeg,angMaxDeg,angMinDeg);
    angPID.Compute();

 

    #if true
        #if true
    Serial.print("motorSpd:");
    Serial.print(spdMotorAvgLPF);
    Serial.print(" spdSPRPM:");
    Serial.print(spdSetPointRPM+spdOffsetRPM);
    Serial.print(" spdSP:");
    Serial.print(spdSP);
    Serial.print(" spdIn:");
    Serial.print(spdIn);
    Serial.print(" spdOut:");
    Serial.print(spdOut);
        #endif
        #if false
    Serial.print("motorSpd:");
    Serial.print(motor1.getCurrentSpeed());
    Serial.print(" angIn:");
    Serial.print(angIn);
    Serial.print(" angOut:");
    Serial.print(angOut);
    Serial.print(" angSP:");
    Serial.print(angSP);
    Serial.print(" angSPdeg:");
    Serial.print(angSetPointDeg+angOffsetDeg);
        #endif

    Serial.print("\n");
    #endif
   //Motor Output
    motor1.setMotorPwm(-spdOut);
    motor2.setMotorPwm(spdOut);
}

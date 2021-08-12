#include <Filters.h>
// #include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include "MeMegaPi.h"
#include "utility.h"

#define UPRIGHT -1.9  //deg
#define TILTBAK -8  //deg
#define LPF_FRQ 0.25  //Hz
#define ANGPOT_RANGE 10 //deg
#define SPDPOT_RANGE 10 //rpm
#define DUAL_LOOP true

//Angle Control Params
double angSetPointDeg = UPRIGHT;
double angKp = 1;
double angKi = 0;
double angKd = 0;
double angMaxDeg = 25;
double angMinDeg = -angMaxDeg;
//Angle Control Vars
double angYdeg;
double angOffsetDeg;
double angOffset;
double angSP = scale(angSetPointDeg,angMaxDeg,angMinDeg);
double angIn; // 0-255
double angOut;  //0-255

#if DUAL_LOOP
//Speed Control Params
double spdSetPointRPM = 20;
double spdKp = 1;
double spdKi = 0;
double spdKd = 0;
double spdMaxRPM = 255;
double spdMinRPM = 0;
//Speed Control Vars
double spdMotor1;
double spdMotor2;
double spdMotorAvg;
double spdMotorAvgLPF;
double spdOffsetRPM;
double spdOffset;
double spdSP = scale(spdSetPointRPM,spdMaxRPM,spdMinRPM);
double spdIn; // 0-255
double spdOut;  //0-255
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

void isr_process_encoder1(void){
    if(digitalRead(motor1.getPortB()) == 0){
        motor1.pulsePosMinus();
    }
    else{
        motor1.pulsePosPlus();;
    }
}
void isr_process_encoder2(void){
    if(digitalRead(motor2.getPortB()) == 0){
        motor2.pulsePosMinus();
    }
    else{
        motor2.pulsePosPlus();
    }
}
#endif
void setup(){
    #if DUAL_LOOP
    spdMotorAvg = 0;
    spdIn = 0;
    spdPID.SetSampleTime(50); //ms
    spdPID.SetMode(AUTOMATIC);

    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);
    #endif
    //Set Motor PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    Serial.begin(115200);

    gyro.begin();
    angYdeg = gyro.getAngleY();
    angIn = scale(angYdeg,angMaxDeg,angMinDeg);
    angPID.SetSampleTime(50); //ms
    angPID.SetMode(AUTOMATIC);
}

void loop(){
    #if DUAL_LOOP
    //Speed Control
        //modify setpoint by pot value
    spdOffsetRPM = scale(spdPot.read(),972,0,SPDPOT_RANGE,-SPDPOT_RANGE);
    spdOffset = scale(spdOffsetRPM,spdMaxRPM,-spdMinRPM,255,-255);
    spdSP = scale(spdSetPointRPM,spdMaxRPM,spdMinRPM) +spdOffset;
    motor1.updateSpeed();
    motor2.updateSpeed();
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
    angOffset = scale(angOffsetDeg,angMaxDeg,-angMinDeg,255,-255);
    angSP = scale(angSetPointDeg,angMaxDeg,angMinDeg) +angOffset;
    angYdeg = gyro.getAngleY();
    angIn = scale(angYdeg,angMaxDeg,angMinDeg);
    angPID.Compute();

    //Motor Output
    motor1.setMotorPwm(angOut);
    motor2.setMotorPwm(-angOut);

    #if true
        #if DUAL_LOOP
    Serial.print("motorSpd:");
    Serial.print(spdMotorAvgLPF);
        #endif
    Serial.print(" angOut:");
    Serial.print(angOut);
    Serial.print("\n");
    #endif

}

#include <Filters.h>
// #include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Wire.h>
#include "MeMegaPi.h"
#include "utility.h"

#define UPRIGHT 0  //deg
#define TILTBAK 0  //deg
#define LPF_FRQ 0.25  //Hz
#define ANGPOT_RANGE 35 //deg
#define SPDPOT_RANGE 50 //rpm
#define SAMPLETIME 20 //ms
#define DUAL_LOOP true

//Angle Control Params
double angSetPointDeg = UPRIGHT;
double angKpAgg = 15;//2.5;
double angKiAgg = 15;//1;
double angKdAgg = 0.05;
double angKpCon = 4.5;//2.5;
double angKiCon = 3;//1;
double angKdCon = 0;
double angConMaxDeg = 25;
double angConMinDeg = -25;
double angMaxDeg = 41;
double angMinDeg = -41;
//Angle Control Vars
double angYdeg;
double angYscaled;
double angOffsetDeg;
double angOffset;
double angSetPointScaled = scale(angSetPointDeg,angMaxDeg,angMinDeg,255,-255);
double angSP = angSetPointScaled;
double angIn; // 0-255
double angOut;  //0-255


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
double spdSetPointScaled = scale(spdSetPointRPM,spdMaxRPM,spdMinRPM,255,-255);
double spdSP = spdSetPointScaled;
double spdIn; // 0-255
double spdOut;  //0-255
double pidBias = 1.0; // 0 angle control, 1 speed control, 0.5 equal mix
double angInMixed;


//Turn Control Params
double turnSetPointDeg = 0;

double turnKp = 4.5;//2.5;
double turnKi = 3;//1;
double turnKd = 0;
double turnMaxDeg = 25;
double turnMinDeg = -25;
double turnMaxDeg = 41;
double turnMinDeg = -41;
//Turn Steering Vars
double turnYdeg;
double turnYscaled;
double turnOffsetDeg;
double turnOffset;
double turnSetPointScaled = scale(turnSetPointDeg,turnMaxDeg,turnMinDeg,255,-255);
double turnSP = turnSetPointScaled;
double turnIn; // 0-255
double turnOut;  //0-255
double motorOut;
//Class Instantiation
    // MeUltrasonicSensor ultraSensor(PORT_7);
MeGyro gyro;
MePotentiometer angPot(PORT_7);
MePotentiometer spdPot(PORT_8);
MeEncoderOnBoard motor1(SLOT1);
MeEncoderOnBoard motor2(SLOT2);
FilterOnePole spdLPF( LOWPASS, LPF_FRQ);
PID angPID(&angIn, &angOut, &angSP, angKpCon, angKiCon, angKdCon, DIRECT);

PID spdPID(&spdIn, &spdOut, &spdSP, spdKp, spdKi, spdKd, DIRECT);


void isr_process_encoder1(void){
    if(digitalRead(motor1.getPortB()) == 0) motor1.pulsePosMinus();
    else motor1.pulsePosPlus();
}
void isr_process_encoder2(void){
    if(digitalRead(motor2.getPortB()) == 0) motor2.pulsePosMinus();
    else motor2.pulsePosPlus();
}

void setup(){
    
    spdMotorAvg = 0;
    spdIn = 0;
    spdPID.SetSampleTime(SAMPLETIME); //ms
    spdPID.SetOutputLimits(-255,255);
    spdPID.SetMode(AUTOMATIC);

    
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
    angYscaled = scale(angYdeg,angMaxDeg,angMinDeg,255,-255);
    angIn = angYscaled;
    angPID.SetSampleTime(SAMPLETIME); //ms
    angPID.SetOutputLimits(-255,255);
    angPID.SetMode(AUTOMATIC);
}

void loop(){
    motor1.updateSpeed();
    motor2.updateSpeed();
    
    //Speed Control
        //modify setpoint by pot value
    spdOffsetRPM = scale(spdPot.read(),972,0,SPDPOT_RANGE,-SPDPOT_RANGE);
    spdOffset = scale(spdOffsetRPM,spdMaxRPM,spdMinRPM,255,-255);
    spdSP = spdSetPointScaled +spdOffset;
    spdMotor1 = motor1.getCurrentSpeed();
    spdMotor2 = motor2.getCurrentSpeed();
    spdMotorAvg = (spdMotor1-spdMotor2)/2;
    spdLPF.input(spdMotorAvg);
    spdMotorAvgLPF = spdLPF.output();
    spdIn = -scale(spdMotorAvgLPF,spdMaxRPM,spdMinRPM,255,-255);
    spdPID.Compute();
    
    //Angle Control
    gyro.update();
        //modify setpoint by pot value
    angOffsetDeg = scale(angPot.read(),972,0,ANGPOT_RANGE,-ANGPOT_RANGE);
    angOffset = scale(angOffsetDeg,angMaxDeg,angMinDeg,255,-255);
    angSP = spdOut;
    angYdeg = gyro.getAngleY();
    //Choose between agressive and conservative tunings based on current angle
    if (angYdeg > angConMaxDeg || angYdeg < angConMinDeg) {
        angPID.SetTunings(angKpAgg,angKiAgg,angKdAgg);
    }
    else angPID.SetTunings(angKpCon,angKiCon,angKdCon);
    angIn = -scale(angYdeg,angMaxDeg,angMinDeg,255,-255);
    if (angIn > 255) angIn = 255;
    else if (angIn < -255) angIn = -255;
    angPID.Compute();

    //cap at limits
    motorOut = angOut;
    if (angYdeg > angMaxDeg) motorOut = 0;
    else if (angYdeg < angMinDeg) motorOut = 0;


 


    // Serial.print("motorSpd:");
    // Serial.print(motor1.getCurrentSpeed());
    // Serial.print(" spdSPRPM:");
    // Serial.print(spdSetPointRPM+spdOffsetRPM);
    Serial.print(" spdIn:");
    Serial.print(spdIn);
    Serial.print(" spdSP:");
    Serial.print(spdSP);
    // Serial.print(" spdIn:");
    // Serial.print(spdIn);
        // 
        // #if false
    Serial.print(" angIn:");
    Serial.print(angIn);
    // Serial.print(" angSP:");
    // Serial.print(angSetPointScaled +angOffset);
    // Serial.print(" angOut:");
    // Serial.print(angOut);
    Serial.print(" spdOut:");
    Serial.print(spdOut);
    // Serial.print(" spdErr:");
    // Serial.print(spdSP-spdIn);
        

    Serial.print("\n");
    
   //Motor Output
    motor1.setMotorPwm(-motorOut);
    motor2.setMotorPwm(motorOut);
}

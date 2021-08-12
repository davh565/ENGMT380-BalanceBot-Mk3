#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Filters.h>
#include <Wire.h>

#define UPRIGHT -1.9  //1.52
#define TILTBAK -8
#define LPF_FRQ 0.25
// MeUltrasonicSensor ultraSensor(PORT_7);
MeGyro gyro;
MePotentiometer myPotentiometer(PORT_7);
MePotentiometer myPotentiometer2(PORT_8);
MeEncoderOnBoard motor1(SLOT1);
MeEncoderOnBoard motor2(SLOT2);
FilterOnePole lpfSpd( LOWPASS, LPF_FRQ);

void setup(){
    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    Serial.begin(115200);
    gyro.begin();
}

void loop(){
    
}
#ifndef COMMS_H
#define COMMS_H

void sendData(float sig0, float sig90, float output);
void cmdSendDataOn();
void cmdGetSensWfm();
void cmdGetHeaterWfm();
void cmdGetRefWfm();
void cmdTestComs();
void cmdPIDGainFlip();
void cmdResetIntegral();
void cmdPIDOn();
void cmdChangeSetPoint();
void cmdGetSetPoint();
void cmdSetKp();
void cmdSetKi();
void cmdSetKd();
void cmdGetKp();
void cmdGetKi();
void cmdGetKd();
void cmdSetGains();
void cmdSetSensPhase();
void cmdSetSensAmplitude();
void cmdGetSensAmplitude();
void cmdSetReadLag();
void cmdGetReadLag();
void cmdUnrecognised(const char *command);

#endif //COMMS_H

#include <PID_v1.h>
#include "SerialCommand.h"
#include "temperature_controller.h"
#include "comms.h"

// EXTERNS----------------------------------------
//Wave-forms----------
extern int GLB_ref_wv_fm[N_REF_WV];
extern int GLB_heater_wv_fm[Nbase];
extern int GLB_sens_wv_fm[Nbase];
extern int GLB_read_quad0_wv_fm[Nbase];
extern int GLB_read_quad90_wv_fm[Nbase];
extern int GLB_heater_amplitude; // DAC units
extern float GLB_heater_phase; //Radians
extern float GLB_sens_phase; //Radians
extern int GLB_sens_amplitude; // DAC units
extern float GLB_read_phase_lag; //Rads

//Flags to change behaviour
extern bool GLB_send_data;

//PID control globals (could probably avoid kp, ki, kd really...)
extern double GLB_kp; 
extern double GLB_ki; 
extern double GLB_kd;
extern double pidInput, pidOutput, pidSetpoint;
extern PID myPID;


//Needed to share with comms.h
extern volatile int GLB_imax_on_amp;
extern SerialCommand sCmd; 
///----------------------------------------------------------------
//--------------  SERIAL COMMUNICATIONS STUFF
// Functions for talking to the computer
/*Write the time, read values, and output value*/
void sendData(float sig0, float sig90, float output){
	Serial.print("d: ");
	Serial.print(millis());
	Serial.print(" ");
	Serial.print(sig0);
	Serial.print(" ");
	Serial.print(sig90);
	Serial.print(" ");
	Serial.println(output);
}

void cmdSendDataOn(){
	char *arg;
	arg = sCmd.next();	 
	if(strcmp(arg,"True")==0 || strcmp(arg,"1")==0) {		
		GLB_send_data=true;
	}
	else if ( strcmp(arg,"False")==0 || strcmp(arg,"0")==0) {	//strcmpr(arg, "false")
		GLB_send_data=false;
	}
    Serial.print("p: Data sending is "); Serial.println(GLB_send_data);

}
void cmdGetSensWfm(){
    Serial.print("sens_wfm: ");
    //for(int k=0; k<N_SENS_WV; k++){
    for(int k=0; k<Nbase; k++){
        Serial.print(GLB_sens_wv_fm[k]);
        Serial.print(", ");
    }
    Serial.println("");
}
void cmdGetHeaterWfm(){
    Serial.print("heat_wfm: ");
    //for(int k=0; k<N_HEATER_WV; k++){
    for(int k=0; k<Nbase; k++){
        Serial.print(GLB_heater_wv_fm[k]);
        Serial.print(", ");
    }
    Serial.println("");
}
void cmdGetRefWfm(){
    Serial.print("ref_wfm: ");
    for(int k=0; k<N_REF_WV; k++){
        Serial.print(GLB_ref_wv_fm[k]);
        Serial.print(", ");
    }
    Serial.println("");
}
void cmdTestComs(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		Serial.print("echoing- you sent:  ");
		Serial.println(arg);
	}
	else {
		Serial.println("We're communicating.");
	}
}
void cmdPIDGainFlip(){
	char *arg;
	arg = sCmd.next();	 
	int newDir=0;
	if(strcmp(arg,"True")==0 || strcmp(arg,"1")==0) {		
		newDir=REVERSE;
	}
	else if ( strcmp(arg,"False")==0 || strcmp(arg,"0")==0) {	//strcmpr(arg, "false")
		newDir=DIRECT;
	}
	int curMode=myPID.GetMode();
	if (curMode == AUTOMATIC){
		myPID.SetMode(MANUAL);
	}
	myPID.SetControllerDirection(newDir);
	if (curMode == AUTOMATIC){
		myPID.SetMode(AUTOMATIC);
	}
}
void cmdResetIntegral(){
	char *arg;
	arg = sCmd.next();	 
	double newIterm;
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			newIterm=temp;
		}
	}
	else{
		newIterm=0;
	}
	myPID.ResetIntegralTerm(newIterm);
}
void cmdPIDOn(){
	char *arg;
	arg = sCmd.next();	 
	int newState=0;
	if(strcmp(arg,"True")==0 || strcmp(arg,"1")==0) {		
		newState=AUTOMATIC;
		Serial.println("p: PID on");
	}
	else if ( strcmp(arg,"False")==0 || strcmp(arg,"0")==0) {	//strcmpr(arg, "false")
		newState=MANUAL;
		Serial.println("p: PID off");
	}
	else
		Serial.print("p: Don't understand: ");
		Serial.println(arg);
		return;

	if(myPID.GetMode()!=newState){
		myPID.SetMode(newState);
	}
}

void cmdSetOutputAmp(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float val= atof(arg);
		//write_wave_form(GLB_heater_wv_fm, N_HEATER_WV, NCYC_HEAT,val, GLB_heater_phase);
		write_wave_form(GLB_heater_wv_fm, N_HEATER_WV, NCYC_HEAT, IMAX, GLB_heater_phase);
		//GLB_imax_on_amp=IMAX/val;
		if(val==0)
			Serial.print("p: Set output to zero- maybe I didn't understand?");
	}
}
void cmdChangeSetPoint(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			pidSetpoint=temp;
		}
	}
	Serial.print("set_point: "); Serial.println(pidSetpoint);
}
void cmdGetSetPoint(){
	Serial.print("set_point: ");
	Serial.println(pidSetpoint);
}
void cmdSetKp(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_kp=temp;
		}
		myPID.SetTunings(GLB_kp,GLB_ki,GLB_kd);
	}
	Serial.print("kp: "); Serial.println(GLB_kp);
}
void cmdGetKp(){
	Serial.print("kp: "); Serial.println(GLB_kp);
}
void cmdSetKi(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_ki=temp;
		}
		myPID.SetTunings(GLB_kp,GLB_ki,GLB_kd);
	}
	Serial.print("ki: "); Serial.println(GLB_ki);
}
void cmdGetKi(){
	Serial.print("ki: "); Serial.println(GLB_ki);
}
void cmdSetKd(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_kd=temp;
		}
		myPID.SetTunings(GLB_kp,GLB_ki,GLB_kd);
	}
	Serial.print("kd: "); Serial.println(GLB_kd);
}
void cmdGetKd(){
	Serial.print("kd: "); Serial.println(GLB_kd);
}
void cmdSetGains(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_kp=temp;
		}
	}

	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_ki=temp;
		}
	}

	arg = sCmd.next();	 
	if (arg != NULL){
		float temp = atof(arg);
		if(temp != -1){
			GLB_kd=temp;
		}
	}
	myPID.SetTunings(GLB_kp,GLB_ki,GLB_kd);
	Serial.print("p: gains are now	(Kp, Ki, Kd): (");
	Serial.print(GLB_kp);
	Serial.print(", ");
	Serial.print(GLB_ki);
	Serial.print(", ");
	Serial.print(GLB_kd);
	Serial.println(")");
}
void cmdSetSensPhase(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		int new_phase= atof(arg);
		GLB_sens_phase=new_phase;
		write_wave_form(GLB_sens_wv_fm, N_SENS_WV, 8, GLB_sens_amplitude, GLB_sens_phase);
		Serial.println();
		Serial.print("p: NEW SENSE WAVEFORM PHASE IS: ");Serial.println(GLB_sens_phase);
		Serial.println();
	}
	else {
		Serial.print("p: Sense phase is currently set to: "); Serial.println(GLB_sens_phase);
	}
}
void cmdGetSensPhase(){
    Serial.print("sens_phase: "); Serial.println(GLB_sens_phase);
}
void cmdSetSensAmplitude(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		int new_amp= atof(arg);
		GLB_sens_amplitude=new_amp;
		write_wave_form(GLB_sens_wv_fm, N_SENS_WV, NCYC_SENS, GLB_sens_amplitude, GLB_sens_phase);
		Serial.print("p: NEW SENSE WAVEFORM AMPLITUDE IS: ");Serial.println(GLB_sens_amplitude);
	}
	else {
		Serial.print("p: Ampl is currently set to- "); Serial.println(GLB_sens_amplitude);
	}
}
void cmdGetSensAmplitude(){//sens_amp
    Serial.print("sens_amp: ");Serial.println(GLB_sens_amplitude);
}
void cmdSetReadLag(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_lag= atof(arg);
		GLB_read_phase_lag=new_lag;
		write_wave_form(GLB_read_quad0_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag);
		write_wave_form(GLB_read_quad90_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag+PI_ON_2);
		Serial.print("p: NEW PHASE_LAG IS- ");Serial.println(new_lag);
		//If this takes too long, we could also introduce a new pointer for the read waveform and increment it differently to i_sens
	}
	else {
		Serial.print("p: Phase is currently set to "); Serial.println(GLB_read_phase_lag);
	}
}
void cmdGetReadLag(){ //read_lag
		Serial.print("read_lag: ");Serial.println(GLB_read_phase_lag);
}
void cmdUnrecognised(const char *command){
	Serial.println("p: Don't understand!");
}

void connectCommands(){
	sCmd.addCommand("read_lag", cmdSetReadLag);	// 
	sCmd.addCommand("read_lag?", cmdGetReadLag);	// 
	sCmd.addCommand("sens_phase", cmdSetSensPhase);	// 
	sCmd.addCommand("sens_phase?", cmdGetSensPhase);	// 
	sCmd.addCommand("sens_wfm?", cmdGetSensWfm);	// 
	sCmd.addCommand("ref_wfm?", cmdGetRefWfm);	// 
	sCmd.addCommand("heat_wfm?", cmdGetHeaterWfm);	// 
	sCmd.addCommand("sens_amp", cmdSetSensAmplitude);	// 
	sCmd.addCommand("sens_amp?", cmdGetSensAmplitude);	// 
	sCmd.addCommand("kp", cmdSetKp);	// 
	sCmd.addCommand("kp?", cmdGetKp);	// 
	sCmd.addCommand("ki", cmdSetKi);	// 
	sCmd.addCommand("ki?", cmdGetKi);	// 
	sCmd.addCommand("kd", cmdSetKd);	// 
	sCmd.addCommand("kd?", cmdGetKd);	// 
	sCmd.addCommand("gains", cmdSetGains);	// 
	sCmd.addCommand("test_coms", cmdTestComs);	// 		//
	sCmd.addCommand("pid_on", cmdPIDOn);
	sCmd.addCommand("gain_flip", cmdPIDGainFlip);
	sCmd.addCommand("reset_integral", cmdResetIntegral);
	sCmd.addCommand("set_point", cmdChangeSetPoint);
	sCmd.addCommand("set_point?", cmdGetSetPoint);
	sCmd.addCommand("send_data_on", cmdSendDataOn);
	sCmd.addCommand("set_output_amp", cmdSetOutputAmp);

	sCmd.setDefaultHandler(cmdUnrecognised);  
}
// END SERIAL COMMUNCATIONS STUFF

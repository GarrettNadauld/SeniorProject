#include "ClearCore.h"

// Specify which motors to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motorX ConnectorM0
#define motorY ConnectorM1

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Select the baud rate to match the target serial device
#define baudRate 9600
 
// Specify which serial connector to use: ConnectorUsb, ConnectorCOM0, or
// ConnectorCOM1
#define SerialPort ConnectorUsb
 
// This example has built-in functionality to automatically clear motor alerts, 
//  including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when 
//  encountered and return a motor to a state in which motion is allowed. Before 
//  enabling this functionality, be sure to understand this behavior and ensure 
//  your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (0)

// Define the velocity and acceleration limits to be used for each move
const int32_t velocityLimit = 10000; // pulses per sec
const int32_t accelerationLimit = 100000; // pulses per sec^2
 
//Define constants for PDP90A
const int32_t Lx = 10E-3; // Length of x axis in m
const int32_t Ly = 10E-3; // Length of y axis in m
const int32_t en = 300E-6; // Output noise voltage in Vrms
const int32_t Xtol = 5; // X axis tolerance 
const int32_t Ytol = 5; // Y axis tolerance
const int32_t delta = 1; // Steps for smallest delta voltage

int16_t leveling; // State of input switch


// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
bool SynchronizedMove(int32_t distance);
void PrintAlerts();
void HandleAlerts();


/*------------------------------------------------------------------------------
 * Main
 *
 *    Main loop, reads analog input of SUM, deltaX, deltaY
 *		 checks if laser position is within x and y tolerance
 *               calls move motor if out of tolerance, alternates starting with x and y
 *               loops every 1 second 
 *    
 *
 * Parameters:
 *    None
 *
 * Returns: 
 *    None
 */
int main() {

    // Set the resolution of the ADC.
    AdcMgr.AdcResolution(adcResolution);

    double inputVoltageSUM, inputVoltageY, inputVoltageX = 0.0;
    bool LevelFlag = false;	//Used to set level position of first iteration of loop
    bool Xflag = true;	//Used to determine which axis to move first
    double LevelX, LevelY, Xpos, Ypos = 0.0;
 
    while (true) {

        double inputVoltageSUM, inputVoltageY, inputVoltageX = 0.0;
        inputVoltageSUM, inputVoltageY, inputVoltageX = readAnalogInputs();	//Read analog input from sensor

        if (leveling) {	//Once switch has been set to the on position the bed is level, enter automated leveling state
            if (LevelFlag==false) {	//If either level flag is false calibrate level position to be compared to new level data and set flag to true
                LevelX = (Lx * inputVoltageX) / (2.0 * inputVoltageSUM);	//Set LevelX sensor position
                LevelY = (Ly * inputVoltageY) / (2.0 * inputVoltageSUM);	//Set LevelY sensor position
		LevelFlag = true;
            }
    

            Xpos = (Lx * inputVoltageX) / (2.0 * inputVoltageSUM);	//New laser position for X
            Ypos = (Ly * inputVoltageY) / (2.0 * inputVoltageSUM);	//New laser postioin for Y

            if (Xflag) { // If Xflag is true move X axis first
                if (Xpos > LevelX + Xtol) { // If Xpos is greater than LevelX + Xtol
                    MoveDistanceX((Xpos - LevelX) / delta);
                } else if (Xpos < LevelX - Xtol) { // If Xpos is less than LevelX - Xtol
                    MoveDistanceX((LevelX - Xpos) / delta); 
                }
                
                if (Ypos > LevelY + Ytol) { // If Ypos is greater than LevelY + Ytol
                    MoveDistanceY((Ypos - LevelY) / delta);
                } else if (Ypos < LevelY - Ytol) { // If Ypos is less than LevelY - Ytol
                    MoveDistanceY((LevelY - Ypos) / delta);
                }
            } else { // If Xflag is false move Y axis first
                if (Ypos > LevelY + Ytol) { // If Ypos is greater than LevelY + Ytol
                    MoveDistanceY((Ypos - LevelY) / delta);
                } else if (Ypos < LevelY - Ytol) { // If Ypos is less than LevelY - Ytol
                    MoveDistanceY((LevelY - Ypos) / delta);
                }
                
                if (Xpos > LevelX + Xtol) { // If Xpos is greater than LevelX + Xtol
                    MoveDistanceX((Xpos - LevelX) / delta); 
                } else if (Xpos < LevelX - Xtol) { // If Xpos is less than LevelX - Xtol
                    MoveDistanceX((LevelX - Xpos) / delta);
                }
            }
            Xflag = !Xflag; // Switch flag for next iteration
	}
 
        Delay_ms(1000);		// Wait a 1 second before the next reading.
    }
}

/*------------------------------------------------------------------------------
 * ReadAnalogInputs
 *
 *    Used to get voltages from A9-A12
 *   
 *
 * Parameters:
 *    None
 *
 * Returns:
 *    SUM voltage
 *    X voltage
 *    Y voltage
 */
double readAnalogInputs() {
    // Read the analog input (A-9 through A-12 may be configured as analog
        // inputs).
        int16_t adcSUM = ConnectorA12.State();
        // Convert the reading to a voltage.
        double inputVoltageSUM = 10.0 * adcSUM / ((1 << adcResolution) - 1);
        
        int16_t adcY = ConnectorA11.State();
        // Convert the reading to a voltage.
        double inputVoltageY = 10.0 * adcY / ((1 << adcResolution) - 1);
        
        int16_t adcX = ConnectorA10.State();
        // Convert the reading to a voltage.
        double inputVoltageX = 10.0 * adcX / ((1 << adcResolution) - 1);

    return inputVoltageSUM, inputVoltageY, inputVoltageX;
}
 
/*------------------------------------------------------------------------------
 * MoveDistanceX
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motorX has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool MoveDistanceX(int32_t distance) {
    // Check if a motorX alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motorX.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("MotorX alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Move canceled.");      
        SerialPort.SendLine();
        return false;
    }
 
    SerialPort.Send("Moving distance: ");
    SerialPort.SendLine(distance);
 
    // Command the move of incremental distance
    motorX.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    SerialPort.SendLine("Moving.. Waiting for HLFB");
    while ( (!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorX.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // Check if motorX alert occurred during move
    // Clear alert if configured to do so 
    if (motorX.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("MotorX alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("Move Done");
        return true;
    }
}

bool MoveDistanceY(int32_t distance) {
    // Check if a motorY alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("MotorY alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Move canceled.");      
        SerialPort.SendLine();
        return false;
    }
 
    SerialPort.Send("Moving distance: ");
    SerialPort.SendLine(distance);
 
    // Command the move of incremental distance
    motorY.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    SerialPort.SendLine("Moving.. Waiting for HLFB");
    while ( (!motorY.StepsComplete() || motorY.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorY.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // Check if motorY alert occurred during move
    // Clear alert if configured to do so 
    if (motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("MotorY alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        SerialPort.SendLine("Move Done");
        return true;
    }
}
//------------------------------------------------------------------------------
 
 
/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
 void PrintAlerts(){
    // report status of alerts
    SerialPort.SendLine("Alerts present: ");
    if(motor.AlertReg().bit.MotionCanceledInAlert){
        SerialPort.SendLine("    MotionCanceledInAlert "); }
    if(motor.AlertReg().bit.MotionCanceledPositiveLimit){
        SerialPort.SendLine("    MotionCanceledPositiveLimit "); }
    if(motor.AlertReg().bit.MotionCanceledNegativeLimit){
        SerialPort.SendLine("    MotionCanceledNegativeLimit "); }
    if(motor.AlertReg().bit.MotionCanceledSensorEStop){
        SerialPort.SendLine("    MotionCanceledSensorEStop "); }
    if(motor.AlertReg().bit.MotionCanceledMotorDisabled){
        SerialPort.SendLine("    MotionCanceledMotorDisabled "); }
    if(motor.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("    MotorFaulted ");
    }
 }
//------------------------------------------------------------------------------
 
 
/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
 void HandleAlerts(){
    if(motor.AlertReg().bit.MotorFaulted){
        // if a motor fault is present, clear it by cycling enable
        SerialPort.SendLine("Faults present. Cycling enable signal to motor to clear faults.");
        motor.EnableRequest(false);
        Delay_ms(10);
        motor.EnableRequest(true);
    }
    // clear alerts
    SerialPort.SendLine("Clearing alerts.");
    motor.ClearAlerts();
 }
//------------------------------------------------------------------------------

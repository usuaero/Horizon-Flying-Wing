/* Code Outline
1) Read in PPM signal from reciever
2) Read in telemetry data
3) Check for the mode and evaluate the control mapping functions
5) Convert control surface deflections to servo arm deflections
6) output the throttle & 11 servo commands
*/

/* NOTES:
 * PWM values treated as integers and are packaged into the custom struct 
 *      "pilot" which is a global variable
 * degree values treated as doubles and there are two arrays, one for the
 *      control surface deflections "deg" and another for the servo arm
 *      deflections "servoDeg". Both of these are global variables
 * dL is a instance of the running average class that will help smooth out
 *      the sensor data for lift control. This is also a global variable.
 * all functions and variables associated with the control mapping are
 *      included in the header file "controlMapping.h".
 */

// included the needed libraries
//##########################################################################
// Servo Library -- to control each servo/ send out individual PWM signals
#include "Servo.h"
// PulsePosition Library -- to read in / interpret the PPM signal 
#include <PulsePosition.h>
// library to read in pixhawk data
#include "ardupilotmega/mavlink.h"
// include control mapping items
#include "libraries\controlMapping\controlMapping.h"
// include running average
#include "libraries\runningAverage\runningAverage.cpp"

// FUNCTIONS USED
//##########################################################################
// Sabrina's functions
void ReadInPPM();
void deg2servoDeg();
void Send2Servo();
void debug();
void comm_receive();
void printVal(char *name, double val);

// define the global constants (they are all in uppercase for ease of identifying)
//##########################################################################
// deg2servoDeg mapping values
#define INTERCEPT 90.
#define SLOPE_L4 2.1638
#define SLOPE_L3 2.369
#define SLOPE_L2 3.2169
#define SLOPE_L1 2.4064
#define SLOPE_L0 2.3123
#define SLOPE_CE -3.0402
#define SLOPE_R0 -2.2135
#define SLOPE_R1 -2.0643
#define SLOPE_R2 -1.7686
#define SLOPE_R3 -1.9165
#define SLOPE_R4 -1.6743

// GLOBAL VARIABLES
//##########################################################################
// Create the PPM input variable - read on FALLING edge
PulsePositionInput DLRXinput(FALLING);
// Define 12 Servo instances & their corresponding pin on the Teensy
// 11 servos & 1 motor (signal split to both motors)
Servo Thrust;
int pThr = 14;
Servo L0;
int pL0 = 6;
Servo L1;
int pL1 = 5;
Servo L2;
int pL2 = 4;
Servo L3;
int pL3 = 3;
Servo L4;
int pL4 = 2;
Servo Ce;
int pC = 7;
Servo R4;
int pR4 = 12;
Servo R3;
int pR3 = 11;
Servo R2;
int pR2 = 10;
Servo R1;
int pR1 = 9;
Servo R0;
int pR0 = 8;

// create arrays for degrees deflection and servo arm degrees
double deg[11], servoDeg[11];
// create instance of telemetry data and pilot commands
struct telemetryData pix;
struct pilotCommands pilot;
// initialize the running average
runAvg* dL = new runAvg(200, 0.);

//##########################################################################
//##########################################################################
//##########################################################################

void loop() {
  //int i;
  
  // 1) Read in the PPM signal & convert to 4 PWM signals
  ReadInPPM();
  
  // read in pixhawk data
  comm_receive();
  
  // update dL
  dL->update(calc_dL(pix));
  
  // check for the mode
  if (pilot.modeSwitch < TRANS_PWM_NOM) {
    mode1(pilot, deg);
  }
  //else if (pilot.modeSwitch < ?) {
     //implement a third mode
  //}
  else {
    mode2(pilot, dL->getAverage(), deg);
  }
  
  // convert degrees control surface deflection to degrees servo arm deflection
  deg2servoDeg();
  
  // send out the individual PWM signals
  Send2Servo();
  
  // displays values to the serial monitor
  // debug();
}

// Launch the serial port in setup
void setup() {
  
  DLRXinput.begin(13); //signal from RX must be connected to pin 13

  Thrust.attach(pThr);
  L0.attach(pL0);
  L1.attach(pL1);
  L2.attach(pL2);
  L3.attach(pL3);
  L4.attach(pL4);
  Ce.attach(pC);
  R4.attach(pR4);
  R3.attach(pR3);
  R2.attach(pR2);
  R1.attach(pR1); 
  R0.attach(pR0); 
  
  Serial1.begin(57600);    // TELEM2 from Pixhawk

  // Serial.begin(9600);
  
  //Serial.print("Setup Complete\n");

  pix.airspeed = 16.5;
  pix.climbRate = 0.;
  pix.bankAngle = 0.;
  pix.elevationAngle = 0.;
  pix.rollRate = 0.;
}

void ReadInPPM(){
  // read in the PWM channels from the RX PPM signal
  pilot.ail = DLRXinput.read(1);  // roll - delta l
  pilot.ele = DLRXinput.read(2);  // pitch - delta m
  Thrust.write( DLRXinput.read(3) );  // motor - pass through directly 
  pilot.rud = DLRXinput.read(4);  // yaw - ignore Mode 1
  // we will need to read in one more channel, likely a 2 or 3 way switch so that the pilot can change modes as desired
  pilot.modeSwitch = DLRXinput.read(5);
}

void deg2servoDeg(){
  servoDeg[0]  = deg[0]  * SLOPE_L4 + INTERCEPT;
  servoDeg[1]  = deg[1]  * SLOPE_L3 + INTERCEPT;
  servoDeg[2]  = deg[2]  * SLOPE_L2 + INTERCEPT;
  servoDeg[3]  = deg[3]  * SLOPE_L1 + INTERCEPT;
  servoDeg[4]  = deg[4]  * SLOPE_L0 + INTERCEPT;
  servoDeg[5]  = deg[5]  * SLOPE_CE + INTERCEPT;
  servoDeg[6]  = deg[6]  * SLOPE_R0 + INTERCEPT;
  servoDeg[7]  = deg[7]  * SLOPE_R1 + INTERCEPT;
  servoDeg[8]  = deg[8]  * SLOPE_R2 + INTERCEPT;
  servoDeg[9]  = deg[9]  * SLOPE_R3 + INTERCEPT;
  servoDeg[10] = deg[10] * SLOPE_R4 + INTERCEPT;
}

void Send2Servo(){
  // individually sends out servo commands - PWM signal
  L4.write(servoDeg[0]);
  L3.write(servoDeg[1]);
  L2.write(servoDeg[2]);
  L1.write(servoDeg[3]);
  L0.write(servoDeg[4]);
  Ce.write(servoDeg[5]);
  R0.write(servoDeg[6]);
  R1.write(servoDeg[7]);
  R2.write(servoDeg[8]);
  R3.write(servoDeg[9]);
  R4.write(servoDeg[10]);
}

void comm_receive() {
  uint8_t c;
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(Serial1.available() > 0) { // as long as buffer size bigger than 0 -- pull out serial data

    c = Serial1.read(); // reads in the serial message
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        
        switch(msg.msgid) {
        default: break;
        // IMU data -- roll & yaw rates
        case MAVLINK_MSG_ID_RAW_IMU:  // #105 highres IMU / Check scaled IMU for acutal value readings 
          {
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            pix.rollRate = raw_imu.xgyro;
            // printVal("roll rate raw: ", raw_imu.xgyro);
          }
          break;
        // airspeed 
        case MAVLINK_MSG_ID_VFR_HUD:  // #74 VFR_HUD
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
            pix.airspeed = vfr_hud.airspeed;  // m/s
            pix.climbRate = vfr_hud.climb;  // m/s
            // printVal("airspeed: ", vfr_hud.airspeed);
            // printVal("climb rate: ", vfr_hud.climb);
            
          }
          break;
        // attitude data -- roll angles (bank angle)
        case MAVLINK_MSG_ID_ATTITUDE:  // #30 ATTITUDE
          {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            pix.bankAngle = attitude.roll;  // rad (-pi..+pi)
            pix.elevationAngle = attitude.pitch;
            // printVal("bank: ", attitude.roll);
            // printVal("elevation: ", attitude.pitch);
            
          }
          break;
      }
    }
  }
}

// void printVal(char *name, double val) {
    // Serial.print(name);
    // Serial.printf("%20.12f", val);
    // Serial.println();
    // delay(1);
// }

void debug(){
    int i;
    
    Serial.print("pilotCommands: ");
    Serial.printf("%4u", pilot.ail);
    Serial.print(", ");
    Serial.printf("%4u",pilot.ele);
    Serial.print(", ");
    Serial.printf("%4u",pilot.rud);
    Serial.print(", ");
    Serial.printf("%4u",pilot.modeSwitch);
    Serial.print(", ");
    
    Serial.print(" pixhawk: ");
    Serial.printf("%6.2f", pix.airspeed);
    Serial.print(", ");
    Serial.printf("%6.2f", pix.climbRate);
    Serial.print(", ");
    Serial.printf("%6.2f", pix.bankAngle);
    Serial.print(", ");
    Serial.printf("%6.2f", pix.elevationAngle);
    Serial.print(", ");
    Serial.printf("%6.2f", pix.rollRate);
    Serial.print(", ");
    Serial.printf("%5.3f", dL->getAverage());
    Serial.print(", ");
    
    Serial.print(" degrees: ");
    for (i=0; i<11; i++){
        Serial.printf("%6.2f", deg[i]);
        Serial.print(", ");
    }
    Serial.println();
}

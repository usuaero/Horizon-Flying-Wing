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
//// PulsePosition Library -- to read in / interpret the PPM signal 
//#include <PulsePosition.h>
// SRXL2 Library -- to read in / interpret SRXL2 signal
#include "spm_srxl.h"
// library to read in pixhawk data
#include "ardupilotmega/mavlink.h"
// include control mapping items
#include "libraries\controlMapping\controlMapping.h"
// include running average
#include "libraries\runningAverage\runningAverage.cpp"
//#include <SD.h>
//#include <SPI.h>

// FUNCTIONS USED
//##########################################################################
// Sabrina's functions
//void ReadInPPM();
void defDefl_2_degServo(double *defl, double *serv);
void Send2Servo();
void Write2Card();
void comm_receive();
void checkSRXL2();
void initializeSD();

void blend(double dL, struct pilotCommands in, double b, double *degDefl);
//#define BlendFactor 6.6666666666666667e-06            // five second blend term
#define M2BlendFactor 9.3333333333333333e-06            // five second blend term
#define M3BlendFactor 1.5333333333333333e-05            // five second blend term
#define M4BlendFactor 4.0000000000000000e-05            // five second blend term

#define Pixport Serial4

//// Global Constants for SD Card Logging
//const int chipSelect = 10;
//File logfile;
//// name of the log file stored to the SD card
//char name[11]; // "TLg999.txt"
//const int mode_1_write_loops = 500;
//const int mode_2_write_loops =  250;
//int file_number = 0;

// Values for SRXL2
#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 22 
#define srxl2port Serial1
unsigned long currentTime;

// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define RUDD 3
#define MOSW 4
#define BASW 5
#define HMSW 6
#define AUX3 7

// GLOBAL VARIABLES
//##########################################################################
//// Create the PPM input variable - read on FALLING edge
//PulsePositionInput DLRXinput(FALLING);
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
int pR4 = 19;
Servo R3;
int pR3 = 18;
Servo R2;
int pR2 = 15;
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
runAvg* dL = new runAvg(3000, 0.);
const int mode_1_runAvg_loops = 1;
const int mode_2_runAvg_loops = 1;
double b=0.0;
double CL_fixed =  0.5; // 0.4274;

//// initialize counter for running average
//int k = 0;
//// initialize counter for writing to card
//int j = 0;

int ThrustPWM = 1024; // variable to store the PWM Throttle position 
double SRXL2scale = 1400.0 / 2048.0; 
int SRXL2offset = 1500 - int(double(1024) * SRXL2scale);
bool SD_card_not_initialized = true;

int counter = 0;
int counter_max = 50000;
int counter2 = 0;

//##########################################################################
//##########################################################################
//##########################################################################

void setup()
{
//  Serial.begin(9600);
  // set up SRXL2
  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
  if (!srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001)) {
    Serial.println("SRXL2 Device initialization failed");
    return;
  }
  if (!srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES)) {
    Serial.println("SRXL2 Bus initialization failed");
    return;
  }
  else {
    Serial.println("SRXL2 initialized");
  }

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
  
  Pixport.begin(57600);    // TELEM2 from Pixhawk
  
  Serial.println("Setup Complete");

  // initialize pix struct values
  pix.airspeed = 20.5;
  pix.climbRate = 0.;
  pix.bankAngle = 0.;
  pix.elevationAngle = 0.;
  pix.rollRate = 0.;
  pix.time_msec = 0;

  // initialize pilot struct values
  pilot.ail = 1500;
  pilot.ele = 1500;
  pilot.rud = 1500;
  pilot.modeSwitch = 1800;
  pilot.baySwitch = 800;
}
void loop() {

  // 1) Read in the SRXL2 signal & convert to 5 PWM signals
  checkSRXL2();
  
  // read in pixhawk data
  comm_receive();

  // update dL
//  k = k+1;
//  if (pilot.modeSwitch < TRANS_PWM_NOM) { // in mode 1
//    if (k >= mode_1_runAvg_loops) {
//      dL->update(calc_dL(pix));
//      k = 0;
//    }
//  }
//  else { // in mode 2
//    if (k >= mode_2_runAvg_loops) {
//      dL->update(calc_dL(pix));
//      k = 0;
//    }
//  }
  dL->update(calc_dL(pix));
  
  // check for the mode
  if (pilot.modeSwitch < TRANS_PWM_NOM) {
    mode1(pilot, deg);
    b = 0.0;
  }
  //else if (pilot.modeSwitch < ?) {
     //implement a third mode
  //}
  else {
    if (b < 1.0) {
//        b += BlendFactor;
      if (pilot.higherModeSwitch < TRANS_PWM_NOM - 200) {
          b += M2BlendFactor;
      }
      else if (pilot.higherModeSwitch > TRANS_PWM_NOM + 200) {
          b += M4BlendFactor;
      }
      else {
          b += M3BlendFactor;
      }
        if (b > 1.0) {b = 1.0;}
//        blend(dL->getAverage(), pilot, b, deg);
        blend(CL_fixed, pilot, b, deg);
    }
    else {
      if (pilot.higherModeSwitch < TRANS_PWM_NOM - 200) {
//          mode2(pilot, dL->getAverage(), deg);
          mode2(pilot, CL_fixed, deg);
      }
      else if (pilot.higherModeSwitch > TRANS_PWM_NOM + 200) {
//          mode4(pilot, dL->getAverage(), deg);
          mode4(pilot, CL_fixed, deg);
      }
      else {
//          mode3(pilot, dL->getAverage(), deg);
          mode3(pilot, CL_fixed, deg);
      }
    }
  }
  
  // convert degrees control surface deflection to degrees servo arm deflection
//  deg2servoDeg();
  defDefl_2_degServo(deg, servoDeg);
//  servoDeg[6] = double(double((dL->getAverage() - 0.1)/0.9) * 30.0 + 90.0);
//  servoDeg[7] = double(b * 30.0 + 90.0);

  // bay switch
  if (pilot.baySwitch > (TRANS_PWM_NOM - 100.0)) {
    servoDeg[5] = 90.0;
  }
  
  // send out the individual PWM signals
  Send2Servo();
  
//  // write values to attached SD card
//  j = j+1;
//  if (pilot.modeSwitch < TRANS_PWM_NOM) { // in mode 1
//    if (j >= mode_1_write_loops) {
//      Write2Card();
//      j = 0;
//    }
//  }
//  else { // in mode 2
//    if (j >= mode_2_write_loops) {
//      Write2Card();
//      j = 0;
//    }
//  }
 
}

void blend(double dL, struct pilotCommands in, double b, double *degDefl) {
    /*
     * inputs
     * ======
     * dL -> current value of the running average for CL estimate
     * in -> pilot commands
     * b  -> blend parameter
     * outputs
     * =======
     * degDefl -> array of degrees deflection across horizon
     */
    // declare variables
    double m1[11], mh[11];
    int i;
    // compute mode 1 and higher mode deflections
    mode1(in, m1);
//    mode2(in, dL, mh);
    if (in.higherModeSwitch < TRANS_PWM_NOM - 200) {
        mode2(in, dL, mh);
    }
    else if (in.higherModeSwitch > TRANS_PWM_NOM + 200) {
        mode4(in, dL, mh);
    }
    else {
        mode3(in, dL, mh);     
    }
    // compute output of blended values
    for (i=0; i<11; i++) {
        degDefl[i] = (mh[i] - m1[i]) * b + m1[i];
    }
}

//void initializeSD() {
//  // set up SD card
//  Serial.println("Initializing SD card...");
//  if (!SD.begin(chipSelect)) {
//    Serial.println("initialization failed!");
//    return;
//  }
//  Serial.println("initialization done.");
//  // Determine file name for most recent file
//  // check to see if "full", otherwise continue
//  if (SD.exists("TLg999.txt")) {
//    return;
//  }
//  // for loop, check root.exists(filename), update filename, run till false
//  for (file_number=0; file_number <= 999; file_number++){
//      snprintf(name,sizeof(name),"TLg%03d.txt",file_number);
//      if (SD.exists(name)) {
//        continue;
//      }
//      else {
//        break;
//      }
//  }
//    // open the file.
//  logfile = SD.open(name, FILE_WRITE);
//  logfile.print("Initializing Complete\n");
//  logfile.close();
//}

void checkSRXL2() {
  currentTime = millis();

  static unsigned long prevSerialRxTime = 0;

  // UART receive buffer
  static uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
  static uint8_t rxBufferIndex = 0;

  if (currentTime - prevSerialRxTime > SRXL2_FRAME_TIMEOUT)
  {
    prevSerialRxTime = currentTime;
    rxBufferIndex = 0;
    srxlRun(0, SRXL2_FRAME_TIMEOUT);
  }

  if ( srxl2port.available() )
  {
    prevSerialRxTime = currentTime;
    unsigned char c = srxl2port.read(); // 
    rxBuffer[rxBufferIndex++] = c;
  }

  if (rxBufferIndex >= 5)
  {
    if(rxBuffer[0] == SPEKTRUM_SRXL_ID)
    {
      uint8_t packetLength = rxBuffer[2];
      if (rxBufferIndex >= packetLength)
      {
        // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
        if (srxlParsePacket(0, rxBuffer, packetLength))
        {
          // Move any remaining bytes to beginning of buffer (usually 0)
          rxBufferIndex -= packetLength;
          memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
        }
        else
        {
            rxBufferIndex = 0;
        }
      }
    }
  }
}

void defDefl_2_degServo(double *defl, double *serv) {
    /*
     * defl = array of 11 elements of the deg deflection values of L4, L3,
     *          L2, L1, L0, C, R0, R1, R2, R3, R4
     * serv = array of 11 elements of the deg servo values of L4, L3, L2,
     *          L1, L0, C, R0, R1, R2, R3, R4
    */
    int i;
    // L4
    serv[0] = 91.308+defl[0]*(1.3052+defl[0]*(0.0232+defl[0]*(0.0001-defl[0]*3.0e-5)));
    // L3
    serv[1] = 88.871+defl[1]*(1.564+defl[1]*(0.0031+defl[1]*(0.0003-defl[1]*2.0e-6)));
    // L2
    serv[2] = 88.573+defl[2]*(1.799+defl[2]*(0.0124-defl[2]*0.0003));
    // L1
    serv[3] = 90.036+defl[3]*(1.6049+defl[3]*(0.014+defl[3]*0.0004));
    // L0
    serv[4] = 91.241+defl[4]*(1.6469+defl[4]*(0.0089+defl[4]*0.0008));
    // C
    serv[5] = 92.255+defl[5]*(-2.3126+defl[5]*(0.0054-defl[5]*(0.0008-defl[5]*4.0e-5)));
    // R0
    serv[6] = 91.038+defl[6]*(-1.7425+defl[6]*(-0.012-defl[6]*0.0006));
    // R1
    serv[7] = 90.646+defl[7]*(-1.2843+defl[7]*(-0.0002-defl[7]*0.0008));
    // R2
    serv[8] = 89.657+defl[8]*(-1.7604+defl[8]*(-0.007-defl[8]*8.0e-5));
    // R3
    serv[9] = 90.819+defl[9]*(-1.296+defl[9]*(-0.0027-defl[9]*0.0008));
    // R4
    serv[10] = 90.431+defl[10]*(-1.101+defl[10]*(-0.0145+defl[10]*(-0.0004+defl[10]*1.0e-5)));
    
    for (i=0; i<11; i++) {
      if (i != 5) {
        if (serv[i] > 150.0) {serv[i] = 150.0;}
        if (serv[i] <  30.0) {serv[i] =  30.0;}
      } else {
        if (serv[i] > 160.0) {serv[i] = 160.0;}
        if (serv[i] <  20.0) {serv[i] =  20.0;}
      }
    }
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
  
  while(Pixport.available() > 0) { // as long as buffer size bigger than 0 -- pull out serial data
//    Serial.println("got data!");
    

    uint8_t c = Pixport.read(); // reads in the serial message
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        
        switch(msg.msgid) {
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
        case MAVLINK_MSG_ID_VFR_HUD:  // #74 VFR_HUDInitializing Complete
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
//            if (pilot.speed > TRANS_PWM_NOM + TRANS_PWM_NOISE) {
//              pix.airspeed = vfr_hud.groundspeed;  // m/s
//            }
//            else {
              pix.airspeed = vfr_hud.airspeed;  // m/s
//            }
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
            pix.time_msec = (int) attitude.time_boot_ms;
            // printVal("bank: ", attitude.roll);
            // printVal("elevation: ", attitude.pitch);
            
          }
          break;
        default: break;
      }
    }
  }
}

void Write2Card(){
//    int i;
//    logfile = SD.open(name, FILE_WRITE);
    
//    logfile.printf("%11" PRIu64 ", pilotCommands, %4u, %4u, %4u, %4u, pixhawk, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %5.3f, ",pix.time_usec,pilot.ail,pilot.ele,pilot.rud,pilot.modeSwitch, pix.airspeed, pix.climbRate, pix.bankAngle, pix.elevationAngle, pix.rollRate, dL->getAverage());
//    logfile.printf(" degrees, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, \n",deg[0],deg[1],deg[2],deg[3],deg[4],deg[5],deg[6],deg[7],deg[8],deg[9],deg[10]);
//  Serial.printf("%10u, pilotCommands, %4u, %4u, %4u, %4u, pixhawk, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %5.3f,",pix.time_msec,pilot.ail,pilot.ele,pilot.rud,pilot.modeSwitch);
  Serial.printf("pixhawk, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %5.3f,", pix.airspeed, pix.climbRate, pix.bankAngle, pix.elevationAngle, pix.rollRate, dL->getAverage());
  Serial.printf(" degrees, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, \n",deg[0],deg[1],deg[2],deg[3],deg[4],deg[5],deg[6],deg[7],deg[8],deg[9],deg[10]);
//  if (SD_card_not_initialized && currentTime > 5000) {
//    Serial.println("whee doggy!");
////    initializeSD();
//    SD_card_not_initialized = false;
//  }
}


///////////////////////// SRXL2 channel interface //////////////////////////////


 void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
 {
 // Get throttle channel value and convert to 1000 - 1500 - 2000 pwm range
  ThrustPWM = srxlChData.values[THRO] >> 5;     // 16-bit to 11-bit range (0 - 2048)
  ThrustPWM = int(double(ThrustPWM)*SRXL2scale) + SRXL2offset;
  // write directly to throttle
  Thrust.write(ThrustPWM);
  
  // Get Aile channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.ail = srxlChData.values[AILE] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.ail = int(double(pilot.ail)*SRXL2scale) + SRXL2offset;

   // Get elevator channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.ele = srxlChData.values[ELEV] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.ele = int(double(pilot.ele)*SRXL2scale) + SRXL2offset;

// Get rudder channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.rud = srxlChData.values[RUDD] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.rud = int(double(pilot.rud)*SRXL2scale) + SRXL2offset;

   // Get modeswitch channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.modeSwitch = srxlChData.values[MOSW] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.modeSwitch = int(double(pilot.modeSwitch)*SRXL2scale) + SRXL2offset;

   // Get bayswitch channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.baySwitch = srxlChData.values[BASW] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.baySwitch = int(double(pilot.baySwitch)*SRXL2scale) + SRXL2offset;

   // Get higher modeswitch channel value and convert to 1000 - 1500 - 2000 pwm range
  pilot.higherModeSwitch = srxlChData.values[HMSW] >> 5;    // 16-bit to 11-bit range (0 - 2048)
  pilot.higherModeSwitch = int(double(pilot.higherModeSwitch)*SRXL2scale) + SRXL2offset;
 }

 void uartSetBaud(uint8_t uart, uint32_t baudRate) // Automatic adjust SRXL2 baudrate. 
 {
  // Not supported yet
 }

 void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length)
 {
  for (uint8_t i=0; i < length; i++)
  {
    srxl2port.write(pBuffer[i]);
  }
  srxl2port.flush();
 }
 

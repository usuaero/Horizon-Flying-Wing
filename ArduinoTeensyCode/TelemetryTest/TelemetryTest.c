// External Libraries - mavlink
#include "ardupilotmega/mavlink.h"

//create mavlink.h file in main folder (c_library_v2-master) --> write #include ardupilotmega/mavlink.h

//either edit path or keep main folder in same directory as code with path to the mavlink.h header file (see above)

//hardwire: TX <-> Rx and vice versa -- see links I sent Ben -- currently plugged into TX1 & RX1 on Teensy

void setup() {
  // Set serial rates
  Serial1.begin(57600);    // TELEM2 from Pixhawk

  Serial.print("Setup Complete\n");
  //delay(10000);
}

void loop() {
  //Serial.println("in loop");
  // put your main code here, to run repeatedly:

    comm_receive();
    delay(1);
}

void comm_receive() {

  //variables needed -- airspeed, climb rate, bank angle, roll rate
    
  // variables to define (globally?) 
  double roll; // bank angle
  double airspeed;
  double climb_rate;
  double roll_rate;
  
  
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(Serial1.available() > 0) { // as long as buffer size bigger than 0 -- pull out serial data

    uint8_t c = Serial1.read(); // reads in the serial message
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        
        switch(msg.msgid) {
      
        default: break;
        
        // IMU data -- roll & yaw rates
        case MAVLINK_MSG_ID_RAW_IMU:  // #105 highres IMU / Check scaled IMU for acutal value readings 
          {
          
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);

            roll_rate = raw_imu.xgyro;
            Serial.print("Roll Rate: ");
            Serial.print(roll_rate);
            Serial.print("\n");

          }
          break;

        // airspeed 
        case MAVLINK_MSG_ID_VFR_HUD:  // #74 VFR_HUD
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);

            airspeed = vfr_hud.airspeed;  // m/s
            climb_rate = vfr_hud.climb;  // m/s

          }
          break;

        // attitude data -- roll angles (bank angle)
        case MAVLINK_MSG_ID_ATTITUDE:  // #30 ATTITUDE
          {

            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            roll = attitude.roll;  // rad (-pi..+pi)

          }
          break;      
      }
    }
  }
}

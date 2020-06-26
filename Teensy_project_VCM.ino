/*
Hardware notes:
This is designed to work on a teensy 4.1 board. a 4.0 doesnt have enough inputs and outputs.


canbus 1 rx           // dedicated        PIN # 0  CRX2
canbus 1 tx           // dedicated        PIN # 1  CTX2
Shifter_a             // digital input    PIN # 2  DI
Shifter_b             // digital input    PIN # 3  DI
Shifter_c             // digital input    PIN # 4  DI
Shifter_d             // digital input    PIN # 5  DI
Shifter_e             // digital input    PIN # 6  DI
Shifter_f             // digital input    PIN # 7  DI
Shifter_g             // digital input    PIN # 8  DI
Shifter_h             // digital input    PIN # 9  DI
Coolant pump 1(battery)   // PWM output   PIN # 10 PWM
Coolant pump 2(drive unit)// PWM output   PIN # 11 PWM
Coolant pump 3(radiator)  // PWM output   PIN # 12 PWM
led                   // digital output   PIN # 13 DO
Coolant mode Pos      // analog input     PIN # 14 A0
Coolant batt Pos      // analog input     PIN # 15 A1
ESP 8622 RX/SCL       // dedicated        PIN # 16 RX4/SCL1
ESP 8622 TX/SDA       // dedicated        PIN # 17 TX4/SDA1
Accelerometer SDA     // dedicated        PIN # 18 SDA
Accelerometer SCL     // dedicated        PIN # 19 SCL
A/C High pressure     // analog input     PIN # 20 A6
A/C Low pressure      // analog input     PIN # 21 A7
canbus 0 tx           // dedicated        PIN # 22 CTX1
canbus 0 rx           // dedicated        PIN # 23 CRX1
condensor temp        // analog input     PIN # 24 A10
evap temp             // analog input     PIN # 25 A11
ambient temp          // analog input     PIN # 26 A12
accy voltage          // analog input     PIN # 27 A13
Coolant mode driver   // PWM output       PIN # 28 PWM
Coolant battery driver// PWM output       PIN # 29 PWM
canbus 2 rx           // dedicated        PIN # 30 CRX3
canbus 2 tx           // dedicated        PIN # 31 CTX3
dc_dc_hv_on           // digital output   PIN # 32 DO
car_on_override       // digital output   PIN # 33 DO
accessory_hv_on       // digital output   PIN # 34 DO
A/C Requested         // digital input    PIN # 35 DI
Coolant fan 1         // PWM output       PIN # 36 PWM
Coolant fan 2         // PWM output       PIN # 37 PWM
aps1                  // analog input     PIN # 38 A14
aps2                  // analog input     PIN # 39 A15
rps1                  // analog input     PIN # 40 A16
rps2                  // analog input     PIN # 41 A17

Extended I/O
m_stby                // digital output   PIN # 0,  GPA0 DO
m1_in1                // digital output   PIN # 1,  GPA1 DO                                   
m1_in2                // digital output   PIN # 2,  GPA2 DO
m2_in1                // digital output   PIN # 3,  GPA3 DO
m2_in2                // digital output   PIN # 4,  GPA4 DO
unused                //                  PIN # 5,  GPA5
unused                //                  PIN # 6,  GPA6
shiftlock             // digital ouput    PIN # 7,  GPA7 DO
cruise_on             // digital input    PIN # 8,  GPB0 DI
cruise_accel          // digital input    PIN # 9,  GPB1 DI
cruise_coast          // digital input    PIN # 10, GPB2 DI
brake_in              // digital input    PIN # 11, GPB3 DI
awake_out             // digital output   PIN # 12, GPB4 DO
unused                //                  PIN # 13, GPB5
unused                //                  PIN # 14, GPB6                                                                     
unused                //                  PIN # 15, GPB7


2000ms
self resetting faults (comms)

SOC table
I created a lookup table that references pack voltage, and amperage. It looks up an appropriate SOC based on that. Its not great, its not coulomb counting or anything,
but I wanted to see how functional it could be, and help find the voltage drop under load for the full range of SOC's. fascinating stuff.

coulomb counting
Size of battery in KWH / nominal voltage = approximate amp/hours. AH*3600 = coulombs. Coulomb's are 1 amp per second.
So we can take estimated SOC from the average cell voltage. Figure out how many coulombs we have left and approximate the discharge from there
I'm thinking... we can read in the amperage continually, have each value populate an array, average the array over 1 second and decrement that from the estimated columbs we started with.
Rectification of disparity? coulomb counting should be more accurate than the V-A SOC chart, but if thats dialed in things will be better overall. So many factors. temperature, voltages, load.
lets setup a display that keeps track of them independantly and the difference between them to dial in one another. if we can nail the near zero column on the chart, and the temperature compensation,
we can adjust the nominal voltage of the battery to align the coulomb counter with the SOC chart. The goal would be to get it accurate enough where the disparity reading is minimal.
This will keep the inital SOC from chart reading from suddenly changing displayed SOC when we reboot.

traction control
I added comms to talk to the drive unit "DU" I plan on using a 6 axis accelerometer to compare to the RPM of the drive unit, and trim back the pot (accelerator) signal to the DU to implement traction control.
This means I need several analog inputs, two for the accelerator signal, which is 5V based, and also I need to read in the accelerometer, factor out gravity, Since all accelerometers suffer from bias and drift,
we will use any period of low acceleration to match the current interpreted velocity of the accelerator to the current velocity based on the motor speed. I'm also thinking about using GPS to fine tune the speed variables as well.
Also, we'll include regen in this scheme.

*/

#include <FlexCAN_T4.h>       // Library for Can on Teensy
#include <Wire.h>             // I2C comms library
#include <Adafruit_MCP23017.h>// Library for using MCP23017 I/O extender
#include "Teensy_Project.h"   // include header file
#include "SdFat.h"            // library for micro SD
#include <TimeLib.h>          // Real Time Clock


//borrowed this stuff from SdFat-Beta, Thanks Bill Greiman!
#define SD_FAT_TYPE 3         // FAT16, FAT32, EXFAT SD formats supported
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

#define NUM_TX_MAILBOXES 4
#define NUM_RX_MAILBOXES 15

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

Adafruit_MCP23017 mcp;

void setup()
{
  Serial.begin(115200); delay(400);
  mcp.begin(7);      // use default address 0

  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  
  analogReadResolution(12);   // (0-4095)
   
  //pin mode setups
  pinMode(shifter_a, INPUT);
  pinMode(shifter_b, INPUT);
  pinMode(shifter_c, INPUT);
  pinMode(shifter_d, INPUT);
  pinMode(shifter_e, INPUT);
  pinMode(shifter_f, INPUT);
  pinMode(shifter_g, INPUT);
  pinMode(shifter_h, INPUT);
  pinMode(coolant_pump_batt_pin, OUTPUT);
  pinMode(coolant_pump_drive_unit_pin, OUTPUT);
  pinMode(coolant_pump_radiator_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(an_coolant_diverter_mode, INPUT);
  pinMode(an_coolant_diverter_batt, INPUT);  
  pinMode(an_ac_high_pressure, INPUT);
  pinMode(an_ac_low_pressure, INPUT);
  pinMode(an_condensor, INPUT);
  pinMode(an_evaporator, INPUT);
  pinMode(an_ambient, INPUT);
  pinMode(an_acc_voltage, INPUT);
  pinMode(coolant_diverter_mode_pin, OUTPUT);
  pinMode(coolant_diverter_batt_pin, OUTPUT);
  pinMode(car_on_override_pin, OUTPUT);
  pinMode(dc_dc_power_on_pin, OUTPUT);
  pinMode(ac_requested_pin, INPUT);
  pinMode(coolant_fan_1_pin, OUTPUT);
  pinMode(coolant_fan_2_pin, OUTPUT);
  pinMode(an_aps1, INPUT);
  pinMode(an_aps2, INPUT);
  pinMode(an_rps1, INPUT);
  pinMode(an_rps2, INPUT);
  
  //mcp23017 pin mode setups
  mcp.pinMode(m_stby_pin, OUTPUT);
  mcp.pinMode(mode_in1_pin, OUTPUT);
  mcp.pinMode(mode_in2_pin, OUTPUT);
  mcp.pinMode(batt_in1_pin, OUTPUT);
  mcp.pinMode(batt_in2_pin, OUTPUT);
  mcp.pinMode(shiftlock_pin, OUTPUT);
  mcp.pinMode(cruise_on_pin, INPUT);
  mcp.pinMode(cruise_accel_pin, INPUT);
  mcp.pinMode(cruise_coast_pin, INPUT);
  mcp.pinMode(brake_vcm_pin, INPUT);
  mcp.pinMode(awake_out_pin, OUTPUT);  
  
  setup_accelerometer();      // setup MCP_6050, on Drive_Unit_control tab
  setup_coulomb_counter();    // setup initial coulomb counter stuff on Battery_Balancing tab
  setup_sensor_limits();      // setup APS and RPS sensors
  setup_thermal_management(); // setup thermal management stuff
  setup_file_time_and_logs();      // setup file and time stuff
  
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can0.setMB(i,RX,STD);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can0.setMB(i,TX,STD);
  }
   
  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  Can0.setMBFilter(MB0, 0x200);                       // Set mailbox 0 to allow CAN ID 0x200 to be collected.
  Can0.setMBFilter(MB1, 0x202);                       // Set mailbox 1 to allow CAN ID 0x202 to be collected.
  Can0.setMBFilter(MB2, 0x204);                       // Set mailbox 2 to allow CAN ID 0x204 to be collected.
  Can0.setMBFilter(MB3, 0x206);                       // Set mailbox 3 to allow CAN ID 0x206 to be collected.
  Can0.setMBFilter(MB4, 0x210);                       // Set mailbox 4 to allow CAN ID 0x210 to be collected.
  Can0.setMBFilter(MB5, 0x302);                       // Set mailbox 5 to allow CAN ID 0x302 to be collected.
  Can0.setMBFilter(MB6, 0x102);                       // Set mailbox 6 to allow CAN ID 0x102 to be collected.
  Can0.setMBFilter(MB7, 0x103);                       // Set mailbox 7 to allow CAN ID 0x103 to be collected.
  Can0.setMBFilter(MB8, 0x104);                       // Set mailbox 8 to allow CAN ID 0x105 to be collected.
  Can0.setMBFilter(MB9, 0x1D6);                       // Set mailbox 9 to allow CAN ID 0x1D6 to be collected.
  Can0.setMBFilter(MB10, 0x410);                      // Set mailbox 9 to allow CAN ID 0x410 to be collected.
  Can0.onReceive(MB0, parse_pack0_200);                 // Go parse frame data
  Can0.onReceive(MB1, parse_pack0_202);
  Can0.onReceive(MB2, parse_pack0_204);
  Can0.onReceive(MB3, parse_pack0_206);
  Can0.onReceive(MB4, parse_pack0_210);
  Can0.onReceive(MB5, parse_pack0_302);
  Can0.onReceive(MB6, parse_du_102); 
  Can0.onReceive(MB7, parse_du_103); 
  Can0.onReceive(MB8, parse_du_104);
  Can0.onReceive(MB9, parse_apm_1d6);
  Can0.onReceive(MB10, parse_charger_410);
  Can0.mailboxStatus();
  
  //Can1 
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can1.setMB(i,RX,STD);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can1.setMB(i,TX,STD);
  }
   
  //Can1.setMBFilter(REJECT_ALL);
  //Can1.enableMBInterrupts();
  /*Can1.setMBFilter(MB0, 0x200);                       // Set mailbox 0 to allow CAN ID 0x200 to be collected.
  Can1.setMBFilter(MB1, 0x202);                       // Set mailbox 1 to allow CAN ID 0x202 to be collected.
  Can1.setMBFilter(MB2, 0x204);                       // Set mailbox 2 to allow CAN ID 0x204 to be collected.
  Can1.setMBFilter(MB3, 0x206);                       // Set mailbox 3 to allow CAN ID 0x206 to be collected.
  Can1.setMBFilter(MB4, 0x200);                       // Set mailbox 4 to allow CAN ID 0x210 to be collected.
  Can1.setMBFilter(MB5, 0x302);                       // Set mailbox 5 to allow CAN ID 0x302 to be collected.
  Can1.onReceive(MB0, parse_pack1_200);                 // Go parse frame data
  Can1.onReceive(MB1, parse_pack1_202);
  Can1.onReceive(MB2, parse_pack1_204);
  Can1.onReceive(MB3, parse_pack1_206);
  Can1.onReceive(MB4, parse_pack1_210);
  Can1.onReceive(MB5, parse_pack1_302);
  Can1.mailboxStatus();
  */
  //Can2
  Can2.begin();
  Can2.setBaudRate(500000);
  Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can2.setMB(i,RX,STD);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can2.setMB(i,TX,STD);
  }
   
  Can2.setMBFilter(REJECT_ALL);
  Can2.enableMBInterrupts();
  /*Can2.setMBFilter(MB0, 0x200);                       // Set mailbox 0 to allow CAN ID 0x200 to be collected.
  Can2.setMBFilter(MB1, 0x202);                       // Set mailbox 1 to allow CAN ID 0x202 to be collected.
  Can2.setMBFilter(MB2, 0x204);                       // Set mailbox 2 to allow CAN ID 0x204 to be collected.
  Can2.setMBFilter(MB3, 0x206);                       // Set mailbox 3 to allow CAN ID 0x206 to be collected.
  Can2.setMBFilter(MB4, 0x210);                       // Set mailbox 4 to allow CAN ID 0x210 to be collected.
  Can2.setMBFilter(MB5, 0x302);                       // Set mailbox 5 to allow CAN ID 0x302 to be collected.
  Can2.onReceive(MB0, parse_pack1_200);                 // Go parse frame data
  Can2.onReceive(MB1, parse_pack1_202);
  Can2.onReceive(MB2, parse_pack1_204);
  Can2.onReceive(MB3, parse_pack1_206);
  Can2.onReceive(MB4, parse_pack1_210);
  Can2.onReceive(MB5, parse_pack1_302);
  */
  Can2.mailboxStatus();
}// End of Setup

void loop()                          // main loop.
{                                    // you can always comment out various functions to stop them from running

  Can0.events();
  Can1.events();
  
  // Timers in main loop
  
  currentMillis = millis();                                              // jot the current millis down for comparison
 
  if(currentMillis - previous_010_timer >= timer_010_millis)             // if 10 or more ms have elapsed since the last run, reset the timer and run 010 functions.
  {
    previous_010_timer = currentMillis;                                  // reset the timer to current millis
    loop010();                                                           // do 010ms functions.
  }  

  currentMillis = millis();                                              // jot the current millis down for comparison
 
  if(currentMillis - previous_020_timer >= timer_020_millis)             // if 20 or more ms have elapsed since the last run, reset the timer and run 020 functions.
  {
    previous_020_timer = currentMillis;                                  // reset the timer to current millis
    loop020();                                                           // do 020ms functions.
  }  

  currentMillis = millis();                                              // jot the current millis down for comparison
 
  if(currentMillis - previous_250_timer >= timer_250_millis)             // if 250 or more ms have elapsed since the last run, reset the timer and run 250 functions.
  {
    previous_250_timer = currentMillis;                                  // reset the timer to current millis
    loop250();                                                           // do 250ms functions.
  }  

  currentMillis = millis();                                              // jot the current millis down for comparison
 
  if(currentMillis - previous_1000_timer >= timer_1000_millis)           // if 1000 or more ms have elapsed since the last run, reset the timer and run 1000 functions.
  {
    previous_1000_timer = currentMillis;                                 // reset the timer to current millis
    loop1000();                                                          // do 1000ms functions.
  }  

  currentMillis = millis();                                              // jot the current millis down for comparison
 
  if(currentMillis - previous_2000_timer >= timer_2000_millis)           // if 2000 or more ms have elapsed since the last run, reset the timer and run 2000 functions.
  {
    previous_2000_timer = currentMillis;                                 // reset the timer to current millis
    loop2000();                                                          // do 2000ms functions.
  }  

  currentMillis = millis();                                              // jot the current millis down for comparison

  if(pack0_comms_timeout_trace > 0)                                      // if any comms have flagged as recieved...
  {
    pack0_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
  }
  if(currentMillis - pack0_comms_timeout_clock >= pack_comms_timeout)    // if more than the "timeout" ms have elapsed since the last run, reset the timer and run reset functions.
  {
    pack0_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
    pack0_reset_incoming_data();                                         // reset incoming data.
  }  

 currentMillis = millis();                                               // jot the current millis down for comparison

  if(pack1_comms_timeout_trace > 0)                                      // if any comms have flagged as recieved...
  {
    pack1_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
  }
  if(currentMillis - pack1_comms_timeout_clock >= pack_comms_timeout)    // if more than the "timeout" ms have elapsed since the last run, reset the timer and run reset functions.
  {
    pack1_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
    pack1_reset_incoming_data();                                         // reset incoming data.
  }  

}                                    // end of main loop

void loop010()                       // This function called every 10mS
{
  shifter_pos();
  du_calcs();
  du_outgoing_message();
  du_accelerometer_read();
  du_traction_control();
 
}                                    // end of loop010

void loop020()                       // This function called every 20mS
{
  output_balancing_can();            //output balancing data to both packs
  if (master_reset)
  {
    execute_master_reset();            // master reset
  }
  shifter_lock();                    // operate shifter lock
  cruise_control();                  // do cruise control stuff
  coolant_diverter_control();         // do coolant diverter stuff
  digitalWrite(led_pin, !digitalRead(led_pin)); // toggle LED by inverting whats read from it
}                                    // end of loop020

void loop250()                       // This function called every 250mS
{
  fault_logic();                     // do simple fault logic
  apm_outgoing_message();            // send can comms to the auxiliary power module (DC DC converter)
  charger_outgoing_message();        // send can comms to the charger
  battery_statistics();              // calculate some battery statistics from the incoming data
  balancing_determination();         // figure out which cells need to be balanced, and if there are any WAY out of balance
  balancing_state();                 // count the quantity of cells balancing, and set the balance state accordingly
  if (pack0_enabled)                 // check if pack0 is enabled
  {
    pack0_balance_output_sequencer();//runs determined balance cells through a sequencer to determine the appropriate outputs
  }
  if (pack1_enabled)                 // check if pack1 is enabled
  {
    pack1_balance_output_sequencer();// runs determined balance cells through a sequencer to determine the appropriate outputs
  }
  check_for_shutdown();              // save data before shutdown File_management tab.
  
}                                    // end of 250ms loop

void loop1000()                      // This function called every 1000mS
{
  coulomb_SOC();                     // run coulomb SOC calc 
  if (log_drive_unit_comms)              
  {
    drive_unit_log();                // log drive unit data
  }
  if (log_cell_statistics)           // if cell stats logging...
  {
    cell_statistics();               // go log them
  }
  
  if (log_pack0_cell_voltages)       // if battery cell debug is enabled...
  {
    pack0_cell_log();                // do cell output sequence (file management tab)
  }
  if (log_pack1_cell_voltages)       // if battery cell debug is enabled...
  {
    pack1_cell_log();                // do cell output sequence (file management tab)
  }
  if (log_accelerator_positions)     // if log accelerator positions is true...
  {
    accelerator_pos_log();           // do accelerator position logging
  }
  if (log_pack_data)                 // if log pack data is enabled... 
  {
    pack_data_log()     ;             // do pack data logging
  }
  pack_comms_trace_reset();          // reset comms trace, will initiate pack incoming data resets if they don't start flagging as read
} // end of loop1000

void loop2000()                      // This function called every 2000mS
{
  restore_resettables();             // restore fault variables that can self heal
  coolant_fan_pump_output();         // sets PWM's for the fans and the pumps.
  
} // end of loop2000

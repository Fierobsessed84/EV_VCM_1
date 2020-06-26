/*  This tab contains the functions relating to the control of the drive unit. this includes the shifter, accelerator pedal, regen sensor,
    traction control, accelerometers, and error checking

THINGS THAT NEED WORK:
traction control 
fault tolerant aps/rps handeling
modeling different aps/rps modes
*/
  
void setup_accelerometer()
{
// MCP_6050 6 axis accelerometer settings https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000);    //I2C address of the MPU
  Wire.write(0x1a);                     //Accessing the register 1a - Configuration (Sec. 4.3)
  Wire.write(0x00000100);               //Setting the digital low Pass Filter to 8.4ms
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);    //I2C address of the MPU
  Wire.write(0x1b);                     //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0b00001000);               //Setting the gyro to full scale +/- 500deg./s                                                                
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);    //I2C address of the MPU
  Wire.write(0x1c);                     //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00010000);               //Setting the accel to +/- 8g                                                                   
  Wire.endTransmission();
    
  //Wire.beginTransmission(0b1101000);    //I2C address of the MPU
  //Wire.write(0x6b);                     //Accessing the register 6b - PWR MGMT (Sec. 4.3)
  //Wire.write(0b10000000);               //reset device (comment this section out unless you have issues                                                                  
  //Wire.endTransmission();   
}

void setup_sensor_limits()
// sets limits based on fault tolerance percentages, on what it will allow the range of the sensors to be set to if the analog signal violates these constraints, the signal to the DU is zeroed out.
{
  aps1_high_limit = constrain(aps1_high + ((aps1_high - aps1_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  aps1_low_limit = constrain(aps1_low - ((aps1_high - aps1_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  aps2_high_limit = constrain(aps2_high + ((aps2_high - aps2_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  aps2_low_limit = constrain(aps2_low - ((aps2_high - aps2_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  rps1_high_limit = constrain(rps1_high + ((rps1_high - rps1_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  rps1_low_limit = constrain(rps1_low - ((rps1_high - rps1_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  rps2_high_limit = constrain(rps2_high + ((rps2_high - rps2_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);
  rps2_low_limit = constrain(rps2_low - ((rps2_high - rps2_low) * (aps_rps_fault_tolerance / 100)), 0, 4095);


}    
void parse_du_102(const CAN_message_t &msg)
{
  udc   = (msg.buf[0] + msg.buf[1] * 256);            // drive unit DC voltage
  idc   = (msg.buf[2] + msg.buf[3] * 256);            // drive unit DC Amperage
  speed = (msg.buf[4] + msg.buf[5] * 256);            // drive unit Speed RPM
}    
void parse_du_103(const CAN_message_t &msg)
{
  tmphs   = (msg.buf[0] + msg.buf[1] * 256);          // Temperature heat sink C
  tmpm    = (msg.buf[2] + msg.buf[3] * 256);          // temperature Motor C
}    
void parse_du_104(const CAN_message_t &msg)
{
  dir    = msg.buf[0];                                // direction of drive unit
  opmode = msg.buf[1];                                // operator mode
}

void du_calcs()
{
  // figure out what the DU outputs need to be 
  canio = cruise + (du_start * 2) + (brake * 4) + (forward * 8) + (reverse * 16) + (bms * 32); // compact these bools into a byte
  canio_old = canio;                                                                           // save a copy for display and logging

  // GM pedal: A Ground, B sweep, C 5V only, D Ground, E sweep, F 5V only. Use a voltage divider on pin E, 10k to pin, 33k to ground.
  // As discovered range, APS1 without any division 475 minimum, 550-570 typical, 2750 high.
  // As discovered range, APS2 with division 680 minimum, 790-810 typical, 4095 high. It just kisses 3.3V with the divider

  cruise_on = mcp.digitalRead(cruise_on_pin);         // 
  cruise_accel = mcp.digitalRead(cruise_accel_pin);   //
  cruise_coast = mcp.digitalRead(cruise_coast_pin);   //
  brake_vcm = mcp.digitalRead(brake_vcm_pin);         //
  aps1 = int(analogRead(an_aps1));                    //
  aps2 = int(analogRead(an_aps2));                    //
  rps1 = int(analogRead(an_rps1));                    //
  rps2 = int(analogRead(an_rps2));                    //
  
  // Sensor reading - low value offset * ((1000 + deadband * 2) / sensor range) - deadband
  normalized_aps1 = constrain((((aps1 - aps1_low) * (1000 + (aps_rps_deadband * 20)) / float(aps1_high - aps1_low)) - (aps_rps_deadband * 10)), 0, 1000); // change raw aps1 signal into a 0 - 1000 signal, that can't leave that range, including invisible deadband
  normalized_aps2 = constrain((((aps2 - aps2_low) * (1000 + (aps_rps_deadband * 20)) / float(aps2_high - aps2_low)) - (aps_rps_deadband * 10)), 0, 1000); // change raw aps1 signal into a 0 - 1000 signal, that can't leave that range, including invisible deadband
  normalized_rps1 = constrain((((rps1 - rps1_low) * (1000 + (aps_rps_deadband * 20)) / float(rps1_high - rps1_low)) - (aps_rps_deadband * 10)), 0, 1000); // change raw aps1 signal into a 0 - 1000 signal, that can't leave that range, including invisible deadband
  normalized_rps2 = constrain((((rps2 - rps2_low) * (1000 + (aps_rps_deadband * 20)) / float(rps2_high - rps2_low)) - (aps_rps_deadband * 10)), 0, 1000); // change raw aps1 signal into a 0 - 1000 signal, that can't leave that range, including invisible deadband

  // This will extend the sensor settings beyond the initial calibration.
  if (aps1 < aps1_low)                              // if aps1 goes below the bottom of its allowable range...
  {
    aps1_low = aps1;                                // stretch the allowable range
  }

  if (aps1 > aps1_high)                              // if aps1 goes above the top of its allowable range...
  {
    aps1_high = aps1;                                // stretch the allowable range
  }

  if (aps2 < aps2_low)
  {
    aps2_low = aps2;
  }

  if (aps2 > aps2_high)
  {
    aps2_high = aps2;
  }

  if (rps1 < rps1_low)
  {
    rps1_low = rps1;
  }

  if (rps1 > rps1_high)
  {
    rps1_high = rps1;
  }

  if (rps2 < rps2_low)
  {
    rps2_low = rps2;
  }

  if (rps2 > rps2_high)
  {
    rps2_high = rps2;
  }
  
  // fault checking range
  if (aps1 > aps1_high_limit || aps1 < aps1_low_limit || aps2 > aps2_high_limit || aps2 < aps2_low_limit)
  {
    aps_fault = true;
  }
  
  if (rps1 > rps1_high_limit || rps1 < rps1_low_limit || rps2 > rps2_high_limit || rps2 < rps2_low_limit)
  {
    rps_fault = true;
  }
  
  // fault checking redundance
 // if ((abs(normalized_aps1 - normalized_aps2) / 10) > aps_rps_fault_tolerance)
 // {
 //   aps_fault = true;
 // }
  
  if ((abs(normalized_rps1 - normalized_rps2) / 10) > aps_rps_fault_tolerance)
  {
    rps_fault = true;
  }
  

  // No acceleration if aps is in fault, if brake pressed, if shifter is in neutral or left shift. no deceleration if rps is in fault. otherwise, average the normalized inputs and send them out, with traction control reductions factored in.
  if (aps_fault || brake || shift_state == 3 || shift_state == 1)
  {
    pot = 0;
    pot_old = 0; // for diagnostic purposes, not sent to the DU
  }
  else
  {
    pot = max((((normalized_aps1 + normalized_aps2) / 2) - traction_control_aps_reduction), (cruise_aps_val - traction_control_aps_reduction));
    pot_old = max((((normalized_aps1 + normalized_aps2) / 2) - traction_control_aps_reduction), (cruise_aps_val - traction_control_aps_reduction)); // for diagnostic purposes, not sent to the DU
  }

  if (rps_fault)
  {
    pot2 = 0;
    pot2_old = 0; // for diagnostic purposes, not sent to the DU
  }
  else
  {
    pot2 = max((((normalized_rps1 + normalized_rps2) / 2) - traction_control_rps_reduction), (cruise_rps_val - traction_control_rps_reduction));
    pot2_old = max((((normalized_rps1 + normalized_rps2) / 2) - traction_control_rps_reduction), (cruise_rps_val - traction_control_rps_reduction)); // for diagnostic purposes, not sent to the DU
  }
  
} // end du calcs

void cruise_control()
{
  if (brake_vcm = 0 || shift_state <= 3) // if brake is pressed, or shifter is in left shift, reverse, or neutral, cruise is off
  {
    if (cruise_on)
    {
      // do cruise stuff here.
    }
    else
    {
      cruise_aps_val = 0;
      cruise_rps_val = 0;
    }
  }
  else
  {
    cruise_aps_val = 0;
    cruise_rps_val = 0;
  }
}

void du_outgoing_message()
{  
  CAN_message_t msg;
  msg.id = 0x100;
  msg.flags.extended = 0;
  msg.len = 8;
  msg.buf[0] = canio;                // Canio, cpmpacted byte
   msg.buf[4] = pot & 0x00ff;         // throttle pot 1 LSB
  msg.buf[5] = int(pot / 256);       // throttle pot 1 MSB
  msg.buf[6] = pot2 & 0x00ff;        // regen pot 2 LSB
  msg.buf[7] = int(pot2 / 256);      // regen pot 2 MSB
  
  Can0.write(msg);                   // send that to the DU!
  
  canio = 0x00;                                 // Set all DU outputs to a safe state. All inputs need to be re-calculated to send next update.
  cruise   = 0;                                
  du_start = 0;                                
  brake    = 0;                               
  forward  = 0;                              
  reverse  = 0;                           
  bms      = 0;                                
  pot      = 0;                                   
  pot2     = 0;                                   
}

void du_accelerometer_read()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  short raw_X_axis_accel=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  short raw_Y_axis_accel=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  short raw_Z_axis_accel=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  short raw_tmp         =Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  short raw_roll        =Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  short raw_pitch       =Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  short raw_yaw         =Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Accelerometer calibration. 32768 / limit of units desired gives you the divisor IE, 32768 / 8G's gices us 4096. or 32768 / 500deg/s = 65.536.
  X_axis_accel = raw_X_axis_accel / 40.96;          //gives us G's X100
  Y_axis_accel = raw_Y_axis_accel / 40.96;          //gives us G's X100
  Z_axis_accel = raw_Z_axis_accel / 40.96;          //gives us G's X100
  tmp          = raw_tmp          / 340.00 + 36.53;
  roll         = raw_roll         / 65.536;          //gives us deg/s
  pitch        = raw_pitch        / 65.536;          //gives us deg/s
  yaw          = raw_yaw          / 65.536;          //gives us deg/s
}

void shifter_pos()
{
  int shift_state_old = shift_state;
  
  if (digitalRead(shifter_a) && !digitalRead(shifter_b) && digitalRead(shifter_c) && digitalRead(shifter_d) && digitalRead(shifter_e) && !digitalRead(shifter_f) && !digitalRead(shifter_g) && !digitalRead(shifter_h))
  {
    shift_state = 1; // leftshift
  }
  else if (digitalRead(shifter_a) && !digitalRead(shifter_b) && !digitalRead(shifter_c) && digitalRead(shifter_d) && digitalRead(shifter_e) && digitalRead(shifter_f) && !digitalRead(shifter_g) && !digitalRead(shifter_h))
  {
    shift_state = 2; // reverse
  }
  else if (digitalRead(shifter_a) && !digitalRead(shifter_b) && digitalRead(shifter_c) && !digitalRead(shifter_d) && digitalRead(shifter_e) && !digitalRead(shifter_f) && digitalRead(shifter_g) && !digitalRead(shifter_h))
  {
    shift_state = 3;// neutral
  }
  else if (!digitalRead(shifter_a) && digitalRead(shifter_b) && digitalRead(shifter_c) && digitalRead(shifter_d) && !digitalRead(shifter_e) && !digitalRead(shifter_f) && !digitalRead(shifter_g) && digitalRead(shifter_h))
  {
    shift_state = 4; // drive
  }
  else if (!digitalRead(shifter_a) && !digitalRead(shifter_b) && digitalRead(shifter_c) && digitalRead(shifter_d) && digitalRead(shifter_e) && !digitalRead(shifter_f) && !digitalRead(shifter_g) && digitalRead(shifter_h))
  {
    shift_state = 5; // upshift
  }
  else if (digitalRead(shifter_a) && digitalRead(shifter_b) && digitalRead(shifter_c) && digitalRead(shifter_d) && !digitalRead(shifter_e) && !digitalRead(shifter_f) && !digitalRead(shifter_g) && !digitalRead(shifter_h))
  {
    shift_state = 6; // downshift
  }
  else if (digitalRead(shifter_a) && !digitalRead(shifter_b) && digitalRead(shifter_c) && digitalRead(shifter_d) && !digitalRead(shifter_e) && !digitalRead(shifter_f) && !digitalRead(shifter_g) && digitalRead(shifter_h))
  {
    shift_state = 7; //rightshift
  }
  else
  {
    shift_state = 0;
  }
  if (shift_state_old != shift_state)
  {
    Serial.print("Shift State Changed!");
    Serial.println(shifter[shift_state]);
    printNow();                                // time stamp
    file.print(",Shift State,");               // print category
    file.print(shifter[shift_state]);          // print shift state
  }
  switch (shift_state)
  {
    case 0: // invalid
    forward = 0;
    reverse = 0;
    break;
    
    case 1: // leftshift
    // we'll find a use for this later...
    break;
    
    case 2: // reverse
    forward = 0;
    reverse = 1;
    if (du_start == 0)
    {
      du_start = 1;
    }
    break;
    
    case 3: // neutral
    forward = 0;
    reverse = 0;
    break;
    
    case 4: // drive
    forward = 1;
    reverse = 0;
    if (du_start == 0)
    {
      du_start = 1;
    }
    break;
    
    case 5: // upshift
    // we'll find a use for this later...
    break;
    
    case 6: // downshift
    // we'll find a use for this later...
    break;
    
    case 7: // rightshift
    // we'll find a use for this later...
    break;
  }                             // end switch shift_state
}                               // end shifter_pos

void shifter_lock()
{
  if (brake_vcm != 0 && shift_state == 3) // if we're in neutral, and the brake is pressed, ok to release shift lock.
  {
    mcp.digitalWrite(shiftlock_pin, HIGH);   
  }
  else
  {
    mcp.digitalWrite(shiftlock_pin, LOW);
  }

}

void du_traction_control()
{

}

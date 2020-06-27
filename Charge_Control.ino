/*  This tab contains the functions relating to the control of the Charger unit. this includes handeling the charge port lights, latch enable, charger enable,
 *  amp maximums, DC/DC converter wake and settings and disabling the DU.
 *  Still need to do charger stuff at some point soon. 
 */
void parse_apm_1d6(const CAN_message_t &msg)
{
  apm_output_v  = (msg.buf[2] / 1.27);           // auxiliary power module voltage output
  apm_temp_1    = (msg.buf[3] - 40);             // auxiliary power module temperature 1, in C
  apm_temp_2    = (msg.buf[4] - 40);             // auxiliary power module temperature 2, in C
  apm_output_a  = msg.buf[5];                    // auxiliary power module amperage output
}

void apm_outgoing_message()
{
  if (!scheduler(12)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  apm_voltage_req = 135;                         // for testing we're setting it to 13.5 volts, arbitrarily  
  CAN_message_t msg;
  msg.id = 0x1D4;                                // output message ID for the auxiliary power module
  msg.flags.extended = 0;
  msg.len = 2;
  msg.buf[0] = 0xa0;                             // turn on bits make up this byte
  msg.buf[1] = (apm_voltage_req * 1.27);
  Can0.write(msg);                               // send that to the auxiliary power module
}

void parse_charger_410(const CAN_message_t &msg)
{
  charger_dc_volts   = msg.buf[0];                                   //
  charger_ac_amps    = msg.buf[1] + (msg.buf[2] * 256);              //
  charger_dc_amps    = msg.buf[3] + (msg.buf[4] * 256) * .005 ;     //
  charger_limit_amps = msg.buf[5] + (msg.buf[6] * 256);              //
  charger_prox       = msg.buf[7] >> 6 & 0x04;                       //
  charger_type       = msg.buf[7] >> 4 & 0x04;                       //
}

void charger_outgoing_message()
{
  if (!scheduler(13)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  CAN_message_t msg;
  // msg.buf[0] = 
  // msg.buf[1] = charger_volt_set >> 8;
  // msg.buf[2] = charger_volt_set & 0x00ff; 
  // msg.buf[3] = max_dc_amp >> 8;
  // msg.buf[4] = max_dc_amp & 0x00ff;
  // msg.buf[5] = charger_module_limit_amp >> 8;
  // msg.buf[6] = charger_module_limit_amp & 0x00ff;
}

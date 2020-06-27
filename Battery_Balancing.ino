void setup_coulomb_counter()
{
  pack0_coulomb_total = (pack0_size / pack0_nominal_voltage) * 3600; // calc total coulombs. Pack size in WH / nominal voltage = amp/hours. * 3600 gives us amp seconds, which is coulombs
  pack1_coulomb_total = (pack1_size / pack1_nominal_voltage) * 3600; // calc total coulombs. Pack size in WH / nominal voltage = amp/hours. * 3600 gives us amp seconds, which is coulombs
}

void output_balancing_can()          // battery balancing comms, sends to 0x20A a status of 0x02, which is chevy volt speak for "charging". It can only balance in this status. Not sure if there is a reason to change it.
{                                    // the next part is the actual balance comands. They go out in 3 seperate blocks that go out every 20ms in sequence, preceeded by the 0x20A, (car mode) each time.
  if (!scheduler(5)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  CAN_message_t msg;
  switch (balance_loop)
  { 
    case 0:
    msg.id = 0x20A;
    msg.flags.extended = 0;
    msg.len = 1;
    msg.buf[0] = car_mode;
    Can0.write(msg);
    //Can1.write(msg);
    msg.id = 0x20E;
    msg.flags.extended = 0;
    msg.len = 6;
    msg.buf[0] = 0x18;
    msg.buf[1] = pack0_cell_balancing[0]; //cells 1-8...
    msg.buf[2] = pack0_cell_balancing[1];
    msg.buf[3] = pack0_cell_balancing[2];
    msg.buf[4] = pack0_cell_balancing[3];
    msg.buf[5] = pack0_cell_balancing[4];
    Can0.write(msg);
    msg.buf[1] = pack1_cell_balancing[0]; //cells 1-8...
    msg.buf[2] = pack1_cell_balancing[1];
    msg.buf[3] = pack1_cell_balancing[2];
    msg.buf[4] = pack1_cell_balancing[3];
    msg.buf[5] = pack1_cell_balancing[4];
    //Can1.write(msg);
    balance_loop++;
    break;

    case 1:
    msg.id = 0x20A;
    msg.flags.extended = 0;
    msg.len = 1;
    msg.buf[0] = car_mode;
    Can0.write(msg);
    //Can1.write(msg);
    msg.id = 0x20E;
    msg.flags.extended = 0;
    msg.len = 6;
    msg.buf[0] = 0x28;
    msg.buf[1] = pack0_cell_balancing[5]; //cells 41-48...
    msg.buf[2] = pack0_cell_balancing[6];
    msg.buf[3] = pack0_cell_balancing[7];
    msg.buf[4] = pack0_cell_balancing[8];
    msg.buf[5] = pack0_cell_balancing[9];
    Can0.write(msg);
    msg.buf[1] = pack1_cell_balancing[5]; //cells 41-48...
    msg.buf[2] = pack1_cell_balancing[6];
    msg.buf[3] = pack1_cell_balancing[7];
    msg.buf[4] = pack1_cell_balancing[8];
    msg.buf[5] = pack1_cell_balancing[9];
    //Can1.write(msg);
    balance_loop++;
    break;

    case 2:
    msg.id = 0x20A;
    msg.flags.extended = 0;
    msg.len = 1;
    msg.buf[0] = car_mode;
    Can0.write(msg);
    //Can1.write(msg);
    msg.id = 0x20E;
    msg.flags.extended = 0;
    msg.len = 6;
    msg.buf[0] = 0x38;
    msg.buf[1] = pack0_cell_balancing[10]; //cells 81-88...
    msg.buf[2] = pack0_cell_balancing[11]; //cells 89-96
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    Can0.write(msg);
    msg.buf[1] = pack1_cell_balancing[10]; //cells 81-88...
    msg.buf[2] = pack1_cell_balancing[11]; //cells 89-96
    //Can1.write(msg);
    balance_loop++;
    balance_loop = 0;
    break;
  }                           // end of switch balance loop
}                             // end of output_balancing_can

void coulomb_SOC()
{
  if (!scheduler(20)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (pack0_coulomb_SOC_initialized)
  {
    pack0_coulombs_remaining = pack0_coulombs_remaining + (pack0_amp_accumulator / pack0_amp_counter);   // average of the amperage in the last second are subtracted from the columns remaining.
    pack0_amp_counter = 0;                                                     // reset the counter to zero
    pack0_amp_accumulator = 0;
    pack0_coulomb_SOC = pack0_coulombs_remaining / pack0_coulomb_total * 100;  // calculate remaining SOC%
  }
  else
  {
    pack0_amp_counter = 0;                                                     // reset the counter to zero
    pack0_amp_accumulator = 0;
    if (!pack0_loss_of_comms)
    {
      battery_statistics();              // calculate some battery statistics from the incoming data  
      pack0_coulombs_remaining = (pack0_state_of_charge / 100) * pack0_coulomb_total; // figures out how many coulombs we start with based on battery voltage/amperage chart
      pack0_coulomb_SOC_initialized = true;
    }
  }         // end of else statement
  
  if (pack1_coulomb_SOC_initialized)
  {
    pack1_coulombs_remaining = pack1_coulombs_remaining + (pack1_amp_accumulator / pack1_amp_counter);   // average of the amperage in the last second are subtracted from the columns remaining.
    pack1_amp_counter = 0;                                                     // reset the counter to zero
    pack1_amp_accumulator = 0;
    pack1_coulomb_SOC = pack1_coulombs_remaining / pack1_coulomb_total * 100;  // calculate remaining SOC%
  }
  else
  {
    pack1_amp_counter = 0;                                                     // reset the counter to zero
    pack1_amp_accumulator = 0;
    
    if (!pack1_loss_of_comms)
    {
      battery_statistics();              // calculate some battery statistics from the incoming data  
      pack1_coulombs_remaining = (pack1_state_of_charge / 100) * pack1_coulomb_total; // figures out how many coulombs we start with based on battery voltage/amperage chart
      pack1_coulomb_SOC_initialized = true;
    }
  }         // end of else statement
}           // end of coulomb_SOC


void balancing_state()
{
  if (!scheduler(16)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  //Add up all the bits within the cell balancing high array to display the quantity of cells high balancing currently

  pack0_quantity_balancing_high = 0;
  pack1_quantity_balancing_high = 0;
  for(int i=0; i <= 11; i++)
  {
    pack0_quantity_balancing_high += ((pack0_cell_balancing_high[i] & 0x01) + (pack0_cell_balancing_high[i] >> 1 & 0x01) + (pack0_cell_balancing_high[i] >> 2 & 0x01) + \
    (pack0_cell_balancing_high[i] >> 3 & 0x01) + (pack0_cell_balancing_high[i] >> 4 & 0x01) + (pack0_cell_balancing_high[i] >> 5 & 0x01) + (pack0_cell_balancing_high[i] >> 6 &  0x01) + (pack0_cell_balancing_high[i] >> 7 & 0x01));
    pack1_quantity_balancing_high += ((pack1_cell_balancing_high[i] & 0x01) + (pack1_cell_balancing_high[i] >> 1 & 0x01) + (pack1_cell_balancing_high[i] >> 2 & 0x01) + \
    (pack1_cell_balancing_high[i] >> 3 & 0x01) + (pack1_cell_balancing_high[i] >> 4 & 0x01) + (pack1_cell_balancing_high[i] >> 5 & 0x01) + (pack1_cell_balancing_high[i] >> 6 &  0x01) + (pack1_cell_balancing_high[i] >> 7 & 0x01));
  }

  if (pack0_quantity_balancing_high >= 1)  //check to see if any high balancing cells exist
  {
    pack0_cell_balancing_state |= 0x01;   //turn on bit indicating high balancing active
  }
  else
  {
    pack0_cell_balancing_state &= 0xFE;   //turn off bit indicating high balancing active
  }

  if (pack1_quantity_balancing_high >= 1)  //check to see if any high balancing cells exist
  {
    pack1_cell_balancing_state |= 0x01;   //turn on bit indicating high balancing active
  }
  else
  {
    pack1_cell_balancing_state &= 0xFE;   //turn off bit indicating high balancing active
  }

  //Add up all the bits within the cell balancing low array

  pack0_quantity_balancing_low = 0;
  pack1_quantity_balancing_low = 0;
  for(int i=0; i <= 11; i++)
  {
    pack0_quantity_balancing_low += ((pack0_cell_balancing_low[i] & 0x01) + (pack0_cell_balancing_low[i] >> 1 & 0x01) + (pack0_cell_balancing_low[i] >> 2 & 0x01) + \
    (pack0_cell_balancing_low[i] >> 3 & 0x01) + (pack0_cell_balancing_low[i] >> 4 & 0x01) + (pack0_cell_balancing_low[i] >> 5 & 0x01) + (pack0_cell_balancing_low[i] >> 6 &  0x01) + (pack0_cell_balancing_low[i] >> 7 & 0x01));
    pack1_quantity_balancing_low += ((pack1_cell_balancing_low[i] & 0x01) + (pack1_cell_balancing_low[i] >> 1 & 0x01) + (pack1_cell_balancing_low[i] >> 2 & 0x01) + \
    (pack1_cell_balancing_low[i] >> 3 & 0x01) + (pack1_cell_balancing_low[i] >> 4 & 0x01) + (pack1_cell_balancing_low[i] >> 5 & 0x01) + (pack1_cell_balancing_low[i] >> 6 &  0x01) + (pack1_cell_balancing_low[i] >> 7 & 0x01));
  }

  if (pack0_quantity_balancing_low >= 1) //check to see if any low balancing cells exist
  {
    pack0_cell_balancing_state |= 0x02; //turn on bit indicating low balancing active
  } 
  else
  {
    pack0_cell_balancing_state &= 0xFD; //turn off bit indicating low balancing active
  }
  if (pack1_quantity_balancing_low >= 1) //check to see if any low balancing cells exist
  {
    pack1_cell_balancing_state |= 0x02; //turn on bit indicating low balancing active
  } 
  else
  {
    pack1_cell_balancing_state &= 0xFD; //turn off bit indicating low balancing active
  }
}                                 // end of balancing_state function


// Charging Amperage target vs SOC
  
// Amperage reduction for out of balance
  
// Pack errors
  
// Regen Limiting
  
// Pack temperature range
  
// Charge statuses

// Outgoing comms

// Tesla charge port comms

void pack_comms_trace_reset()    // periodically runs to set the comms trace to zero.
{
  if (!scheduler(27)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  pack0_comms_timeout_trace = 0; // Reset comms trace to zero
  pack1_comms_timeout_trace = 0; // Reset comms trace to zero
}

void pack0_reset_incoming_data()
{
  if (!scheduler(30)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if(pack1_comms_timeout_trace > 0)                                      // if any comms have flagged as recieved...
  {
    pack1_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
  }
  if(currentMillis - pack1_comms_timeout_clock >= pack_comms_timeout)    // if more than the "timeout" ms have elapsed since the last run, reset the timer and run reset functions.
  {
    for (int i = 0; i <= 95; ++i) //setting array to zero using a loop
    {
      pack0_cell_voltages[i] = 0;
    }
    for (int i = 0; i <= 8; ++i)  //setting array to zero using a loop
    {
      pack0_temperatures[i] = 0;
    }
    pack0_high_range_amperage = 0;
    pack0_low_range_amperage = 0;
    pack0_voltage = 0;
    pack0_loss_of_comms = true;
  }
}                               //end of pack0_reset_incoming_data

void pack1_reset_incoming_data()
{
  if (!scheduler(31)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if(pack1_comms_timeout_trace > 0)                                      // if any comms have flagged as recieved...
  {
    pack1_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
  }
  if(currentMillis - pack1_comms_timeout_clock >= pack_comms_timeout)    // if more than the "timeout" ms have elapsed since the last run, reset the timer and run reset functions.
  {
    pack1_comms_timeout_clock = currentMillis;                           // reset the timer to current millis
    pack1_reset_incoming_data();                                         // reset incoming data.                                    
    for (int i = 0; i <= 95; ++i) //setting array to zero using a loop
    {
      pack1_cell_voltages[i] = 0;
    }
    for (int i = 0; i <= 8; ++i)  //setting array to zero using a loop
    {
      pack1_temperatures[i] = 0;
    }
    pack1_high_range_amperage = 0;
    pack1_low_range_amperage = 0;
    pack1_voltage = 0;
    pack1_loss_of_comms = true;
  }    
}                               //end of reset_incoming_data

void execute_master_reset()
{
  if (!scheduler(6)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (master_reset)
  {
    pack0_cell_high_fault         = false; // 
    pack1_cell_high_fault         = false; //
    pack0_cell_high_warning       = false; //
    pack1_cell_high_warning       = false; //
    pack0_cell_low_fault          = false; //
    pack1_cell_low_fault          = false; //
    pack0_cell_low_warning        = false; //
    pack1_cell_low_warning        = false; //
    pack0_inhibit_high_balancing  = false; //
    pack1_inhibit_high_balancing  = false; //
    pack0_inhibit_low_balancing   = false; //
    pack1_inhibit_low_balancing   = false; //
    charging_allowed              = true;  //
    pack0_loss_of_comms           = false; //
    pack1_loss_of_comms           = false; //
    aps_fault                     = false; //
    rps_fault                     = false; // 
  }
} // end execute_master_reset

void restore_resettables()
{
  if (!scheduler(28)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  pack0_loss_of_comms = false;
  pack1_loss_of_comms = false;
}                             //end of restore_resettables

void pack0_balance_output_sequencer()
{
  if (!scheduler(17)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (pack0_enabled)
  {
    switch (pack0_cell_balancing_state)
    {
      case 0:
      for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
      {
        pack0_cell_balancing[i] = 0;
      }
      pack0_balance_sequencer = 0;
      break;
  
      case 1: //balancing high cell(s) only active
      for (int i = 0; i <= 11; ++i) //copy the array over
      {
        pack0_cell_balancing[i] = pack0_cell_balancing_high[i];
      }    
      pack0_balance_sequencer = 0;
      break;
  
      case 2:  //balancing low cell(s) only active
      if (pack0_balance_sequencer > 3) pack0_balance_sequencer = 0;
      switch (pack0_balance_sequencer)
      {
        case 0:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x11; //turn on balancing on every 4th cell excluding the low cells
        }
        pack0_balance_sequencer++;
        break;
            
        case 1:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x22; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack0_balance_sequencer++;
        break;
            
        case 2:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x44; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack0_balance_sequencer++;
        break;          
            
        case 3:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x88; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        if (pack0_balance_sequencer >= 3) pack0_balance_sequencer = 0;
        break;          
      }                   // end of switch (balance_low_counter)
      break;
  
      case 3:  //balancing high and low cell(s) active
      if (pack0_balance_sequencer > 7) pack0_balance_sequencer = 0;
      switch (pack0_balance_sequencer)
      {
        case 0:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x11; //turn on balancing on every 4th cell excluding the low cells
        }
        pack0_balance_sequencer++;
        break;
  
        case 1:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack0_cell_balancing[i] = pack0_cell_balancing_high[i];
        }
        pack0_balance_sequencer++;
        break;
  
        case 2:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x22; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack0_balance_sequencer++;
        break;
  
        case 3:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack0_cell_balancing[i] = pack0_cell_balancing_high[i];
        }
        pack0_balance_sequencer++;          
        break;
            
        case 4:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x44; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack0_balance_sequencer++;
        break;          
  
        case 5:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack0_cell_balancing[i] = pack0_cell_balancing_high[i];
        }
        pack0_balance_sequencer++;          
        break;
            
        case 6:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack0_cell_balancing[i] = (0xFF - pack0_cell_balancing_low[i]) & 0x88; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack0_balance_sequencer++;
        break;
            
        case 7:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack0_cell_balancing[i] = pack0_cell_balancing_high[i];
        }
        pack0_balance_sequencer++;          
        break;
       
      }                       // end of switch (balance_low_counter)
      break;
    }                         // end of switch (balance_state)
  }                           // end if pack0_enabled statement                   
}                             // end of pack0_balance_output_sequencer function

void pack1_balance_output_sequencer()
{
  if (!scheduler(18)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (pack1_enabled)
  {
    switch (pack1_cell_balancing_state)
    {
      case 0:
      for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
      {
        pack1_cell_balancing[i] = 0;
      }
      pack1_balance_sequencer = 0;
      break;
  
      case 1: //balancing high cell(s) only active
      for (int i = 0; i <= 11; ++i) //copy the array over
      {
        pack1_cell_balancing[i] = pack1_cell_balancing_high[i];
      }    
      pack1_balance_sequencer = 0;
      break;
  
      case 2:  //balancing low cell(s) only active
      if (pack1_balance_sequencer > 3) pack1_balance_sequencer = 0;
      switch (pack1_balance_sequencer)
      {
        case 0:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x11; //turn on balancing on every 4th cell excluding the low cells
        }
        pack1_balance_sequencer++;
        break;
            
        case 1:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x22; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack1_balance_sequencer++;
        break;
            
        case 2:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x44; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack1_balance_sequencer++;
        break;          
            
        case 3:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x88; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        if (pack1_balance_sequencer >= 3) pack1_balance_sequencer = 0;
        break;          
      }                   // end of switch (balance_low_counter)
      break;
  
      case 3:  //balancing high and low cell(s) active
      if (pack1_balance_sequencer > 7) pack1_balance_sequencer = 0;
      switch (pack1_balance_sequencer)
      {
        case 0:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x11; //turn on balancing on every 4th cell excluding the low cells
        }
        pack1_balance_sequencer++;
        break;
  
        case 1:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack1_cell_balancing[i] = pack1_cell_balancing_high[i];
        }
        pack1_balance_sequencer++;
        break;
  
        case 2:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x22; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack1_balance_sequencer++;
        break;
  
        case 3:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack1_cell_balancing[i] = pack1_cell_balancing_high[i];
        }
        pack1_balance_sequencer++;          
        break;
            
        case 4:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x44; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack1_balance_sequencer++;
        break;          
  
        case 5:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack1_cell_balancing[i] = pack1_cell_balancing_high[i];
        }
        pack1_balance_sequencer++;          
        break;
            
        case 6:
        for (int i = 0; i <= 11; ++i) //setting array to zero using a loop
        {
          pack1_cell_balancing[i] = (0xFF - pack1_cell_balancing_low[i]) & 0x88; //turn on balancing on every 4th cell excluding the low cells, next 4 in line
        }
        pack1_balance_sequencer++;
        break;
            
        case 7:
        for (int i = 0; i <= 11; ++i) //copy the array over
        {
          pack1_cell_balancing[i] = pack1_cell_balancing_high[i];
        }
        pack1_balance_sequencer++;          
        break;
       
      }                       // end of switch (balance_low_counter)
      break;
    }                         // end of switch (balance_state)
  }                           // end of if pack1_enabled
}                             // end of pack1_balance_output_sequencer function
  // Battery statistic calcs
void battery_statistics()
{
  if (!scheduler(14)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  pack0_cell_sum_volt = 0;                              // reset the average to zero so it doesn't accumulate each loop
  pack1_cell_sum_volt = 0;                              // reset the average to zero so it doesn't accumulate each loop
  pack0_highest_cell = 0;                               // reset the highest cell statistic
  pack1_highest_cell = 0;                               // reset the highest cell statistic
  pack0_lowest_cell = pack0_cell_voltages[0];           // reset the lowest cell statistic to the first cells voltage
  pack1_lowest_cell = pack1_cell_voltages[0];           // reset the lowest cell statistic to the first cells voltage
  for(int i=0; i <= 95 ; i++)
  {
    pack0_cell_sum_volt += pack0_cell_voltages[i];
    pack1_cell_sum_volt += pack1_cell_voltages[i];
    
    if (pack0_cell_voltages[i] > pack0_highest_cell)    // if the current cell is higher than the highest cell...
    {
      pack0_highest_cell = pack0_cell_voltages[i];      // make it the new high
    }
    if (pack1_cell_voltages[i] > pack1_highest_cell)    // if the current cell is higher than the highest cell...
    {
      pack1_highest_cell = pack1_cell_voltages[i];      // make it the new high
    }
    if (pack0_cell_voltages[i] < pack0_lowest_cell)     // if the current cell is lower than the lowest cell...
    {
      pack0_lowest_cell = pack0_cell_voltages[i];      // make it the new low
    }
    if (pack1_cell_voltages[i] < pack1_lowest_cell)     // if the current cell is lower than the lowest cell...
    {
      pack1_lowest_cell = pack1_cell_voltages[i];      // make it the new low
    }
  }
  // average values
  pack0_cell_avg_volt = pack0_cell_sum_volt / 96;
  pack1_cell_avg_volt = pack1_cell_sum_volt / 96;
  int pack0_avg_volt = pack0_voltage * 10.417; // 
  int pack1_avg_volt = pack1_voltage * 10.417; // 1000/96 so we get cell average MV from pack voltage
   
  pack0_sum_temp = 0;
  pack1_sum_temp = 0;

   
  for(int i=0;i <= 8 ;i++)
  {
    pack0_sum_temp += pack0_temperatures[i];
    pack1_sum_temp += pack1_temperatures[i];
  }
  pack0_avg_temp = (pack0_sum_temp / 9 );
  pack1_avg_temp = (pack1_sum_temp / 9 );
    
  // SOC
    pack0_state_of_charge = table_lookup(SOC, pack0_high_range_amperage, pack0_cell_avg_volt, -420, 230, 3100, 4300, 17, 17); //Parameters (Table name, X input, Y input, min X value, max X value, min Y value, max Y value, # of Columns, # of Rows)
    pack1_state_of_charge = table_lookup(SOC, pack1_high_range_amperage, pack1_cell_avg_volt, -420, 230, 3100, 4300, 17, 17); //Parameters (Table name, X input, Y input, min X value, max X value, min Y value, max Y value, # of Columns, # of Rows)
}                 //end of battery_statistics function

void fault_logic()
{
  if (!scheduler(11)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if(pack0_enabled && pack0_cell_low_warning)
  {
    pack0_inhibit_high_balancing   = false; // warning only, no action
    pack0_inhibit_low_balancing    = false; // warning only, no action
    charging_allowed               &= true;  // warning only, no action
  }
  
  if(pack1_enabled && pack1_cell_low_warning)
  {
    pack1_inhibit_high_balancing   = false; // warning only, no action
    pack1_inhibit_low_balancing    = false; // warning only, no action
    charging_allowed               &= true;  // warning only, no action
  }
  
  if(pack0_enabled && pack0_cell_high_warning)
  {
    pack0_inhibit_high_balancing   &= true;  // if its already inhibited by another process, keep it inhibited, otherwise allow it to balance that cell down
    pack0_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    charging_allowed               = false; // stop charging
  }
  
  if(pack1_enabled && pack1_cell_high_warning)
  {
    
    pack1_inhibit_high_balancing   &= true;  // if its already inhibited by another process, keep it inhibited, otherwise allow it to balance that cell down
    pack1_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    charging_allowed               = false; // stop charging
  }  
  
  if(pack0_enabled && (pack0_loss_of_comms || pack0_cell_high_fault || pack0_cell_low_fault))
  {
    pack0_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack1_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack0_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    pack1_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    charging_allowed               = false; // stop charging
  }
  
  if(pack1_enabled && (pack1_loss_of_comms || pack1_cell_high_fault || pack1_cell_low_fault))
  {
    pack0_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack1_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack0_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    pack1_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    charging_allowed               = false; // stop charging
  }

  if(!charging_allowed)
  {
    pack0_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack1_inhibit_high_balancing   = true;  // cells are too far out of balance, stop everything
    pack0_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
    pack1_inhibit_low_balancing    = true;  // don't want low balancing to take place when not charging, ever
  }

}                  // end fault_logic

  // figure out which cells need to be balanced, and if there are any WAY out of balance
void balancing_determination()
{
  if (!scheduler(15)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  // this "for loop" is the one that looks at each cell to determine its deviation from average, then corrective action
  for(int i=0; i <= 95 ; i++)
  {
    
    int balance_byte_on =  pow(2,(i -(8 * (int(i / 8)))));  //create a variable of what the byte in either cell_balancing_high would be set/cleared
    int balance_byte_off =  0xff - pow(2,(i -(8 * (int(i / 8)))));
    int balance_select = int(i / 8);                //create a value to reference which byte would be used for set/clear balancing
    int pack0_balance_l_bit = ((pack0_cell_balancing_low[(int(i / 8))]) >> (i -(8 * (int(i / 8))))) & 0x01;
    int pack1_balance_l_bit = ((pack1_cell_balancing_low[(int(i / 8))]) >> (i -(8 * (int(i / 8))))) & 0x01;
    int pack0_balance_h_bit = ((pack0_cell_balancing_high[(int(i / 8))]) >> (i -(8 * (int(i / 8))))) & 0x01;
    int pack1_balance_h_bit = ((pack1_cell_balancing_high[(int(i / 8))]) >> (i -(8 * (int(i / 8))))) & 0x01;
    
    float pack0_cell_difference = pack0_cell_voltages[i] - pack0_cell_avg_volt; // get the deviation from average for each cell as it runs through the for loop
    float pack1_cell_difference = pack1_cell_voltages[i] - pack1_cell_avg_volt; // get the deviation from average for each cell as it runs through the for loop

    if (pack0_cell_difference > cell_high_fault_tolerance) bool pack0_cell_high_fault = true;     //
    if (pack1_cell_difference > cell_high_fault_tolerance) bool pack1_cell_high_fault = true;     //
    if (pack0_cell_voltages[i]> cell_high_fault_threshold) bool pack0_cell_high_fault = true;     //  
    if (pack1_cell_voltages[i]> cell_high_fault_threshold) bool pack1_cell_high_fault = true;     //     
    if (pack0_cell_difference > cell_high_warning_tolerance) bool pack0_cell_high_warning = true; //
    if (pack1_cell_difference > cell_high_warning_tolerance) bool pack1_cell_high_warning = true; //
    if (pack0_cell_voltages[i]> cell_high_warning_threshold) bool pack0_cell_high_warning = true; //  
    if (pack1_cell_voltages[i]> cell_high_warning_threshold) bool pack1_cell_high_warning = true; //      
    if (pack0_cell_difference < cell_low_fault_tolerance) bool pack0_cell_low_fault = true;       //
    if (pack1_cell_difference < cell_low_fault_tolerance) bool pack1_cell_low_fault = true;       //
    if (pack0_cell_voltages[i]< cell_low_fault_threshold) bool pack0_cell_low_fault = true;       //  
    if (pack1_cell_voltages[i]< cell_low_fault_threshold) bool pack1_cell_low_fault = true;       //  
    if (pack0_cell_difference < cell_low_warning_tolerance) bool pack0_cell_low_warning = true;   // 
    if (pack1_cell_difference < cell_low_warning_tolerance) bool pack1_cell_low_warning = true;   //
    if (pack0_cell_voltages[i]< cell_low_warning_threshold) bool pack0_cell_low_warning = true;   //  
    if (pack1_cell_voltages[i]< cell_low_warning_threshold) bool pack1_cell_low_warning = true;   //   
    
    // charge reduction code, when we get to it.
    
    // Check balancing status of individual cell and if starting or completing a balance, update the bit on the array, and notify via the serial monitor.\
    Some complexity (switch/case) was added to make it only send 1 serial update as opposed to each time the loop ran through.

    if (!pack0_enabled || pack0_inhibit_high_balancing)
    {
      pack0_cell_balancing_high[balance_select] &= balance_byte_off;   //clear all balancing cells
    }
    else
    {
      switch (pack0_balance_h_bit)
    
      {
        case 0:
        if (pack0_cell_difference > balance_high_enable)
        {
          pack0_cell_balancing_high[balance_select] |= balance_byte_on; //
          Serial.print("Balancing High Cell Started on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack0_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing High Started on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack0_cell_difference); 
          
        }
        break;

        case 1:
        if (pack0_cell_difference < balance_high_complete)
        {
          pack0_cell_balancing_high[balance_select] &= balance_byte_off; //
          Serial.print("Balancing High Cell Completed on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack0_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing High Completed on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack0_cell_difference);   
        }
        break;      
      }                      // end of switch (balance_h_bit)
    }                        // end of else statement
    
    if (!pack1_enabled || pack1_inhibit_high_balancing)
    {
      pack1_cell_balancing_high[balance_select] &= balance_byte_off;   //clear all balancing cells
    }
    else
    {
      switch (pack1_balance_h_bit)
      {
        case 0:
        if (pack1_cell_difference > balance_high_enable)
        {
          pack1_cell_balancing_high[balance_select] |= balance_byte_on; //
          Serial.print("Balancing High Cell Started on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack1_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing High Started on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack1_cell_difference); 
        }
        break;

        case 1:
        if (pack1_cell_difference < balance_high_complete)
        {
          pack1_cell_balancing_high[balance_select] &= balance_byte_off; //   
          Serial.print("Balancing High Cell Completed on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack1_cell_difference);
          file.println();
          printNow();
          file.print(",Battery Balancing,Balancing High Completed on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack1_cell_difference);   
        }
        break;      
      }                      // end of switch (balance_h_bit)
    }                        // end of else statement

    if (!pack0_enabled || pack0_inhibit_low_balancing)
    {
      pack0_cell_balancing_low[balance_select] &= balance_byte_off;   //clear all balancing cells
    }
    else
    {
      switch (pack0_balance_l_bit)
      {
        case 0:
        if (pack0_cell_difference < balance_low_enable)
        {
          pack0_cell_balancing_low[balance_select] |= balance_byte_on; //
          Serial.print("Balancing Low Cell Started on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack0_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing Low Started on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack0_cell_difference); 
        }
        break;
        case 1:
        if (pack0_cell_difference > balance_low_complete)
        {
          pack0_cell_balancing_low[balance_select] &= balance_byte_off; //
          Serial.print("Balancing Low Cell Completed on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack0_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing Low Completed on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack0_cell_difference);  
        }
        break;  
      }                      // end of switch (balance_l_bit)
    }                        // end of else statement
    if (!pack1_enabled || pack1_inhibit_low_balancing)
    {
      pack1_cell_balancing_low[balance_select] &= balance_byte_off;   //clear all balancing cells
    }
    else
    {
      switch (pack1_balance_l_bit)
      {
        case 0:
        if (pack1_cell_difference < balance_low_enable)
        {
          pack1_cell_balancing_low[balance_select] |= balance_byte_on; //
          Serial.print("Balancing Low Cell Started on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack1_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing Low Started on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack1_cell_difference); 
        }
        break;
        case 1:
        if (pack1_cell_difference > balance_low_complete)
        {
          pack0_cell_balancing_low[balance_select] &= balance_byte_off; //
          Serial.print("Balancing Low Cell Completed on cell #");
          Serial.print(i + 1,DEC);
          Serial.print("Deviation from average cell voltage: ");
          Serial.print(pack1_cell_difference);
          printNow();
          file.print(",Battery Balancing,Balancing Low Completed on cell # ");
          file.println(i + 1,DEC);
          file.print(",Deviation from average cell voltage ,");
          file.println(pack1_cell_difference);   
        }
        break;  
      }                            // end of switch (balance_l_bit)
    }                              // end of else statement
  }                                // end of for loop, the one that checks all the cells for balancing needs and faults
}                                  // end of balancing_determination function

// table function
float table_lookup(float table[], float Raw_X, float Raw_Y, float Min_X_Input, float Max_X_Input, float Min_Y_Input, float Max_Y_Input, int Columns, int Rows)
{
  // Parameters in order:
  // Min_X_Input = ####.####;    // Can be switched to "int" if the chart values don't contain decimals
  // Max_X_Input = ####.####;    // Can be switched to "int" if the chart values don't contain decimals
  // Min_Y_Input = ####.####;    // Can be switched to "int" if the chart values don't contain decimals
  // Max_Y_Input = ####.####;    // Can be switched to "int" if the chart values don't contain decimals
  // Columns = ##;               // Number of Columns in table
  // Rows = ##;                  // Number of Rows in table

  float Raw_X_Constrained = constrain(Raw_X, Min_X_Input, Max_X_Input);            // limits range in X axis
  float Raw_Y_Constrained = constrain(Raw_Y, Min_Y_Input, Max_Y_Input);            // limits range in X axis
  float Raw_X_Charted = Raw_X_Constrained - Min_X_Input;                           // apply the charting offset for X axis
  float Raw_Y_Charted = Raw_Y_Constrained - Min_Y_Input;                           // apply the charting offset for Y axis
  float X_Lookup = Raw_X_Charted / ((Max_X_Input - Min_X_Input) / (Columns - 1));  // Divides the X number by the X index spacing, which is overall range, divided by the numbwer of columns
  float Y_Lookup = Raw_Y_Charted / ((Max_Y_Input - Min_Y_Input) / (Rows - 1));     // Divides the Y number by the Y index spacing, which is overall range, divided by the numbwer of Rows
  int Target_Cell = int(Y_Lookup) * Columns + int(X_Lookup);                       // Finds the main target cell# for lookup
  float X_Interpolate = X_Lookup - int(X_Lookup);                                  // Finds the decimal value for percise location between the cells for interpolation
  float Y_Interpolate = Y_Lookup - int(Y_Lookup);                                  // Finds the decimal value for percise location between the cells for interpolation
  float Upper_Interpolate = ((table[(Target_Cell + 1)] - table[Target_Cell]) * X_Interpolate) + table[Target_Cell];                                      // Find the interpolated value between the target cell, and the one to the right of it
  float Lower_Interpolate = ((table[((Target_Cell + Columns) + 1)] - table[(Target_Cell + Columns)]) * X_Interpolate) + table[(Target_Cell + Columns)];  // Find the interpolated value between the cell below the target cell, and the one to the right of it
  float Output = ((Lower_Interpolate - Upper_Interpolate) * Y_Interpolate) + Upper_Interpolate;                                                          // Interpolate between the upper and lower interpolated values, this gives us the final value
  return Output;
}                         // end of table function



// this is the part where we take the incoming can frames and turn them into useful variables

void parse_pack0_200(const CAN_message_t &msg)
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x01;
  volt_select = (msg.buf[6] >> 5);
  pack0_cell_voltages[(volt_select*3)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+1)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+2)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack0_202(const CAN_message_t &msg)
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x02;
  volt_select = (msg.buf[6] >> 5);
  pack0_cell_voltages[(volt_select*3+24)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+25)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+26)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack0_204(const CAN_message_t &msg)
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x04;
  volt_select = (msg.buf[6] >> 5);
  pack0_cell_voltages[(volt_select*3+48)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+49)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+50)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}
    
void parse_pack0_206(const CAN_message_t &msg) 
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x08;
  volt_select = (msg.buf[6] >> 5);
  pack0_cell_voltages[(volt_select*3+72)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+73)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack0_cell_voltages[(volt_select*3+74)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack0_210(const CAN_message_t &msg)
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x10;
  pack0_voltage = float((msg.buf[0] * 16) + (msg.buf[1] >> 4)) * 0.125;
  pack0_low_range_amperage = float(((msg.buf[1] & 0x01) * 256) + msg.buf[2]) * 0.078;
   if ((((msg.buf[1] & 0x01) * 256) + msg.buf[2]) > 256)
  {
    pack0_low_range_amperage = float((((msg.buf[1] & 0x01) * 256) + msg.buf[2]) - 512) * 0.078;
  }
  else
  {
    pack0_low_range_amperage = float(((msg.buf[1] & 0x01) * 256) + msg.buf[2]) * 0.078;
  }

  
  if (((msg.buf[3] * 32) + (msg.buf[4] >> 3)) > 4095)
  {
    pack0_high_range_amperage = float(((msg.buf[3] * 32) + (msg.buf[4] >> 3)) - 8192) * 0.0793;
  }
  else
  {
    pack0_high_range_amperage = float((msg.buf[3] * 32) + (msg.buf[4] >> 3)) * 0.0793;
  }

  
  // This little bit is the amperage accumulator for coulomb counting
  pack0_amp_counter++;                                             // keep track of how many amperage samples we generate
  if (abs(pack0_high_range_amperage) >= abs(pack0_low_range_amperage)) // use the finer resolution if its in range
  {
    pack0_amp_accumulator += pack0_high_range_amperage;  // add the high range amperage to the rolling total
  }
  else
  {
    pack0_amp_accumulator += pack0_low_range_amperage;  // add the low range amperage to the rolling total
  }
}

void parse_pack0_302(const CAN_message_t &msg)
{
  pack0_comms_timeout_trace = pack0_comms_timeout_trace | 0x20;
  switch (msg.buf[7] >> 5)
  {
    case 0x00:
    pack0_temperatures[0] = (float(msg.buf[1]) - 80) * 0.555;  // temperature, expressed in Celcius. replace " - 80 * 0.555;" with " - 46;" for Farenheit, you knuckle dragger
    pack0_temperatures[1] = (float(msg.buf[2]) - 80) * 0.555;
    pack0_temperatures[2] = (float(msg.buf[3]) - 80) * 0.555;
    pack0_temperatures[3] = (float(msg.buf[4]) - 80) * 0.555;
    pack0_temperatures[4] = (float(msg.buf[5]) - 80) * 0.555;
    pack0_temperatures[5] = (float(msg.buf[6]) - 80) * 0.555;
    break;
    case 0x01:
    pack0_temperatures[6] = (float(msg.buf[1]) - 80) * 0.555;
    pack0_temperatures[7] = (float(msg.buf[2]) - 80) * 0.555;
    pack0_temperatures[8] = (float(msg.buf[3]) - 80) * 0.555;
    break;
  }
}

void parse_pack1_200(const CAN_message_t &msg)
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x01;
  volt_select = (msg.buf[6] >> 5);
  pack1_cell_voltages[(volt_select*3)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+1)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+2)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack1_202(const CAN_message_t &msg)
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x02;
  volt_select = (msg.buf[6] >> 5);
  pack1_cell_voltages[(volt_select*3+24)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+25)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+26)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack1_204(const CAN_message_t &msg)
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x04;
  volt_select = (msg.buf[6] >> 5);
  pack1_cell_voltages[(volt_select*3+48)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+49)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+50)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}
    
void parse_pack1_206(const CAN_message_t &msg) 
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x08;
  volt_select = (msg.buf[6] >> 5);
  pack1_cell_voltages[(volt_select*3+72)] = float(((msg.buf[0] & 0x1F)  * 128) + (msg.buf[1] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+73)] = float(((msg.buf[2] & 0x1F)  * 128) + (msg.buf[3] >> 1)) * 1.250;
  pack1_cell_voltages[(volt_select*3+74)] = float((msg.buf[4] * 16) + (msg.buf[5] >> 4)) * 1.250;
}

void parse_pack1_210(const CAN_message_t &msg)
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x10;
  pack1_voltage = float((msg.buf[0] * 16) + (msg.buf[1] >> 4)) * 0.125;
  
  if ((((msg.buf[1] & 0x01) * 256) + msg.buf[2]) > 256)
  {
    pack1_low_range_amperage = float((((msg.buf[1] & 0x01) * 256) + msg.buf[2]) - 256) * 0.078;
  }
  else
  {
    pack1_low_range_amperage = float(((msg.buf[1] & 0x01) * 256) + msg.buf[2]) * 0.078;
  }

  if (((msg.buf[3] * 32) + (msg.buf[4] >> 3)) > 4095)
  {
    pack1_high_range_amperage = float(((msg.buf[3] * 32) + (msg.buf[4] >> 3)) - 8192) * 0.0793;
  }
  else
  {
    pack1_high_range_amperage = float((msg.buf[3] * 32) + (msg.buf[4] >> 3)) * 0.0793;
  }

  
  // This little bit is the amperage accumulator for coulomb counting
  pack1_amp_counter++;                                             // keep track of how many amperage samples we generate
  if (abs(pack1_high_range_amperage) >= abs(pack1_low_range_amperage)) // use the finer resolution if its in range
  {
    pack1_amp_accumulator += pack1_high_range_amperage;  // add the high range amperage to the rolling total
  }
  else
  {
    pack1_amp_accumulator += pack1_low_range_amperage;  // add the low range amperage to the rolling total
  }
}

void parse_pack1_302(const CAN_message_t &msg)
{
  pack1_comms_timeout_trace = pack1_comms_timeout_trace | 0x20;
  switch (msg.buf[7] >> 5)
  {
    case 0x00:
    pack1_temperatures[0] = (float(msg.buf[1]) - 80) * 0.555;  // temperature, expressed in Celcius. replace " - 80 * 0.555;" with " - 46;" for Farenheit, you knuckle dragger
    pack1_temperatures[1] = (float(msg.buf[2]) - 80) * 0.555;
    pack1_temperatures[2] = (float(msg.buf[3]) - 80) * 0.555;
    pack1_temperatures[3] = (float(msg.buf[4]) - 80) * 0.555;
    pack1_temperatures[4] = (float(msg.buf[5]) - 80) * 0.555;
    pack1_temperatures[5] = (float(msg.buf[6]) - 80) * 0.555;
    break;
    case 0x01:
    pack1_temperatures[6] = (float(msg.buf[1]) - 80) * 0.555;
    pack1_temperatures[7] = (float(msg.buf[2]) - 80) * 0.555;
    pack1_temperatures[8] = (float(msg.buf[3]) - 80) * 0.555;
    break;
  }
}    

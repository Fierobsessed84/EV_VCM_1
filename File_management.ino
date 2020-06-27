void setup_file_time_and_logs()
{
  setSyncProvider(getTeensy3Time);
  FsDateTime::setCallback(dateTime); 
  char filename1 = "VCM Logs ";
  char filename2 = year();
  char filename3 = "-";
  char filename4 = month();
  char filename5 = day();
  char filename6 = ".txt";
  char filename = "Log.csv";//filename1 + filename2 + filename3 + filename4 + filename3 + filename5 + filename6;
  if (sd.exists("Log.csv"))
  {
    if (!file.open("Log.csv", FILE_WRITE))
    {
      Serial.println(F("file.open failed"));
    } 
  }                                 
  file.open("Log.csv", FILE_WRITE);
  file.print("Timestamp,Category,Data");
  if (log_pack0_cell_voltages)                   // if log pack cells is enabled...
  {
    printNow();                                  // time stamp it
    file.print(",Pack 0 cell data,Comms OK,Cell # ");     // create CSV header
    for(int i=0; i <= 95 ; i++)                  // go through each cell
    {
      file.print(i + 1);                         // print all the cell numbers
      file.print(",Cell # ");                    // print all the cell numbers
    }
  }  // end if
  
  if (log_pack1_cell_voltages)                   // if log pack cells is enabled...
  {
    printNow();                                  // time stamp it
    file.print(",Pack 1 cell data,Comms OK,Cell # ");     // create CSV header
    for(int i=0; i <= 95 ; i++)                  // go through each cell
    {
      file.print(i + 1);                         // print all the cell numbers
      file.print(",Cell # ");                    // print all the cell numbers
    }
  } // end if
  if (log_pack_data)                             // if log pack data is enabled... 
  {
    printNow();                                  // time stamp
    file.print(",Pack Data,Pack0 Voltage,Pack 0 Amps high,Pack 0 Amps Low,Pack 0 Temp 1,Pack 0 Temp 2,Pack 0 Temp 3,Pack 0 Temp 4,Pack 0 Temp 5,Pack 0 Temp 6,Pack 0 Temp 7,Pack 0 Temp 8,Pack 0 Temp 9"); // create CSV header first half
    file.print(",Pack1 Voltage,Pack 1 Amps high,Pack 1 Amps Low,Pack 1 Temp 1,Pack 1 Temp 2,Pack 1 Temp 3,Pack 1 Temp 4,Pack 1 Temp 5,Pack 1 Temp 6,Pack 1 Temp 7,Pack 1 Temp 8,Pack 1 Temp 9");           // create CSV header second half
  }
  if (log_drive_unit_comms)                      // if log drive unit comms is enabled... 
  {
    printNow();                                  // time stamp it
    file.print(",Drive unit comms,Cruise,Start,Brake,Forward,Reverse,BMS,Pot 1,Pot 2,DC Voltage,DC Amperage,Speed,Inverter Temperature,Motor Temperature,Direction,Operating Mode"); // create CSV header
  }

  if (log_cell_statistics)                       // if log drive unit comms is enabled... 
  {
    printNow();                                  // time stamp it
    file.print(",Cell Statistics,Pack 0 Voltage,Pack 1 Voltage,Pack 0 Average,Pack 1 Average,Pack 0 highest Cell,Pack 1 Highest Cell,Pack 0 Lowest Cell,Pack 1 Lowest Cell,Pack 0 Average Temp,Pack 1 Average Temp"); // create CSV header
  }
  if (log_accelerator_positions)
  {
    printNow();                                  // time stamp
    file.print(",Accel/Regen,APS1,APS2,Normalized APS1,Normalized APS2,APS1 HIGH,APS1 LOW,APS2 HIGH,APS2 LOW,APS1 HIGH LIM,APS1 LOW LIM,APS2 HIGH LIM,APS2 LOW lIM,APS Fault,DU APS (POT)"); // create CSV header first half
    file.print(",RPS1,RPS2,Normalized RPS1,Normalized RPS2,RPS1 HIGH,RPS1 LOW,RPS2 HIGH,RPS2 LOW,RPS1 HIGH LIM,RPS1 LOW LIM,RPS2 HIGH LIM,RPS2 LOW lIM,DU RPS (POT2)");            // create CSV header second half
  }
  
} // end setup_file_time_and_logs

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10)
{
  
  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());
  
  // Return low time bits in units of 10 ms.
  *ms10 = second() & 1 ? 100 : 0;
}

void flash_led()

{
  if (!scheduler(10)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  digitalWrite(led_pin, !digitalRead(led_pin)); // toggle LED by inverting whats read from it
}

bool scheduler(byte index)
{
   currentMillis = millis();                                            // jot the current millis down for comparison
 
  if(currentMillis - schedule[(index * 2) + 1]>= schedule[index * 2])   // if 10 or more ms have elapsed since the last run, reset the timer and run 010 functions.
  {
    schedule[(index * 2) + 1] = currentMillis;                                    // reset last run to current "millis"
    return true;                                                        // ok to run that function
  }
  return false;                                                         // skip this function this time
}

void printNow()
{
  file.println();
  file.print(year());
  file.print('-'); 
  file.print(month());
  file.print('-');
  file.print(day());
  file.print(' ');
  file.print(hour());
  file.print(':');
  file.print(minute());
  file.print(':');
  file.print(second());
}

void check_for_shutdown()  
{
  if (!scheduler(19)) // if its NOT time to run this
  {
    return;          // then skip it
  } 
  if ((analogRead(an_acc_voltage)) <= 900)
  {
    if (shutdown_countdown == shutdown_duration)
    {
      //decrement shutdown counter by 10 mS, since this routine is on a 10mS loop
      // Print current date time to file.
      file.close();
      // List files in SD root.
      sd.ls(LS_DATE | LS_SIZE);
      Serial.println(F("Logs Saved sucsessfully, Goodbye!"));
    }  
    shutdown_countdown -= 100; 
  }
  else
  {
    mcp.digitalWrite(awake_out_pin, HIGH);
    shutdown_countdown = shutdown_duration; 
  }
  if (99 > shutdown_countdown && shutdown_countdown > -1)
  {
    mcp.digitalWrite(awake_out_pin, LOW);                          //Shutdown processor.
    Serial.println(F("VCM Shutdown Execut...e...d..."));
  }  
} // end of check for shutdown

void pack0_cell_log()
{
  if (!scheduler(23)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_pack0_cell_voltages)       // if battery cell debug is enabled...
  {
    printNow();                                  // time stamp
    file.print(",Pack 0 cell data,");            // category
    if (pack0_loss_of_comms)                     // if loss of comms is true..
    {
      file.print("Loss of Comms,");               // print "Loss of Comms"
      Serial.println("Pack 0 Loss of comms");     // print "Loss of Comms" to serial monitor
    }
    else                                         // otherwise
    {
      file.print("Comms Ok,");                    // Print "Comms Ok"
    }
    for(int i=0; i <= 95 ; i++)                  // go through each cell
    {    
      file.print(pack0_cell_voltages[i]);        // print voltage for current cell number
      file.print(",");                           // add a comma to delimit each cell
      
      if ((i+1) % 12 == 0)                       // if the cell number divides into 12 evenly, print on a new line to keep it from being obnoxiously long.
      {
        Serial.print("Cell# ");                  // print cell number
        Serial.print(i+1);                       // print cell number
        Serial.print(" ");                       // space
        Serial.print(pack0_cell_voltages[i]);    // print cell voltage
        Serial.println("V");                     // V for volts and a new line
      }
      else
      {
        Serial.print("Cell# ");                  // print cell number
        Serial.print(i+1);                       // print cell number
        Serial.print(" ");                       // space
        Serial.print(pack0_cell_voltages[i]);    // print cell voltage
        Serial.print("V ");                      // V for volts and space
      }
    }                                            // end for statement
  }
}

void pack1_cell_log()
{
  if (!scheduler(24)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_pack0_cell_voltages)       // if battery cell debug is enabled...
  {
    printNow();                                  // time stamp it
    file.print(",Pack 1 cell data,");            // category
    if (pack1_loss_of_comms)                     // if loss of comms is true..
    {
      file.print("Loss of Comms");               // print "Loss of Comms"
      Serial.println("Pack 1 Loss of comms");     // print "Loss of Comms" to serial monitor
    }
    else                                         // otherwise
    {
      file.print("Comms Ok");                    // Print "Comms Ok"
    }
    file.print(",");                             // delimiter
    for(int i=0; i <= 95 ; i++)                  // go through each cell
    {    
      file.print(pack1_cell_voltages[i]);        // print voltage for current cell number
      file.print(",");                           // add a comma to delimit each cell
      
      if ((i+1) % 12 == 0)                       // if the cell number divides into 12 evenly, print on a new line to keep it from being obnoxiously long.
      {
        Serial.print("Cell# ");                  // print cell number
        Serial.print(i+1);                       // print cell number
        Serial.print(" ");                       // space
        Serial.print(pack1_cell_voltages[i]);    // print cell voltage
        Serial.println("V");                     // V for volts and a new line
      }
      else
      {
        Serial.print("Cell# ");                  // print cell number
        Serial.print(i+1);                       // print cell number
        Serial.print(" ");                       // space
        Serial.print(pack1_cell_voltages[i]);    // print cell voltage
        Serial.print("V ");                      // V for volts and space
      }
    }                                            // end for statement
  }
}

void cell_statistics()
{
  if (!scheduler(22)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_cell_statistics)
  {
    Serial.print("Pack 0 Voltage: ");            // print label
    Serial.println(pack0_voltage);               // print pack 0 voltage
    Serial.print("Pack 1 Voltage: ");            // print label
    Serial.println(pack1_voltage);               // print pack 1 voltage   
    Serial.print("Pack 0 Cell average: ");       // print label
    Serial.println(pack0_cell_avg_volt);         // print pack 0 average cell voltage
    Serial.print("Pack 1 Cell average: ");       // print label
    Serial.println(pack1_cell_avg_volt);         // print pack 1 average cell voltage
    Serial.print("Pack 0 highest cell: ");       // print label
    Serial.println(pack0_highest_cell);          // print pack 0 highest cell
    Serial.print("Pack 1 Highest Cell: ");       // print label
    Serial.println(pack1_highest_cell);          // print pack 1 highest cell
    Serial.print("Pack 0 Lowest Cell: ");        // print label
    Serial.println(pack0_lowest_cell);           // print pack 0 lowest cell
    Serial.print("Pack 1 Lowest Cell: ");        // print label
    Serial.println(pack1_lowest_cell);           // print pack 1 lowest cell
    Serial.print("Pack 0 Average Temp: ");       // print label
    Serial.println(pack0_avg_temp);              // print pack 0 average temp
    Serial.print("Pack 1 Average Temp: ");       // print label
    Serial.println(pack1_avg_temp);              // print pack 1 average temp
  
    printNow();                                  // time stamp
    file.print(",Cell Statistics,");             // print category
    file.print(pack0_voltage);                   // print pack 0 voltage
    file.print(",");                             // delimiter
    file.print(pack1_voltage);                   // print pack 1 voltage
    file.print(",");                             // delimiter 
    file.print(pack0_cell_avg_volt);             // print pack 0 average cell voltage
    file.print(",");                             // delimiter
    file.print(pack1_cell_avg_volt);             // print pack 1 average cell voltage
    file.print(",");                             // delimiter
    file.print(pack0_highest_cell);              // print pack 0 highest cell
    file.print(",");                             // delimiter
    file.print(pack1_highest_cell);              // print pack 1 highest cell
    file.print(",");                             // delimiter
    file.print(pack0_lowest_cell);               // print pack 0 lowest cell
    file.print(",");                             // delimiter
    file.print(pack1_lowest_cell);               // print pack 1 lowest cell
    file.print(",");                             // delimiter
    file.print(pack0_avg_temp);                  // print pack 0 average temp
    file.print(",");                             // delimiter
    file.print(pack1_avg_temp);                  // print pack 1 average temp
  }
}   // end cell statistics

void drive_unit_log()
{
  if (!scheduler(21)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_drive_unit_comms)
  {
    printNow();                                  // time stamp
    file.print(",Drive unit comms,");            // category
    // to drive unit
    Serial.print("Cruise: ");
    Serial.println(canio_old & 0x01);
    file.print(canio_old & 0x01);                // cruise status
    file.print(",");                             // delimiter
    Serial.print("Start: ");
    Serial.println((canio_old & 0x02) >> 1);
    file.print((canio_old & 0x02) >> 1);         // DU start
    file.print(",");                             // delimiter
    Serial.print("Brake: ");
    Serial.println((canio_old & 0x04) >> 2);
    file.print((canio_old & 0x04) >> 2);         // brake status
    file.print(",");                             // delimiter
    Serial.print("Forward: ");
    Serial.println((canio_old & 0x08) >> 3);
    file.print((canio_old & 0x08) >> 3);         // forward status
    file.print(",");                             // delimiter
    Serial.print("Reverse: ");
    Serial.println((canio_old & 0x10) >> 4);
    file.print((canio_old & 0x10) >> 4);         // reverse status
    file.print(",");                             // delimiter
    Serial.print("BMS: ");
    Serial.println((canio_old & 0x20) >> 5);
    file.print((canio_old & 0x20) >> 5);         // bms status
    file.print(",");                             // delimiter
    Serial.print("Pot 1: ");
    Serial.println(pot_old);
    file.print(pot_old);                         // pot status
    file.print(",");                             // delimiter
    Serial.print("Pot 2: ");
    Serial.println(pot2_old);
    file.print(pot2_old);                        // pot 2 status
    file.print(",");                             // delimiter
    
    //From drive unit
    Serial.print("DC Voltage: ");
    Serial.println(udc);
    file.print(udc);                             // dc voltage
    file.print(",");                             // delimiter
    Serial.print("DC Amperage: ");
    Serial.println(idc);
    file.print(idc);                             // dc amperage
    file.print(",");                             // delimiter
    Serial.print("Speed: ");
    Serial.println(speed);
    file.print(speed);                           // motor speed
    file.print(",");                             // delimiter
    Serial.print("Inverter Temperature: ");
    Serial.println(tmphs);
    file.print(tmphs);                           // heatsink temperature
    file.print(",");                             // delimiter
    Serial.print("Motor Temperature: ");
    Serial.println(tmpm);
    file.print(tmpm);                            // motor temperature
    file.print(",");                             // delimiter
    Serial.print("Direction: ");
    Serial.println(dir);
    file.print(dir);                             // motor direction
    file.print(",");                             // delimiter
    Serial.print("Operating Mode: ");
    Serial.println(opmode);                      // operating mode
    file.print(opmode);                          // operating mode
    file.print(",");                             // delimiter
  }
}

void accelerator_pos_log() 
{
  if (!scheduler(25)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_accelerator_positions)
  {
    printNow();
    file.print(",Accel/Regen,");
    Serial.print("APS1: ");
    Serial.println(aps1);
    file.print(aps1);
    file.print(",");                             // delimiter
    Serial.print("APS2: ");
    Serial.println(aps2);
    file.print(aps2);
    file.print(",");                             // delimiter
    Serial.print("Normalized APS1: ");
    Serial.println(normalized_aps1);
    file.print(normalized_aps1);
    file.print(",");                             // delimiter
    Serial.print("Normalized APS2: ");
    Serial.println(normalized_aps2);
    file.print(normalized_aps2);
    file.print(",");                             // delimiter
    Serial.print("APS1 High: ");
    Serial.println(aps1_high);
    file.print(aps1_high);
    file.print(",");                             // delimiter
    Serial.print("APS1 Low: ");
    Serial.println(aps1_low);
    file.print(aps1_low);
    file.print(",");                             // delimiter
    Serial.print("APS2 High: ");
    Serial.println(aps2_high);
    file.print(aps2_high);
    file.print(",");                             // delimiter
    Serial.print("APS2 Low: ");
    Serial.println(aps2_low);
    file.print(aps2_low);
    file.print(",");                             // delimiter
    Serial.print("APS1 High Limit: ");
    Serial.println(aps1_high_limit);
    file.print(aps1_high_limit);
    file.print(",");                             // delimiter
    Serial.print("APS1 Low Limit: ");
    Serial.println(aps1_low_limit);
    file.print(aps1_low_limit);
    file.print(",");                             // delimiter
    Serial.print("APS2 High Limit: ");
    Serial.println(aps2_high_limit);
    file.print(aps2_high_limit);
    file.print(",");                             // delimiter
    Serial.print("APS2 Low Limit: ");
    Serial.println(aps2_low_limit);
    file.print(aps2_low_limit);
    file.print(",");                             // delimiter
    Serial.print("APS Fault: ");
    Serial.println(aps_fault);
    file.print(aps_fault); 
    file.print(",");                             // delimiter
    Serial.print("POT: ");
    Serial.println(pot_old);
    file.print(pot_old);
    file.print(",");                             // delimiter
    
    Serial.print("RPS1: ");
    Serial.println(rps1);
    file.print(rps1);
    file.print(",");                             // delimiter
    Serial.print("RPS2: ");
    Serial.println(rps2);
    file.print(rps2);
    file.print(",");                             // delimiter
    Serial.print("Normalized RPS1: ");
    Serial.println(normalized_rps1);
    file.print(normalized_rps1);
    file.print(",");                             // delimiter
    Serial.print("Normalized RPS2: ");
    Serial.println(normalized_rps2);
    file.print(normalized_rps2);
    file.print(",");                             // delimiter
    Serial.print("RPS1 High: ");
    Serial.println(rps1_high);
    file.print(rps1_high);
    file.print(",");                             // delimiter
    Serial.print("RPS1 Low: ");
    Serial.println(rps1_low);
    file.print(rps1_low);
    file.print(",");                             // delimiter
    Serial.print("RPS2 High: ");
    Serial.println(rps2_high);
    file.print(rps2_high);
    file.print(",");                             // delimiter
    Serial.print("RPS2 Low: ");
    Serial.println(rps2_low);
    file.print(rps2_low);
    file.print(",");                             // delimiter
    Serial.print("RPS1 High Limit: ");
    Serial.println(rps1_high_limit);
    file.print(rps1_high_limit);
    file.print(",");                             // delimiter
    Serial.print("RPS1 Low Limit: ");
    Serial.println(rps1_low_limit);
    file.print(rps1_low_limit);
    file.print(",");                             // delimiter
    Serial.print("RPS2 High Limit: ");
    Serial.println(rps2_high_limit);
    file.print(rps2_high_limit);
    file.print(",");                             // delimiter
    Serial.print("RPS2 Low Limit: ");
    Serial.println(rps2_low_limit);
    file.print(rps2_low_limit);
    file.print(",");                             // delimiter
    Serial.print("RPS Fault: ");
    Serial.println(rps_fault);
    file.print(rps_fault); 
    file.print(",");                             // delimiter
    Serial.print("POT2: ");
    Serial.println(pot2_old);
    file.print(pot2_old);
  }
}

void pack_data_log()
{
  if (!scheduler(26)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if (log_pack_data)
  {
    printNow();
    file.print(",Pack Data,");
    Serial.print("Pack 0 Voltage: ");
    Serial.println(pack0_voltage);
    file.print(pack0_voltage);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 amps high: ");
    Serial.println(pack0_high_range_amperage);
    file.print(pack0_high_range_amperage);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 amps low: ");
    Serial.println(pack0_low_range_amperage);
    file.print(pack0_low_range_amperage);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 1: ");
    Serial.println(pack0_temperatures[0]);
    file.print(pack0_temperatures[0]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 2: ");
    Serial.println(pack0_temperatures[1]);
    file.print(pack0_temperatures[1]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 3: ");
    Serial.println(pack0_temperatures[2]);
    file.print(pack0_temperatures[2]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 4: ");
    Serial.println(pack0_temperatures[3]);
    file.print(pack0_temperatures[3]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 5: ");
    Serial.println(pack0_temperatures[4]);
    file.print(pack0_temperatures[4]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 6: ");
    Serial.println(pack0_temperatures[5]);
    file.print(pack0_temperatures[5]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 7: ");
    Serial.println(pack0_temperatures[6]);
    file.print(pack0_temperatures[6]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 8: ");
    Serial.println(pack0_temperatures[7]);
    file.print(pack0_temperatures[7]);
    file.print(",");                             // delimiter
    Serial.print("Pack 0 temp 9: ");
    Serial.println(pack0_temperatures[8]);
    file.print(pack0_temperatures[8]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 Voltage: ");
    Serial.println(pack1_voltage);
    file.print(pack1_voltage);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 amps high: ");
    Serial.println(pack1_high_range_amperage);
    file.print(pack1_high_range_amperage);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 amps low: ");
    Serial.println(pack1_low_range_amperage);
    file.print(pack1_low_range_amperage);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 1: ");
    Serial.println(pack1_temperatures[0]);
    file.print(pack1_temperatures[0]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 2: ");
    Serial.println(pack1_temperatures[1]);
    file.print(pack1_temperatures[1]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 3: ");
    Serial.println(pack1_temperatures[2]);
    file.print(pack1_temperatures[2]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 4: ");
    Serial.println(pack1_temperatures[3]);
    file.print(pack1_temperatures[3]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 5: ");
    Serial.println(pack1_temperatures[4]);
    file.print(pack1_temperatures[4]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 6: ");
    Serial.println(pack1_temperatures[5]);
    file.print(pack1_temperatures[5]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 7: ");
    Serial.println(pack1_temperatures[6]);
    file.print(pack1_temperatures[6]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 8: ");
    Serial.println(pack1_temperatures[7]);
    file.print(pack1_temperatures[7]);
    file.print(",");                             // delimiter
    Serial.print("Pack 1 temp 9: ");
    Serial.println(pack1_temperatures[8]);
    file.print(pack1_temperatures[8]);
    file.print(",");                             // delimiter
  }                                              // end pack data log
}

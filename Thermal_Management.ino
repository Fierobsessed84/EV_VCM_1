/*  This tab contains the functions relating to thermal management. this includes the coolant pumps, AC compressor, and related valving.
 *   
 */

void setup_thermal_management()
{
  analogWriteFrequency(coolant_pump_batt_pin, coolant_pump_pwm_freq);              // configure PWM frequencies
  analogWriteFrequency(coolant_pump_drive_unit_pin, coolant_pump_pwm_freq);        // 
  analogWriteFrequency(coolant_pump_radiator_pin, coolant_pump_pwm_freq);          // 
  analogWriteFrequency(coolant_fan_1_pin, coolant_fan_pwm_freq);                   // 
  analogWriteFrequency(coolant_fan_2_pin, coolant_fan_pwm_freq);                   // 
  analogWriteFrequency(coolant_diverter_mode_pin, coolant_diverter_pwm_freq);      // 
  analogWriteFrequency(coolant_diverter_batt_pin, coolant_diverter_pwm_freq);      // 
}

void coolant_fan_pump_output()
{
  if (!scheduler(29)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  coolant_pump_batt = 0; // turn off coolant pump

  
  /*          Turns out, Chevy Volt coolant pumps dont do PWM, but if we found some that did, heres the control code for it:
   *
  if (coolant_pump_batt == 0)                                                                   // if pump is commanded off...
  {
    analogWrite(coolant_pump_batt_pin, 0);                                                      // send 0% PWM to stop pump
  }
  else                                                                                          // otherwise...
  {
    analogWrite(coolant_pump_batt_pin, (((float(max_pump_PWM - min_pump_PWM) / 100)) * \
    constrain(coolant_pump_batt, 1, 100)) + min_pump_PWM);                                      // scale pwm accordingly, 1% will kick pump on to minimum, 100% will use max pump PWM.
  }
  
  if (coolant_pump_drive_unit == 0)                                                             // if pump is commanded off...
  {
    analogWrite(coolant_pump_drive_unit_pin, 0);                                                // send 0% PWM to stop pump
  }
  else                                                                                          // otherwise...
  {
    analogWrite(coolant_pump_drive_unit_pin, (((float(max_pump_PWM - min_pump_PWM) / 100)) * \
    constrain(coolant_pump_drive_unit, 1, 100)) + min_pump_PWM);                                // scale pwm accordingly, 1% will kick pump on to minimum, 100% will use max pump PWM.
  }
  
  if (coolant_pump_radiator == 0)                                                               // if pump is commanded off...
  {
    analogWrite(coolant_pump_radiator_pin, 0);                                                  // send 0% PWM to stop pump
  }
  else                                                                                          // otherwise...
  {
    analogWrite(coolant_pump_radiator_pin, (((float(max_pump_PWM - min_pump_PWM) / 100)) * \
    constrain(coolant_pump_radiator, 1, 100)) + min_pump_PWM);                                  // scale pwm accordingly, 1% will kick pump on to minimum, 100% will use max pump PWM.
  }
  */ 

  // Code for the NON PWM pumps instead...
  // battery pump
  if (coolant_pump_batt == 0)                       // if pump is commanded off...
  {
     analogWrite(coolant_pump_batt_pin, 255);        // send full PWM, which grounds the command wire, shutting the pump off
  }
  else
  {
    analogWrite(coolant_pump_batt_pin, 0);           // Send zero PWM, which releases the command wire, turning the pump on 
  }

  // drive unit pump
  if (coolant_pump_drive_unit == 0)                 // if pump is commanded off...
  {
     analogWrite(coolant_pump_drive_unit_pin, 255);  // send full PWM, which grounds the command wire, shutting the pump off
  }
  else
  {
    analogWrite(coolant_pump_drive_unit_pin, 0);     // Send zero PWM, which releases the command wire, turning the pump on 
  }

  // radiator pump
  if (coolant_pump_radiator == 0)                   // if pump is commanded off...
  {
     analogWrite(coolant_pump_radiator_pin, 255);    // send full PWM, which grounds the command wire, shutting the pump off
  }
  else
  {
    analogWrite(coolant_pump_radiator_pin, 0);       // Send zero PWM, which releases the command wire, turning the pump on 
  }

  // PWM coolant fan 1
  if (coolant_fan_1 == 0)                                                          // if fan is commanded off...
  {
    analogWrite(coolant_fan_1_pin, 0);                                             // send 0% PWM to stop fan
  }
  else                                                                             // otherwise...
  {
    analogWrite(coolant_fan_1_pin, (((float(max_fan_PWM - min_fan_PWM) / 100)) * \
    constrain(coolant_fan_1, 1, 100)) + min_fan_PWM);                              // scale pwm accordingly, 1% will kick fans on to minimum, 100% will use max fan PWM.
  }
  
  // PWM coolant fan 2
  if (coolant_fan_2 == 0)                                                          // if fan is commanded off...
  {
    analogWrite(coolant_fan_2_pin, 0);                                             // send 0% PWM to stop fan
  }
  else                                                                             // otherwise...
  {
    analogWrite(coolant_fan_2_pin, (((float(max_fan_PWM - min_fan_PWM) / 100)) * \
    constrain(coolant_fan_2, 1, 100)) + min_fan_PWM);                              // scale pwm accordingly, 1% will kick fans on to minimum, 100% will use max fan PWM.
  }
}

void coolant_diverter_control()
{
  if (!scheduler(9)) // if its NOT time to run this
  {
    return;          // then skip it
  }
  if ((analogRead(an_acc_voltage)) <= 900)      // if we dont have power, don't try to run the diverter valves, they will try to use 3.3v logic power, that would be bad.
  {
    mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
    mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
    mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
    mcp.digitalWrite(batt_in1_pin, LOW);        // set short brake
    mcp.digitalWrite(batt_in2_pin, LOW);        // set short brake
    mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
    return;                                     // cut this function off here.
  }
  
  coolant_diverter_mode_cmd_position = 0;       // park them here for now, till we have code to operate them
  coolant_diverter_batt_cmd_position = 0;       // park them here for now, till we have code to operate them
  
  switch (coolant_diverter_calibrate_seq)
  {
    case 0:
    coolant_diverter_calibrate_seq = 1;
    coolant_diverter_mode_position_old = analogRead(an_coolant_diverter_mode);
    coolant_diverter_mode_calibrate_timer = 0;
    // read SD card
    // if read ok and valid, set coolant_diverter_calibrate_seq = 5;
    break;
      
    case 1:
    if (coolant_diverter_mode_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      coolant_diverter_calibrate_seq = 2;
      coolant_diverter_mode_high_position = coolant_diverter_mode_position;
      Serial.print("Diverter Mode High Position Set to: ");
      Serial.println(coolant_diverter_mode_high_position);
      coolant_diverter_mode_position_old = analogRead(an_coolant_diverter_mode);
      coolant_diverter_mode_calibrate_timer = 0;
      mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_mode_pin, 0);  // set 0% DC to stop movement
    }
    else
    {
      mcp.digitalWrite(mode_in1_pin, LOW);        // set forward direction
      mcp.digitalWrite(mode_in2_pin, HIGH);         // set forward direction
      mcp.digitalWrite(m_stby_pin, HIGH);          // set motors active
      analogWrite(coolant_diverter_mode_pin, 128); // set 50% DC to initiate movement
      coolant_diverter_mode_position = analogRead(an_coolant_diverter_mode);
      //Serial.print("Diverter Mode Position: ");
      //Serial.println(coolant_diverter_mode_position);
      //Serial.print("Diverter Mode Calibrate Timer: ");
      //Serial.println(coolant_diverter_mode_calibrate_timer);
      if (abs(coolant_diverter_mode_position - coolant_diverter_mode_position_old) < coolant_diverter_calibrate_window)
      {
        coolant_diverter_mode_calibrate_timer += 1;
      }
      else
      {
        coolant_diverter_mode_calibrate_timer = 0;
        coolant_diverter_mode_position_old = coolant_diverter_mode_position;
      }
      
    }
    break;

    case 2:
    if (coolant_diverter_mode_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      coolant_diverter_calibrate_seq = 3;
      coolant_diverter_mode_low_position = coolant_diverter_mode_position;
      Serial.print("Diverter Mode Low Position Set to: ");
      Serial.println(coolant_diverter_mode_low_position);
      coolant_diverter_batt_position_old = analogRead(an_coolant_diverter_batt);
      coolant_diverter_mode_calibrate_timer = 0;
      mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_mode_pin, 0);  // set 0% DC to stop movement
    }
    else
    {
      mcp.digitalWrite(mode_in1_pin, HIGH);         // set reverse direction
      mcp.digitalWrite(mode_in2_pin, LOW);        // set reverse direction
      mcp.digitalWrite(m_stby_pin, HIGH);          // set motors active
      analogWrite(coolant_diverter_mode_pin, 128); // set 50% DC to initiate movement
      coolant_diverter_mode_position = analogRead(an_coolant_diverter_mode);
      //Serial.print("Diverter Mode Position: ");
      //Serial.println(coolant_diverter_mode_position);
      //Serial.print("Diverter Mode Calibrate Timer: ");
      //Serial.println(coolant_diverter_mode_calibrate_timer);
      if (abs(coolant_diverter_mode_position - coolant_diverter_mode_position_old) < coolant_diverter_calibrate_window)
      {
        coolant_diverter_mode_calibrate_timer += 1;
      }
      else
      {
        coolant_diverter_mode_calibrate_timer = 0;
        coolant_diverter_mode_position_old = coolant_diverter_mode_position;
      }
      
    }
    break;

    case 3:
    if (coolant_diverter_batt_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      coolant_diverter_calibrate_seq = 4;
      coolant_diverter_batt_high_position = coolant_diverter_batt_position;
      Serial.print("Diverter Batt High Position Set to: ");
      Serial.println(coolant_diverter_batt_high_position);
      coolant_diverter_batt_position_old = analogRead(an_coolant_diverter_batt);
      coolant_diverter_batt_calibrate_timer = 0;
      mcp.digitalWrite(batt_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(batt_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_batt_pin, 0);  // set 0% DC to stop movement
    }
    else
    {
      mcp.digitalWrite(batt_in1_pin, LOW);        // set forward direction
      mcp.digitalWrite(batt_in2_pin, HIGH);         // set forward direction
      mcp.digitalWrite(m_stby_pin, HIGH);          // set motors active
      analogWrite(coolant_diverter_batt_pin, 128); // set 50% DC to initiate movement
      coolant_diverter_batt_position = analogRead(an_coolant_diverter_batt);
      //Serial.print("Diverter Batt Position: ");
      //Serial.println(coolant_diverter_batt_position);
      //Serial.print("Diverter Batt Calibrate Timer: ");
      //Serial.println(coolant_diverter_batt_calibrate_timer);
      if (abs(coolant_diverter_batt_position - coolant_diverter_batt_position_old) < coolant_diverter_calibrate_window)
      {
        coolant_diverter_batt_calibrate_timer += 1;
      }
      else
      {
        coolant_diverter_batt_calibrate_timer = 0;
        coolant_diverter_batt_position_old = coolant_diverter_batt_position;
      }
      
    }
    break;

    case 4:
    if (coolant_diverter_batt_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      coolant_diverter_calibrate_seq = 5;
      coolant_diverter_batt_low_position = coolant_diverter_batt_position;
      Serial.print("Diverter Batt Low Position Set to: ");
      Serial.println(coolant_diverter_batt_low_position);
      coolant_diverter_batt_calibrate_timer = 0;
      mcp.digitalWrite(batt_in1_pin, LOW);         // set short brake
      mcp.digitalWrite(batt_in2_pin, LOW);         // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);           // set motors inactive
      analogWrite(coolant_diverter_batt_pin, 0);   // set 0% DC to stop movement
      //write the calibrated values to SD card!
    }
    else
    {
      mcp.digitalWrite(batt_in1_pin, HIGH);         // set reverse direction
      mcp.digitalWrite(batt_in2_pin, LOW);        // set reverse direction
      mcp.digitalWrite(m_stby_pin, HIGH);          // set motors active
      analogWrite(coolant_diverter_batt_pin, 128); // set 50% DC to initiate movement
      coolant_diverter_batt_position = analogRead(an_coolant_diverter_batt);
      //Serial.print("Diverter Batt Position: ");
      //Serial.println(coolant_diverter_batt_position);
      //Serial.print("Diverter Batt Calibrate Timer: ");
      //Serial.println(coolant_diverter_batt_calibrate_timer);
      if (abs(coolant_diverter_batt_position - coolant_diverter_batt_position_old) < coolant_diverter_calibrate_window)
      {
        coolant_diverter_batt_calibrate_timer += 1;
      }
      else
      {
        coolant_diverter_batt_calibrate_timer = 0;
        coolant_diverter_batt_position_old = coolant_diverter_batt_position;
      }
      
    }
    break;

    case 5:  // go to commanded positions.
    if (coolant_diverter_mode_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_mode_pin, 0);  // set 0% DC to stop movement
      coolant_diverter_calibrate_seq = 6;
      Serial.println("Coolant Diverter mode Valve Failure to move");
      file.println("Coolant Diverter mode Valve Failure to move");
    }
   
    if (coolant_diverter_batt_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      mcp.digitalWrite(batt_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(batt_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_batt_pin, 0);  // set 0% DC to stop movement
      coolant_diverter_calibrate_seq = 7;
      Serial.println("Coolant Diverter batt Valve Failure to move");
      file.println("Coolant Diverter batt Valve Failure to move");
    }
    
    mode_diverter();
    batt_diverter();
    break;

    case 6: // mode valve failure, check batt valve and continue on
        if (coolant_diverter_batt_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      mcp.digitalWrite(batt_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(batt_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_batt_pin, 0);  // set 0% DC to stop movement
      coolant_diverter_calibrate_seq = 8;
      Serial.println("Coolant Diverter batt Valve Failure to move");
      file.println("Coolant Diverter batt Valve Failure to move");
    }
    batt_diverter();
    break;
    
    case 7: // batt valve failure, check mode valve and continue on
    if (coolant_diverter_mode_calibrate_timer >= coolant_diverter_calibrate_t_limit)
    {
      mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
      mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
      mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
      analogWrite(coolant_diverter_mode_pin, 0);  // set 0% DC to stop movement
      coolant_diverter_calibrate_seq = 8;
      Serial.println("Coolant Diverter mode Valve Failure to move");
      file.println("Coolant Diverter mode Valve Failure to move");
    }
    mode_diverter();
    break;
    
    case 8:
    Serial.println("Both Diverter Valves Failed to move");
    file.println("Both Diverter Valves Failed to move");
    break;
  } // end switch_coolant_diverter_calibrate_seq
} // end coolant_diverter_control

void mode_diverter()
{
  coolant_diverter_mode_position = int((256 / float(coolant_diverter_mode_high_position - coolant_diverter_mode_low_position)) * (analogRead(an_coolant_diverter_mode) - coolant_diverter_mode_low_position)); // Rescale position to 0-255
  
  if (abs(coolant_diverter_mode_cmd_position - coolant_diverter_mode_position) <  coolant_diverter_position_window)
  {
    mcp.digitalWrite(mode_in1_pin, LOW);        // set short brake
    mcp.digitalWrite(mode_in2_pin, LOW);        // set short brake
    mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
    analogWrite(coolant_diverter_mode_pin, 0);  // set 0% DC to stop movement
  }
  if (abs(coolant_diverter_mode_cmd_position - coolant_diverter_mode_position) >  coolant_diverter_position_enable)
  {
    if ((coolant_diverter_mode_cmd_position - coolant_diverter_mode_position) > 0)
    {
    mcp.digitalWrite(mode_in1_pin, LOW);       // set forward direction
    mcp.digitalWrite(mode_in2_pin, HIGH);        // set forward direction
    mcp.digitalWrite(m_stby_pin, HIGH);         // set motors active
    }
    else
    {
    mcp.digitalWrite(mode_in1_pin, HIGH);        // set reverse direction
    mcp.digitalWrite(mode_in2_pin, LOW);       // set reverse direction
    mcp.digitalWrite(m_stby_pin, HIGH);         // set motors active
    }
   
    if (abs(coolant_diverter_mode_cmd_position - coolant_diverter_mode_position) < coolant_diverter_fast_window)
    {
      analogWrite(coolant_diverter_mode_pin, 128);  // set 25% DC to slow movement
    }
    else
    {
      analogWrite(coolant_diverter_mode_pin, 255);  // set 100% DC for max speed, max damage. 
    }
    
    if (abs(coolant_diverter_mode_position - coolant_diverter_mode_position_old) < coolant_diverter_calibrate_window)
    {
      coolant_diverter_mode_calibrate_timer += 1;
    }
    else
    {
      coolant_diverter_mode_calibrate_timer = 0;
      coolant_diverter_mode_position_old = analogRead(an_coolant_diverter_mode);
    }
  }
}// end mode_diverter

void batt_diverter()
{
  coolant_diverter_batt_position = int((256 / float(coolant_diverter_batt_high_position - coolant_diverter_batt_low_position)) * (analogRead(an_coolant_diverter_batt) - coolant_diverter_batt_low_position)); // Rescale position to 0-255
  if (abs(coolant_diverter_batt_cmd_position - coolant_diverter_batt_position) <  coolant_diverter_position_window)
  {
    mcp.digitalWrite(batt_in1_pin, LOW);        // set short brake
    mcp.digitalWrite(batt_in2_pin, LOW);        // set short brake
    mcp.digitalWrite(m_stby_pin, LOW);          // set motors inactive
    analogWrite(coolant_diverter_batt_pin, 0);  // set 0% DC to stop movement
  }
  if (abs(coolant_diverter_batt_cmd_position - coolant_diverter_batt_position) >  coolant_diverter_position_enable)
  {
    if ((coolant_diverter_batt_cmd_position - coolant_diverter_batt_position) > 0)
    {
    mcp.digitalWrite(batt_in1_pin, LOW);       // set forward direction
    mcp.digitalWrite(batt_in2_pin, HIGH);        // set forward direction
    mcp.digitalWrite(m_stby_pin, HIGH);         // set motors active
    }
    else
    {
    mcp.digitalWrite(batt_in1_pin, HIGH);        // set reverse direction
    mcp.digitalWrite(batt_in2_pin, LOW);       // set reverse direction
    mcp.digitalWrite(m_stby_pin, HIGH);         // set motors active
    }
      
    if (abs(coolant_diverter_batt_cmd_position - coolant_diverter_batt_position) < coolant_diverter_fast_window)
    {
      analogWrite(coolant_diverter_batt_pin, 128);  // set 25% DC to slow movement
    }
    else
    {
      analogWrite(coolant_diverter_batt_pin, 255);  // set 100% DC for max speed, max damage. 
    }
    
    if (abs(coolant_diverter_batt_position - coolant_diverter_batt_position_old) < coolant_diverter_calibrate_window)
    {
      coolant_diverter_batt_calibrate_timer += 1;
    }
    else
    {
      coolant_diverter_batt_calibrate_timer = 0;
      coolant_diverter_batt_position_old = analogRead(an_coolant_diverter_batt);
    }
  }
}// end batt_diverter

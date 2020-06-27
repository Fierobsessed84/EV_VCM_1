
// logging
bool log_drive_unit_comms          = true;    // display and log drive unit comms
bool log_accelerator_positions     = true;    // display and log accelerator positions and other related variables
bool log_charger_comms             = false;    // display and log charger communications
bool log_pack0_cell_voltages       = false;    // display and log all battery cell voltages for pack 0
bool log_pack1_cell_voltages       = false;    // display and log all battery cell voltages for pack 1
bool log_cell_statistics           = false;    // display and log cell statistics
bool log_balancing_states          = false;    // display and log all battery balancing cell states (balancing events are always logged anyway!)
bool log_pack_data                 = false;    // display and log pack data (pack voltage, amperages and temperatures)
bool log_thermal_management        = false;    // display and log all coolant valve related stuff
bool log_charge_port               = false;    // display and log data from the charge port
bool log_traction_control          = false;    // display and log traction control variables
bool log_accelerometer             = false;    // display and log accelerometer data

unsigned long schedule[] ={  // mS interval, millis of last run (init at 0), // function    description
10, 0,    // index 0  shifter_pos                      determine shifter position
10, 0,    // index 1  du_calcs                         do drive unit calculations
10, 0,    // index 2  du_outgoing_message              do drive unit outputs
10, 0,    // index 3  du_accelerometer_read            do accelerometer read
10, 0,    // index 4  du_traction_control              do traction control routine
20, 0,    // index 5  output_balancing_can             output balancing data to both packs
20, 0,    // index 6  execute_master_reset             master reset
20, 0,    // index 7  shifter_lock                     operate shifter lock
20, 0,    // index 8  cruise_control                   do cruise control stuff
20, 0,    // index 9  coolant_diverter_control         do coolant diverter stuff
20, 0,    // index 10 flash_led                        flash onboard teensy LED
250, 0,   // index 11 fault_logic                      do simple fault logic
250, 0,   // index 12 apm_outgoing_message             send can comms to the auxiliary power module (DC DC converter)
250, 0,   // index 13 charger_outgoing_message         send can comms to the charger
250, 0,   // index 14 battery_statistics               calculate some battery statistics from the incoming data
250, 0,   // index 15 balancing_determination          figure out which cells need to be balanced, and if there are any WAY out of balance
250, 0,   // index 16 balancing_state                  count the quantity of cells balancing, and set the balance state accordingly
250, 0,   // index 17 pack0_balance_output_sequencer   runs determined balance cells through a sequencer to determine the appropriate outputs
250, 0,   // index 18 pack1_balance_output_sequencer   runs determined balance cells through a sequencer to determine the appropriate outputs
250, 0,   // index 19 check_for_shutdown               save data before shutdown
1000, 0,  // index 20 coulomb_SOC                      determine SOC based on coulombs
1000, 0,  // index 21 drive_unit_log                   log drive unit data
1000, 0,  // index 22 cell_statistics                  go log them
1000, 0,  // index 23 pack0_cell_log                   do cell output sequence (file management tab)
1000, 0,  // index 24 pack1_cell_log                   do cell output sequence (file management tab)
1000, 0,  // index 25 accelerator_pos_log              do accelerator position logging
1000, 0,  // index 26 pack_data_log                    do pack data logging
1000, 0,  // index 27 pack_comms_trace_reset           reset comms trace, will initiate pack incoming data resets if they don't start flagging as read
2000, 0,  // index 28 restore_resettables              restore fault variables that can self heal
2000, 0,  // index 29 coolant_fan_pump_output          sets PWM's for the fans and the pumps
1000, 0,  // index 30 pack0_reset_incoming_data        reset incoming data
1000, 0}; // index 31 pack1_reset_incoming_data        reset incoming data

unsigned long currentMillis           = 0;     // initialize millis comparator
unsigned long previous_010_timer      = 0;     // initialize 010ms timer
unsigned long previous_020_timer      = 0;     // initialize 020ms timer
unsigned long previous_250_timer      = 0;     // initialize 250ms timer
unsigned long previous_1000_timer     = 0;     // initialize 1000ms timer
unsigned long previous_2000_timer     = 0;     // initialize 2000ms timer

unsigned int timer_010_millis        = 10;     // actual ms of the 010 ms timer
unsigned int timer_020_millis        = 20;     // actual ms of the 020 ms timer
unsigned int timer_250_millis       = 250;     // actual ms of the 250 ms timer
unsigned int timer_1000_millis     = 1000;     // actual ms of the 1000 ms timer
unsigned int timer_2000_millis     = 2000;     // actual ms of the 2000 ms timer

const int led_pin                    = 13;     // LED pin config
const int car_on_override_pin        = 33;     // car on override output
const int awake_out_pin              = 12;     // MCP-23017 pin output to keep VCM awake
const int dc_dc_power_on_pin         = 32;     // DC DC power converter power on output
const int accessory_hv_contactor_pin = 34;     // high voltage contactor

int shutdown_countdown;                        // coundown til shutdown in mS
int const shutdown_duration        = 2000;     // keep processor on for a number of seconds after car is turned off in mS, please use 100mS increments.
byte awake_out;                                // output to keep VCM awake
int apm_voltage_req;                           // voltage requested from the auxiliary power module x 10
int apm_output_v;                              // auxiliary power module output voltage X10
int apm_temp_1;                                // auxiliary power module temperature 1 (C)
int apm_temp_2;                                // auxiliary power module temperature 1 (C)
int apm_output_a;                              // auxiliary power module output amperage

int   charger_dc_volts;                        // voltage output from charger
int   charger_ac_amps;                         // charger input amperage
float charger_dc_amps;                         // charger output amperage
int   charger_limit_amps;                      // charger output amperage limit
int   charger_prox;                            // charger prox status
int   charger_type;                            // charger type
int   charger_volt_set;                        // charger voltage set
int   max_dc_amp;                              // charger max dc amperage
int   charger_module_limit_amp;                // charger max amperage per module

// Drive Unit Control settings
// drive unit output parameters, we can only define 10 of these (input + output), but the bools are gouped into one byte, so thats only 1.

byte canio;                                    // in inverter terminal: can rx canio 256 0 6 32 (can recieve "canio" which includes cruise, start, brake, forward, reverse, bms. to can id 100 via bits 0-5 scale 32, which is = 1:1)
byte canio_old;
byte cruise;                                   // 1
byte du_start;                                 // 2
byte brake;                                    // 4
byte forward;                                  // 8
byte reverse;                                  // 16
byte bms;                                      // 32
int pot;                                       // in inverter terminal: can rx pot 256 32 16 32 (can recieve "pot" to can id 100 via bits 32-47 scale 32, which is = 1:1), set range 0-1000, and make throttle range 0-1000 in parameters
int pot2;                                      // in inverter terminal: can rx pot2 256 48 16 32 (can recieve "pot2" to can id 100 via bits 48-64 scale 32, which is = 1:1), set range 0-1000, and make throttle range 0-1000 in parameters
int pot_old;                                   // most recent value sent to the DU, the actual POT value automatically gets set to zero after it is sent, to ensure no "stuck" accelerator, but this is what WAS sent
int pot2_old;                                  // most recent value sent to the DU, the actual POT2 value automatically gets set to zero after it is sent, to ensure no "stuck" regenerative braking, but this is what WAS sent
// drive unit input parameters
int udc;                                       // in inverter terminal: can tx udc 258 0 16 1  (can transmit "udc" DC Voltage to can id 103 via bits 0-15 scale 32, which is = 1:1)
int idc;                                       // in inverter terminal: can tx idc 258 16 16 1 (can transmit "idc" DC Amperage to can id 104 via bits 16-31 scale 32, which is = 1:1) 
int speed;                                     // in inverter terminal: can tx speed 258 32 16 1 (can transmit "speed" RPM to can id 105 via bits 32-47 scale 32, which is = 1:1)
int tmphs;                                     // in inverter terminal: can tx tmphs 259 0 16 1  (can transmit "tmphs" Heatsink temperature C to can id 103 via bits 0-15 scale 32, which is = 1:1)
int tmpm;                                      // in inverter terminal: can tx tmpm 259 16 16 1 (can transmit "tmpm" Motor temperature C to can id 103 via bits 16-31 scale 32, which is = 1:1)
int dir;                                       // in inverter terminal: can tx dir 260 0 8 1  (can transmit "dir" Direction to can id 104 via bits 0-16 scale 32, which is = 1:1) -1 (255?) is rev, 0 is Neutral, +1 is forward
int opmode;                                    // in inverter terminal: can tx opmode 260 8 8 1  (can transmit "opmode" operation mode to can id 104 via bits 0-16 scale 32, which is = 1:1)

int vehicle_speed;                             // calculated vehicle speed
int speed_rpm_ratio;                           // DU RPM/MPH

// Analog inputs
int aps1;                                      // accelerator position 1, redundant to pedal
int aps2;                                      // accelerator position 2, redundant to pedal
int rps1;                                      // regen position 1, redundant
int rps2;                                      // regen position 2, redundant

float latitude;                                // read from GPS
float longitude;                               // read from GPS
int altitude;                                  // read from GPS
int gps_speed;                                 // read from GPS

const int shiftlock_pin = 7;                   // MCP23017 shift lock pin congfig
const int cruise_on_pin = 8;                   // MCP23017 cruise control on pin config
const int cruise_accel_pin = 9;                // MCP23017 cruise accel pin config
const int cruise_coast_pin = 10;               // MCP23017 cruise coast pin config
const int brake_vcm_pin = 11;                  // MCP23017 brake VCM pin config
byte cruise_on;                                // cruise control on state
byte cruise_accel;                             // crusie control accel state
byte cruise_coast;                             // cruise control coast state
int cruise_setpoint;                           // current setpoint for cruise control
int cruise_aps_limit = 25;                     // max accel aps value for cruise control
int cruise_rps_limit = 50;                     // max regen rps value for cruise control
byte brake_vcm;                                // VCM brake input state
int cruise_aps_val;                            // accelerator output value from cruise function
int cruise_rps_val;                            // regen output value from cruise function
float aps_rps_deadband = 8;                    // in percentage, this sets up an buffer to ensure full accel when demanded, and no accel when off.
float aps_rps_fault_tolerance = 15;            // in percentage, if the sensor goes past this percentage from the original settings or the redundant reading, fault the sensor
int aps1_high  = 2650;                         // accelerator position sensor 1 range high value
int aps1_low   = 570 ;                         // accelerator position sensor 1 range low value
int aps2_high  = 2000;                         // accelerator position sensor 2 range high value
int aps2_low   = 570 ;                         // accelerator position sensor 2 range low value
int rps1_high  = 4095;                         // regen position sensor 1 range high value
int rps1_low   = 0   ;                         // regen position sensor 1 range low value
int rps2_high  = 4095;                         // regen position sensor 2 range high value
int rps2_low   = 0   ;                         // regen position sensor 2 range low value
int aps1_high_limit;                           // accelerator position sensor 1 high limit
int aps1_low_limit;                            // accelerator position sensor 1 low limit
int aps2_high_limit;                           // accelerator position sensor 2 high limit
int aps2_low_limit;                            // accelerator position sensor 2 low limit
int rps1_high_limit;                           // regen position sensor 1 high limit
int rps1_low_limit;                            // regen position sensor 1 low limit
int rps2_high_limit;                           // regen position sensor 2 high limit
int rps2_low_limit;                            // regen position sensor 2 low limit
int normalized_aps1;                           // accelerator position after factoring
int normalized_aps2;                           // accelerator position after factoring
int normalized_rps1;                           // accelerator position after factoring 
int normalized_rps2;                           // accelerator position after factoring
bool aps_fault;                                // aps sensor went too far beyond calibration or deviated too far from the redundant sensor
bool rps_fault;                                // rps sensor went too far beyond calibration or deviated too far from the redundant sensor
int traction_control_aps_reduction = 0;        // accelerator reduction called out by traction control
int traction_control_rps_reduction = 0;        // regen reduction called out by traction control

// Digital inputs
const int shifter_a = 2;                       // Digital input pin assignment
const int shifter_b = 3;                       // Digital input pin assignment
const int shifter_c = 4;                       // Digital input pin assignment
const int shifter_d = 5;                       // Digital input pin assignment
const int shifter_e = 6;                       // Digital input pin assignment
const int shifter_f = 7;                       // Digital input pin assignment
const int shifter_g = 8;                       // Digital input pin assignment
const int shifter_h = 9;                       // Digital input pin assignment


int shift_state = 0;
String shifter[] = {"Invalid", "Leftshift", "Reverse",\
"Neutral", "Drive", "Upshift", "Downshift", "Rightshift"}; // Shifter states

short X_axis_accel;                            // accelerometer x axis
short Y_axis_accel;                            // accelerometer y axis
short Z_axis_accel;                            // accelerometer z axis
short pitch;                                   // gyroscope pitch
short yaw;                                     // gyroscope yaw
short roll;                                    // gyroscope roll
short tmp;                                     // MPU_6050 temperaute
short raw_X_axis_accel;
short raw_Y_axis_accel;
short raw_Z_axis_accel;
short raw_tmp;
short raw_roll;
short raw_pitch;
short raw_yaw;
const int an_aps1 = A14;                       // configure pin as Analog Accel Position Sensor 1
const int an_aps2 = A15;                       // configure pin as Analog Accel Position Sensor 2
const int an_rps1 = A16;                       // configure pin as Analog Regen Position Sensor 1
const int an_rps2 = A17;                       // configure pin as Analog Regen Position Sensor 2

const int an_acc_voltage = A13;                // 12V actual voltage reading input pin

// Thermal management settings
const int ac_requested_pin = 35;
const int an_condensor = A10;                  // condensor temperature input pin
const int an_evaporator = A11;                 // evaporator temperature input pin
const int an_ambient = A12;                    // ambient temperature input pin
const int an_ac_high_pressure = A6;            // ac high pressure input pin
const int an_ac_low_pressure = A7;             // ac low pressure input pin
const int an_coolant_diverter_mode = A0;       // coolant diverter batt feedback pin
const int an_coolant_diverter_batt = A1;       // coolant diverter batt feedback pin
const int coolant_pump_batt_pin = 10;          // battery coolant pump pin
const int coolant_pump_drive_unit_pin = 11;    // drive unit coolant pump pin
const int coolant_pump_radiator_pin = 12;      // radiator coolant pump pin
const int coolant_pump_pwm_freq = 1000;        // set pwm frequency to 1khz
const int coolant_fan_1_pin = 36;              // coolant fan 1 pin
const int coolant_fan_2_pin = 37;              // coolant fan 2 pin
const int coolant_fan_pwm_freq = 1000;         // set pwm frequency to 1khz
const int coolant_diverter_mode_pin = 28;      // coolant diverter mode PWM pin
const int coolant_diverter_batt_pin = 29;      // coolant diverter battery PWM pin
const int coolant_diverter_pwm_freq = 1000;    // set pwm frequency to 1khz
const int m_stby_pin = 0;                      // MCP-23017 coolant diverter standby pin (high is active)
const int mode_in1_pin = 1;                    // MCP-23017 coolant diverter mode in1 pin
const int mode_in2_pin = 2;                    // MCP-23017 coolant diverter mode in2 pin
const int batt_in1_pin = 3;                    // MCP-23017 coolant diverter battery in1 pin
const int batt_in2_pin = 4;                    // MCP-23017 coolant diverter battery in2 pin
byte coolant_pump_batt;                        // battery coolant pump output command %
byte coolant_pump_drive_unit;                  // drive unit coolant pump output command %
byte coolant_pump_radiator;                    // radiator coolant pump output command %
byte coolant_fan_1;                            // coolant fan 1 output command %
byte coolant_fan_2;                            // coolant fan 2 output command %
byte min_pump_PWM = 36;                        // minimum PWM to kick pumps on into low (0-255)   ** Tuns out the pumps arent PWM, so these don't get used, but I left the code in incase.
byte max_pump_PWM = 230;                       // PWM where pumps max out (0-255)                 ** Tuns out the pumps arent PWM, so these don't get used, but I left the code in incase.
byte min_fan_PWM = 36;                         // minimum PWM to kick fans on into low (0-255)
byte max_fan_PWM = 230;                        // PWM where fans max out (0-255)
byte coolant_diverter_mode_cmd_position;       // commanded position of coolant diverter valve (0-255)
int coolant_diverter_mode_position;            // position feedback from mode diverter valve
int coolant_diverter_mode_high_position;       // high position target value
int coolant_diverter_mode_low_position;        // low position target value
byte coolant_diverter_batt_cmd_position;       // commanded position of coolant diverter valve (0-255)
int coolant_diverter_batt_position;            // position feedback from battery diverter valve
int coolant_diverter_batt_high_position;       // high position target value
int coolant_diverter_batt_low_position;        // low position target value
byte coolant_diverter_position_window = 5;     // acceptable window for position targeting (0-255)
byte coolant_diverter_position_enable = 15;    // window after target is achieved to enable movement again (0-255)
byte coolant_diverter_fast_window = 50;        // go fast if they're more than this far from target (0-255)
// diverter calibration stuff
int coolant_diverter_mode_position_old;        // position feedback from previous cycle
int coolant_diverter_batt_position_old;        // position feedback from previous cycle
byte coolant_diverter_calibrate_seq;           /* position check sequence
0 = initial setup and check SD
1 = mode high check active
2 = mode low check active
3 = batt high check active
4 = batt low check active
5 = calibration complete; go to position commands instead
6 = mode fault
7 = batt fault
8 = both fault
*/
int coolant_diverter_mode_calibrate_timer;
int coolant_diverter_batt_calibrate_timer;
const byte coolant_diverter_calibrate_window = 50; // analog count window to consider complete
const int coolant_diverter_calibrate_t_limit = 50; // number of (20ms) cycles with analog count within window, basically its looking for motor stall

const int MPU_addr=0x68;  // I2C address of the MPU-6050

//lookup table for SOC percentage, based on voltage and amperage
const float SOC[289] ={\
/*                -420   -379   -339   -298   -258   -217   -176   -136    -95    -54    -14     27     68    108    149    189   230 */\
/*    3.100 */      0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,\
/*    3.175 */      0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,\
/*    3.250 */      0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,\
/*    3.325 */      1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,\
/*    3.400 */    6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,   6.5,\
/*    3.475 */     14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,    14,\
/*    3.550 */     24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,    24,\
/*    3.625 */   37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,  37.5,\
/*    3.700 */     47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,    47,\
/*    3.775 */     57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,    57,\
/*    3.850 */   65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,  65.5,\
/*    3.925 */     75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,    75,\
/*    4.000 */   82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,  82.5,\
/*    4.075 */   91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,  91.5,\
/*    4.150 */     98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,    98,\
/*    4.225 */    100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,\
/*    4.300 */    100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100,   100};

int pack0_cell_voltages[96];                   // cell voltages, pack 0
int pack1_cell_voltages[96];                   // cell voltages, pack 1

bool pack0_enabled = true;                     // is pack0 present?
bool pack1_enabled = true;                     // is pack1 present?

// Coulomb counting stuff
unsigned int pack0_size = 16000;               // size in W/H
unsigned int pack1_size = 16000;               // size in W/H
unsigned int pack0_nominal_voltage = 365;      // nominal voltage for coulomb counting
unsigned int pack1_nominal_voltage = 365;      // nominal voltage for coulomb counting

float pack0_amp_accumulator;                   // amp readings for a second. They come in at 25ms interval, so this will accumulate a total of 40 or so.
float pack1_amp_accumulator;                   // amp readings for a second. They come in at 25ms interval, so this will accumulate a total of 40 or so.
unsigned int pack0_amp_counter = 0;            // how many amp readings have we recieved since the last 1000ms coulomb update? used to find the 1 second average, start at zero
unsigned int pack1_amp_counter = 0;            // how many amp readings have we recieved since the last 1000ms coulomb update? used to find the 1 second average, start at zero
unsigned int pack0_coulomb_total;              // how many coulombs does this pack contain, figured based on total KWH and nominal voltage
unsigned int pack1_coulomb_total;              // how many coulombs does this pack contain, figured based on total KWH and nominal voltage
float pack0_coulombs_remaining;                // this is SOC in coulombs
unsigned int pack1_coulombs_remaining;         // this is SOC in coulombs
float pack0_coulomb_SOC;                       // coulomb SOC in percentage
float pack1_coulomb_SOC;                       // coulomb SOC in percentage
bool pack0_coulomb_SOC_initialized = false;    // has the coulomb SOC been set based on voltage and temperatures yet?
bool pack1_coulomb_SOC_initialized = false;    // has the coulomb SOC been set based on voltage and temperatures yet?

// cell balancing stuff
unsigned int pack0_cell_balancing[12];         //balancing bit array size in bytes, actual balance command status to the battery.
unsigned int pack1_cell_balancing[12];         //balancing bit array size in bytes, actual balance command status to the battery.
unsigned int pack0_cell_balancing_high[12];    //balancing bit array for high cell(s) active
unsigned int pack1_cell_balancing_high[12];    //balancing bit array for high cell(s) active
unsigned int pack0_cell_balancing_low[12];     //balancing bit array for low cell(s) active
unsigned int pack1_cell_balancing_low[12];     //balancing bit array for low cell(s) active

unsigned int pack0_quantity_balancing_high;    //count of cells balancing due to high voltage
unsigned int pack1_quantity_balancing_high;    //count of cells balancing due to high voltage
unsigned int pack0_quantity_balancing_low;     //count of cells balancing duw to low voltage
unsigned int pack1_quantity_balancing_low;     //count of cells balancing duw to low voltage

unsigned int pack0_balance_sequencer;          //used for sequencing high and low balancing strategies
unsigned int pack1_balance_sequencer;          //used for sequencing high and low balancing strategies
unsigned int pack0_cell_balancing_state;       // 0 is no balancing active, 1 is high balancing active, 2, is low balancing active, 3 is both high/low active.
unsigned int pack1_cell_balancing_state;       // 0 is no balancing active, 1 is high balancing active, 2, is low balancing active, 3 is both high/low active.
unsigned int volt_select;                      // used during candecode for referencing the correct cells voltages

int pack0_voltage;                             // total voltage measured
int pack1_voltage;                             // total voltage measured
float pack0_high_range_amperage;               // amperage of the high range current sensor
float pack1_high_range_amperage;               // amperage of the high range current sensor
float pack0_low_range_amperage;                // amperage of the low range current sensor
float pack1_low_range_amperage;                // amperage of the low range current sensor
float pack0_temperatures[9];                   // array of the 9 pack temperature sensors
float pack1_temperatures[9];                   // array of the 9 pack temperature sensors
int pack0_cell_sum_volt;                       // totalled up cell voltage. In theory, should match pack voltage
int pack1_cell_sum_volt;                       // totalled up cell voltage. In theory, should match pack voltage
float pack0_sum_temp;                          // totalled up pack temperature, just for average calc.
float pack1_sum_temp;                          // totalled up pack temperature, just for average calc.
float pack0_avg_temp;                          // average temoerature of the pack sensors
float pack1_avg_temp;                          // average temoerature of the pack sensors
int pack0_avg_volt;                            // average cell voltage based on the overall measured pack voltage
int pack1_avg_volt;                            // average cell voltage based on the overall measured pack voltage
int pack0_cell_avg_volt;                       // average cell voltage based on totalled cell voltage measurements
int pack1_cell_avg_volt;                       // average cell voltage based on totalled cell voltage measurements
int pack0_highest_cell = 0;                    // voltage of the highest cell
int pack1_highest_cell = 0;                    // voltage of the highest cell
int pack0_lowest_cell = 0;                     // voltage of the lowest cell
int pack1_lowest_cell = 0;                     // voltage of the lowest cell
bool pack0_cell_high_fault   = false;          // a cell went too far above pack average
bool pack1_cell_high_fault   = false;          // a cell went too far above pack average
bool pack0_cell_high_warning = false;          // a cell went too far above pack average
bool pack1_cell_high_warning = false;          // a cell went too far above pack average
bool pack0_cell_low_fault    = false;          // a cell went too far below pack average
bool pack1_cell_low_fault    = false;          // a cell went too far below pack average
bool pack0_cell_low_warning  = false;          // a cell went too far below pack average
bool pack1_cell_low_warning  = false;          // a cell went too far below pack average
bool pack0_inhibit_high_balancing;             // balancing inhibited
bool pack1_inhibit_high_balancing;             // balancing inhibited
bool pack0_inhibit_low_balancing;              // balancing inhibited
bool pack1_inhibit_low_balancing;              // balancing inhibited
bool charging_allowed;                         // charging allowed
int pack_comms_timeout = 500;                  // update timeout before we reset all battery data to zero in mS
int pack0_comms_timeout_clock;                 // time since last battery update
byte pack0_comms_timeout_trace = 0;            // bit 1 high is 0x200 rx'd, bit 2 = 0x202, bit 3 = 0x204, 4 = 0x206 5 = 0x210 6 = 0x302
int pack1_comms_timeout_clock;                 // time since last battery update
byte pack1_comms_timeout_trace = 0;            // bit 1 high is 0x200 rx'd, bit 2 = 0x202, bit 3 = 0x204, 4 = 0x206 5 = 0x210 6 = 0x302
bool pack0_loss_of_comms     = false;          // cell voltages are evaluating as zero, so we've lost comms somewhere
bool pack1_loss_of_comms     = false;          // cell voltages are evaluating as zero, so we've lost comms somewhere
bool master_reset;                             // this will go to a button or comm signal later on
bool charging                = true;           // Lying that we're in charge mode, for testing.
unsigned int balance_loop;                     // This counter is for sequencing the commands to the battery
unsigned int car_mode = 0x02;                  // 0x02 is chevy volt speak for "charging". cells won't balance in any other modes... I think.
float pack0_state_of_charge;                   // state of charge
float pack1_state_of_charge;                   // state of charge
unsigned int charge_amperage_allowed;          // max amperage allowed during charge

// battery settings! in millivolts.
const int balance_high_enable          = 0040; // difference over average to trigger a balance high command, must be higher than balance_high_enable
const int balance_high_complete        = 0015; // difference over average to clear off a balance low command, must be lower than balance_high_enable
const int charge_reduction_tolerance   = 0050; // difference over average to reduce charge amperage
const int cell_high_warning_tolerance  = 0150; // difference over average to cause a battery warning
const int cell_high_fault_tolerance    = 0250; // difference over average to cause a battery fault
const int cell_high_warning_threshold  = 4150; // voltage limit to trip a warning
const int cell_high_fault_threshold    = 4200; // voltage limit to trip a fault
const int balance_low_enable          = -0040; // difference below average to trigger a balance command, must be lower than (more negative) negbalance_low_complete
const int balance_low_complete        = -0015; // difference below average to clear off a balance command, must be higher (less negative) than balance_low_enable 
const int cell_low_warning_tolerance  = -0150; // difference below average to cause a battery warning
const int cell_low_fault_tolerance    = -0300; // difference below average to cause a battery fault
const int cell_low_warning_threshold  = -3300; // difference below average to cause a battery warning
const int cell_low_fault_threshold    = -3200; // difference below average to cause a battery fault
const int battery_cell_average_limit  = -4200; // highest cell voltage allowed before regen and charging are forced off
const int pack_voltage_limit            = 408; // 1:1 VOLT! highest pack voltage allowed before regen and charging are forced off
const int pack_temp_warning               = 8; // difference from average in C to cause a pack thermal warning

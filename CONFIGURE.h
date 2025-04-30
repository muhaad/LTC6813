

#ifndef CONFIGURE_H  
#define CONFIGURE_H

#include <stdint.h>

//Safe operating conditions
const float max_temp = 60;
const float min_temp = 0;
const float OV = 4.20;       //over-voltage limit (spelled with an "oh" not zero) (V)
const float UV = 2.8;       //under-voltage limit (V)
const int watchdog_timeout = 0;  //watchdog timeout (in seconds). setting to 0 will DISABLE timer. 

//architecture
const int num_boards = 4;
const int num_cells = 17;       //cells per board

//BMS operation mode. Leave empty to determine mode during runtime
String mode = "";     //"", "charge", "drive", "debug"

//CAN Bus Parameters
uint16_t BMS_ID = 0x123;             //standard ID of BMS TX messages
uint32_t INV_TX_ID = 0x0A7;         //CAN Message ID of message send from inverter of DC Bus Voltage (100 Hz frequency).
uint32_t CHG_TX_ID = 0x18FF50E5;    //CAN Message ID of messages sent from charger

//charging parameters
uint16_t CHG_voltage = 300;
uint16_t CHG_current = 4;
float _qt = 12.6 * 60; //total capacity (coulumbs): total capacity (Ah) * 60s/1hr

//balancing parameters
float balance_threshold = 3.85;    //will not balance cells below this threshold (V)
float max_differnce = 0.3;    //will not continue charging if max-min cell exceeds this threshold

//sense board parameters
int ADC_mode = 0;     //integer 0-7 to set ADC sampling frequency
#define wake_delay 2    //wake delay per board (milliseconds) to bring up power supply to voltage. Depends on Linear voltage regulator capacitance
#define cell_RC 0.0001  //C pin filter RC time constant in milliseconds (R*C*1000)

//SD Card
float SD_card_size = 116;   //SD card size in Gb
#endif
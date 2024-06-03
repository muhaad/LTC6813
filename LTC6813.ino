#include <SPI.h>
#include <cmath>
#include "LTC681x.h"
#include "COMMANDS.h"
#include "HEADER.h"
#include "LUTS.h"
#include "Watchdog_t4.h"

#include <FlexCAN_T4.h>


#define CRX3 23
#define CTX3 22
#define STBY 21     //CAN Transceiver Standby

uint16_t CHG_voltage = 588;
uint16_t CHG_current = 9;

bool CHG_EN = 0; //0: enable charging, 1: disable charging

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

WDT_T4<WDT1> wdt;     //watchdog 1 holds output pin low until power-on-reset. This is desired for a shutdown circuit

void myCallback() {               
  //Serial.println("FEED THE DOG SOON, OR RESET!");
}

//LTC6813 minimum supply voltage is 16V

#define CS 10   //chip select pin 
#define num_boards 10
#define num_cells 14       //cells per board
#define max_temp 50
#define min_temp 0
#define wake_delay 2    //wake delay per board (milliseconds) to bring up power supply to voltage. Depends on Linear voltage regulator capacitance
#define cell_RC 0.0001  //C pin filter RC time constant in milliseconds (R*C*1000)

int wire_cut = 0;
float cell_voltage[num_boards][num_cells];     //most recent cell voltages
float cell_temp[num_boards][9];                //most recent cell temperatures. Contans raw voltage data for the duration of open wire checks
float GPIO_open_wire[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];

float OV = 4.2;       //over-voltage limit (spelled with an "oh" not zero)
float UV = 2.8;       //under-voltage limit       abs

void setup() {
  delay(1000);
  Serial.println("startup");
  pinMode(CS,OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  //CAN
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);

  can.begin();
  can.setBaudRate(250000);
  can.enableFIFO();

  //Watchdog
  WDT_timings_t config;
  //config.trigger = 4; /* in seconds, 0->128 */    //time until watchdog callback function is triggered. 
  config.timeout = 5; /* in seconds, 0->128 */   //time until watchdog reset
  config.pin = 20;                                //pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
  //config.callback = myCallback;
  wdt.begin(config);
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);


}

void loop() {

  digitalWrite(20, LOW);

  while(1){
  measure_voltage();
  measure_temp();
  measure_current();
  send_CAN();
  reset_watchdog();
  delay(1000);  
  }

}


void send_command(uint16_t command){
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t cmd0;
  uint8_t cmd1;

  cmd0 = command >> 8;
  cmd1 = command >> 0;

  wakeup_sleep(num_boards);

  delay(2);                 //small delay is needed after wake to bring up power supply
  digitalWrite(CS, LOW);

  comm_arr[0] = cmd0;
  comm_arr[1] = cmd1;

  pec = pec15_calc(2, comm_arr);
  pec1 = pec >> 0;
  pec0 = pec >> 8;

  SPI.transfer(cmd0);
  SPI.transfer(cmd1);
  SPI.transfer(pec0);
  SPI.transfer(pec1);
}

void read_register_group(uint16_t command, uint8_t response[num_boards][6]){      //register group is always 6 bytes 

  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t response_pec0;
  uint8_t response_pec1;

  send_command(command);

  //Serial.println("Response");
  for (int i = 0; i < num_boards; i++){
    for (int j = 0; j < 6; j++) {
      response[i][j] = SPI.transfer(0b11111111); // Send dummy byte to receive data
      //Serial.println(response[i][j], BIN); 
    }
      response_pec0 = SPI.transfer(0xFF);
      response_pec1 = SPI.transfer(0xFF);
      pec = pec15_calc(6, response[i]);
      pec1 = pec >> 0;
      pec0 = pec >> 8;
      //Serial.println("Response");
      //Serial.println(response_pec0);
      //Serial.println(response_pec1);
      //Serial.println("Calc");
      //Serial.println(pec0);
      //Serial.println(pec1); 
//Serial.println('\n');
  }

      pec = pec15_calc(6, response[0]);     //this needs fixed to include multiple boards

      // Serial.println("response pec");
      // Serial.println(response_pec0, BIN);
      // Serial.println(response_pec1, BIN);
      // Serial.println("calculated pec");
      // Serial.println(pec, BIN);

    digitalWrite(CS, HIGH);

}

void write_register_group(uint16_t command, uint8_t data[6]){

  uint8_t return_data;
  uint8_t data_pec0;
  uint8_t data_pec1;
  uint16_t data_pec = pec15_calc(6, data);

  send_command(command);
  
  data_pec1 = data_pec >> 0;
  data_pec0 = data_pec >> 8;

  for(int i=0; i<6 ; i++){
    SPI.transfer(data[i]);
  }

  return_data = SPI.transfer(data_pec0);
  return_data = SPI.transfer(data_pec1);

  digitalWrite(CS, HIGH);

}

void poll_ADC(uint16_t command){
  uint8_t return_data = 0;

  send_command(command);

  int num_polls = 0;
    while (return_data == 0) {
      return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
      num_polls++;
    }
  //Serial.println("ADC Conversion Done!");
  //Serial.println(num_polls);
  digitalWrite(CS, HIGH);
}

void measure_voltage(){
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};   //read cell voltage registers A through E commands

  ////cell voltage measurement algorithm outlined in INTERNAL PROTECTION AND FILTERING section of LTC6813 datasheet////
  poll_ADC(ADCV | 0b1);   //measure cells 1,7,13 to allow MUX voltage to settle
  delay(cell_RC * 6);
  poll_ADC(ADCV);   //initiate and wait for voltage measurement

  for(int i=0; i*3 < num_cells; i++){         //i: cell group
    //Serial.print('i');
    //Serial.println(i);
    uint16_t curr_comm = cell_comm[i];                 //each command reads a sequential set of three cells from each board
    read_register_group(curr_comm, response);

    for(int j=0; j < num_boards; j++){        //j:board number
      //Serial.print('j');
      //Serial.println(j);
      for(int k=0; k < 3 && i*3+k < num_cells; k++){   //cell number within register group
      //Serial.print('k');
      //Serial.println(k);
        cell_voltage[j][i*3+k] = (float)(((uint8_t)response[j][k*2+1] << 8) | response[j][k*2]) * 0.0001;  //LSB represents 100 uV
      }
    }
  }

  Serial.println("Voltages:");
  int g = 0;
  for(int i=0; i<num_boards; i++){
    Serial.print("board: "); Serial.println(i+1);
    for(int j=0; j<num_cells; j++){
       Serial.println(cell_voltage[i][j]);   
       g++; 
    }
    Serial.println('\n');
  }
 
}

float map_temp(float V){
  int i;
  int size = sizeof(NTC_LUT) / sizeof(NTC_LUT[0]);
  float R_bias = 10200;
  float V_ref = 3.00;

  if(V_ref == V){   //divide by zero case
    return -40;
  }

  float NTC_res = (V/V_ref*R_bias)/(1-V/V_ref);

  float dist = std::abs(NTC_res - NTC_LUT[0]);

  for(i = 1; i<size; i++){
    float new_dist = std::abs(NTC_res - NTC_LUT[i]);
    if(new_dist < dist){
      dist = new_dist;
    }
    else{
      i--;
      break;
    }
  }
  float temperature = float(i)/float(size)*(150+40)-40;
  return(temperature);
}

void measure_temp(bool open_wire_check){
  uint8_t response[num_boards][6];
  uint16_t aux_comm[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};   //read aux registers A through D commands
  int temp_num = 0;       //temperature reading index 0-8
  int command_num = 0;    //command index within aux_comm array

  if(open_wire_check == false){
    poll_ADC(ADAX);   //initiate and wait for GPIO measurement
  }
  else{
    poll_ADC(AXOW);
  }

  while(temp_num < 9){
    uint16_t curr_comm = aux_comm[command_num];                 //each command reads a sequential set of three GPIO from each board (RDAUXB is an exception with just 2 GPIO)
    read_register_group(curr_comm, response);
    for(int k = 0; k < 3 && temp_num < 9; k++){
      if(command_num == 1 && k>1){                              //register group B only contains 2 GPIO measurements
        continue;
      }
      for(int j = 0; j< num_boards; j++){                               //maximum of 3 GPIO per register group and 9 thermistors
              cell_temp[j][temp_num] = (float)(((uint8_t)response[j][k*2+1] << 8) | response[j][k*2]) * 0.0001;  //LSB represents 100 uV
              //Serial.println(cell_temp[j][temp_num]);
      }   
      temp_num++;
    }
  command_num++;
  }

  Serial.println("Tempearatures:");
  for(int i=0; i<num_boards; i++){
    Serial.print("board: "); Serial.println(i+1);
    for(int j=0; j<9; j++){
        //Serial.println(cell_temp[i][j]);   
        Serial.println(map_temp(cell_temp[i][j]));
        cell_temp[i][j] = map_temp(cell_temp[i][j]);
    }
    Serial.println('\n');
  }



}

void reset_watchdog(){
  for(int i = 0; i < num_boards; i++){
    for(int j = 0; j< num_cells; j++){
      if(cell_voltage[i][j] < OV && cell_voltage[i][j] > UV){
        continue;
      }
      else{
          Serial.println("invalid voltage");
          Serial.println(cell_voltage[i][j]);
          digitalWrite(20, LOW);
          return;
      }
    }
  }

    for(int i = 0; i < num_boards; i++){
    for(int j = 0; j< 8; j++){          //9th temp sensor wired incorrectly
      if((cell_temp[i][j] > min_temp && cell_temp[i][j] < max_temp) || (i==7 && j == 7)){   //board 8 temp sensor 8 open
        continue;
      }
      else{
          Serial.println("invalid temp");
          digitalWrite(20, LOW);
          return;
      }
    }
  }
  digitalWrite(20, HIGH);
  wdt.feed();  
  
}

void sense_status(){

}

void measure_current(){
    float R1 = 10000;   //bottom resistor in voltage divider (ohms)
    float R2 = 5100;    //top resistor in voltage divier (ohms)
    int ADC_in;
    float ADC_volt;
    float Hall_volt;
    float current;
    ADC_in = analogRead(A11);       // 0-1023 integer
    //Serial.println(ADC_in);
    ADC_volt = float(ADC_in)/1023*3.3;
    //Serial.println(ADC_volt);
    Hall_volt = ADC_volt*(10000+5100)/10000;
    //Serial.println(Hall_volt);
    current = (Hall_volt-0.25)/(4.5)*(100)-50; 
    Serial.println("Current");       
    Serial.println(current);
    Serial.println();
}

void send_CAN(){
  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);

  CAN_message_t CHGR_EN;
  CHGR_EN.id = 0x1806E6F4;  // Set the CAN message ID
  CHGR_EN.len = 8;     // Set the data length

  CHGR_EN.buf[0] = (uint8_t)(CHG_voltage*10 >> 8);
  CHGR_EN.buf[1] = (uint8_t)(CHG_voltage*10);
  CHGR_EN.buf[2] = (uint8_t)(CHG_current*10 >> 8);
  CHGR_EN.buf[3] = (uint8_t)(CHG_current*10);
  CHGR_EN.buf[4] = (uint8_t)(CHG_EN);
  CHGR_EN.buf[5] = 0;
  CHGR_EN.buf[6] = 0;
  CHGR_EN.buf[7] = 0;

  can.write(CHGR_EN);
  Serial.println("CAN message sent");
}

void RX_CAN(){

  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);

  CAN_message_t msg;
  bool received = false;
  while (received == false) {
    can.read(msg);
    Serial.print("ID: ");
    Serial.print(msg.id, HEX);
    Serial.println(" Data: ");
    msg.len = 20;
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], BIN);
      Serial.print(" ");
    }
    received = true;
    Serial.print('\n');
  }

}



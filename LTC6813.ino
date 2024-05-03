#include <SPI.h>
#include <cmath>
#include "LTC681x.h"
#include "COMMANDS.h"
#include "LUTS.h"
#include "Watchdog_t4.h"

WDT_T4<WDT1> wdt;

void myCallback() {               
  Serial.println("FEED THE DOG SOON, OR RESET!");
}

//LTC6813 minimum supply voltage is 16V

#define CS 10   //chip select pin 
#define num_boards 1
#define num_cells 14       //cells per board

int wire_cut = 0;
float cell_voltage[num_boards][num_cells];     //most recent cell voltages
float cell_temp[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];

float OV = 4.2;       //over-voltage limit (spelled with an "oh" not zero)
float UV = 2.8;       //under-voltage limit

void setup() {
  delay(1000);
  Serial.println("startup");
  pinMode(CS,OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));


  //Watchdog
  WDT_timings_t config;
  config.trigger = 4; /* in seconds, 0->128 */    //time until watchdog callback function is triggered. 
  config.timeout = 5; /* in seconds, 0->128 */   //time until watchdog reset
  config.pin = 20;                                //pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
  config.callback = myCallback;
  wdt.begin(config);
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);


}

void loop() {

  digitalWrite(20, LOW);


  uint8_t response[6];
  uint8_t data[6];
  while(1){
  wakeup_sleep(num_boards);

  measure_voltage();
  measure_temp();
  reset_watchdog();
  wdt.feed();  
  delay(1000);  
  }

}

void read_register_group(uint16_t command, uint8_t response[num_boards][6]){      //register group is always 6 bytes 

  uint8_t return_data;
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t response_pec0;
  uint8_t response_pec1;

  uint8_t cmd0;
  uint8_t cmd1;

  cmd0 = command >> 8;
  cmd1 = command >> 0;

  wakeup_sleep(num_boards);
  delay(2);             //small delay is needed after wake to bring up power supply
  digitalWrite(CS, LOW);

  comm_arr[0] = cmd0;
  comm_arr[1] = cmd1;

  pec = pec15_calc(2, comm_arr);
  pec1 = pec >> 0;
  pec0 = pec >> 8;

  return_data = SPI.transfer(cmd0);
  return_data = SPI.transfer(cmd1);
  return_data = SPI.transfer(pec0);
  return_data = SPI.transfer(pec1);

  //Serial.println("Response");
  for (int i = 0; i < num_boards; i++){
    for (int j = 0; j < 6; j++) {
      response[i][j] = SPI.transfer(0b11111111); // Send dummy byte to receive data
      //Serial.println(response[i][j], BIN); 
    }
      response_pec0 = SPI.transfer(0xFF);
      response_pec1 = SPI.transfer(0xFF);
      pec = pec15_calc(6, response[0]);
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
    wakeup_sleep(num_boards);

  delay(2);               //small delay is needed to bring up LTC6813 regulated voltage
  digitalWrite(CS, LOW);

  uint8_t return_data;
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t cmd0;
  uint8_t cmd1;


  cmd0 = command >> 8;
  cmd1 = command >> 0;

  comm_arr[0] = cmd0;
  comm_arr[1] = cmd1;
  
  pec = pec15_calc(2, comm_arr);

  pec1 = pec >> 0;
  pec0 = pec >> 8;

  uint8_t data_pec0;
  uint8_t data_pec1;
  
  uint16_t data_pec = pec15_calc(6, data);
  data_pec1 = data_pec >> 0;
  data_pec0 = data_pec >> 8;

    delay(2);

    return_data = SPI.transfer(cmd0);
    return_data = SPI.transfer(cmd1);
    return_data = SPI.transfer(pec0);
    return_data = SPI.transfer(pec1);

    for(int i=0; i<6 ; i++){
      SPI.transfer(data[i]);
    }

  return_data = SPI.transfer(data_pec0);
  return_data = SPI.transfer(data_pec1);

  digitalWrite(CS, HIGH);

}

void poll_ADC(uint16_t command){
  uint8_t return_data;
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t cmd0;
  uint8_t cmd1;

  cmd0 = command >> 8;
  cmd1 = command >> 0;

    wakeup_sleep(num_boards);

  delay(2);             //small delay is needed after wake to bring up power supply
  digitalWrite(CS, LOW);

  comm_arr[0] = cmd0;
  comm_arr[1] = cmd1;

  pec = pec15_calc(2, comm_arr);
  pec1 = pec >> 0;
  pec0 = pec >> 8;

  return_data = SPI.transfer(cmd0);
  return_data = SPI.transfer(cmd1);
  return_data = SPI.transfer(pec0);
  return_data = SPI.transfer(pec1);

  return_data = 0b00000000;
  int num_polls = 0;
    while (return_data == 0) {
      return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
      num_polls++;
    }
  Serial.println("ADC Conversion Done!");
  Serial.println(num_polls);
  digitalWrite(CS, HIGH);
}

void measure_voltage(){
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};   //read cell voltage registers A through E commands

  poll_ADC(ADCV);   //initiate and wait for voltage measurement

  for(int i=0; i*3 < num_cells; i++){         //i: cell group
    Serial.print('i');
    Serial.println(i);
    uint16_t curr_comm = cell_comm[i];                 //each command reads a sequential set of three cells from each board
    read_register_group(curr_comm, response);

    for(int j=0; j < num_boards; j++){        //j:board number
      //Serial.print('j');
      //Serial.println(j);
      for(int k=0; k < 3 && i*3+k < num_cells; k++){   //cell number within group
      //Serial.print('k');
      //Serial.println(k);
        cell_voltage[j][i*3+k] = (float)(((uint8_t)response[j][k*2+1] << 8) | response[j][k*2]) * 0.0001;  //LSB represents 100 uV
      }
    }
  }

  Serial.println("Voltages:");
  int g = 0;
  for(int i=0; i<num_boards; i++){
    for(int j=0; j<num_cells; j++){
       Serial.println(cell_voltage[i][j]);   
       g++; 
    }
    Serial.print("next board");
    Serial.println('\n');
  }
 
}

float map_temp(float V){
  int i;
  int size = sizeof(NTC_LUT) / sizeof(NTC_LUT[0]);
  float NTC_res;
  float dist = std::abs(NTC_res - NTC_LUT[0]);
  float V_ref = 3;
  float R_bias = 10200;
  //Serial.println(V);
  NTC_res = (V/V_ref*R_bias)/(1-V/V_ref);
  //Serial.print("NTC_resistance");
  //Serial.println(NTC_res);


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

void measure_temp(){
  uint8_t response[num_boards][6];
  uint16_t aux_comm[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};   //read aux registers A through E commands

  poll_ADC(ADAX);   //initiate and wait for GPIO measurement

  for(int i=0; i*3 < 9; i++){         //i: aux group
    //Serial.print('i');
    //Serial.println(i);
    uint16_t curr_comm = aux_comm[i];                 //each command reads a sequential set of three GPIO from each board
    read_register_group(curr_comm, response);

    for(int j=0; j < num_boards; j++){        //j:board number
      //Serial.print('j');
      Serial.println(j);
      for(int k=0; k < 3 && i*3+k < 9; k++){   //k: GPIO number within group (1st or 2nd GPIO in register)
      //Serial.print('k');
      //Serial.println(k);
        cell_temp[j][i*3+k] = (float)(((uint8_t)response[j][k*2+1] << 8) | response[j][k*2]) * 0.0001;  //LSB represents 100 uV
      }
    }
  }

  Serial.println("Tempearatures:");
  for(int i=0; i<num_boards; i++){
    for(int j=0; j<9; j++){
       //Serial.println(cell_temp[i][j]);   
       Serial.println(map_temp(cell_temp[i][j]));
       cell_temp[i][j] = map_temp(cell_temp[i][j]);
    }
    Serial.print("next board");
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
          digitalWrite(20, LOW);                           //Look for the orange LED driven low upon reset
          return;
      }
    }
  }

    for(int i = 0; i < num_boards; i++){
    for(int j = 0; j< 9; j++){
      if((cell_temp[i][j] > 0 && cell_temp[i][j] < 60) || j == 5){
        continue;
      }
      else{
          Serial.println("invalid temp");
          digitalWrite(20, LOW);                           //Look for the orange LED driven low upon reset
          return;
      }
    }
  }
  digitalWrite(20, HIGH);
  Serial.println("here");
}

#include <SPI.h>
#include "LTC681x.h"
#include "COMMANDS.h"

//LTC6813 minimum supply voltage is 16V

#define CS 10   //chip select pin 
#define num_boards 1

int wire_cut = 0;
float cell_voltage[18];     //most recent cell voltages
unsigned long Time;


void setup() {
  delay(3000);
  pinMode(CS,OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(14000, MSBFIRST, SPI_MODE0));
}

void loop() {
  
  uint8_t response[6];
  uint8_t data[6];

  data[0] = 0b00000000;     //register 0 in group.    1 : GPIO high
  data[1] = 0b11111111;
  data[2] = 0b11111111;
  data[3] = 0b11110111;
  data[4] = 0b11111111;
  data[5] = 0b00000000;


 measure_voltage();
  measure_voltage();


while(1){}

  read_register_group(RDCVA, response);
  poll_ADC(ADCV);
  read_register_group(RDCVA, response);

  write_register_group(WRCFGA, data);
  Serial.print("register A ");
  read_register_group(RDCFGA, response);
  Serial.print("register B ");
  write_register_group(WRCFGB, data);
  read_register_group(RDCFGB, response);
}

void read_register_group(uint16_t command, uint8_t response[6]){      //register group is always 6 bytes 

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

  wakeup_sleep(1);
  delay(2);             //small delay is needed after wake to bring up power supply (3 time constants of 5V capacitor)
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

  Serial.println("Response");
  delay(2);
    for (int i = 0; i < 6; ++i) {
      //delay(.01);
      response[i] = SPI.transfer(0b11111111); // Send dummy byte to receive data
      Serial.println(response[i], BIN); 
    }

    delay(1);

      response_pec0 = SPI.transfer(0xFF);
      response_pec1 = SPI.transfer(0xFF);

      pec = pec15_calc(6, response);

      Serial.println("response pec");
      Serial.println(response_pec0, BIN);
      Serial.println(response_pec1, BIN);
      Serial.println("calculated pec");
      Serial.println(pec, BIN);

    digitalWrite(CS, HIGH);

}

void write_register_group(uint16_t command, uint8_t data[6]){
  wakeup_sleep(1);
  delay(2);
  digitalWrite(CS, LOW);

  uint8_t return_data;
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t cmd0 = 0b00000000;
  uint8_t cmd1 = 0b00000001;  //write config reg A
  cmd0 = command >> 8;
  cmd1 = command >> 0;  //write config reg A

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

  wakeup_sleep(1);
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

  Serial.println("Response");

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
  uint8_t response[6];
  uint16_t cell_voltage_binary[18];
  //poll_ADC(ADCV);
  read_register_group(RDCVA, response);
  cell_voltage_binary[0] = ((uint8_t)response[1] << 8) | response[0];
  cell_voltage_binary[1] = ((uint8_t)response[3] << 8) | response[2];
  cell_voltage_binary[2] = ((uint8_t)response[5] << 8) | response[4];
  read_register_group(RDCVB, response);
  cell_voltage_binary[3] = ((uint8_t)response[1] << 8) | response[0];
  cell_voltage_binary[4] = ((uint8_t)response[3] << 8) | response[2];
  cell_voltage_binary[5] = ((uint8_t)response[5] << 8) | response[4];
  read_register_group(RDCVC, response);
  cell_voltage_binary[6] = ((uint8_t)response[1] << 8) + response[0];
  cell_voltage_binary[7] = ((uint8_t)response[3] << 8) + response[2];
  cell_voltage_binary[8] = ((uint8_t)response[5] << 8) + response[4];
  read_register_group(RDCVD, response);
  cell_voltage_binary[9] = ((uint8_t)response[1] << 8) + response[0];
  cell_voltage_binary[10] = ((uint8_t)response[3] << 8) + response[2];
  cell_voltage_binary[11] = ((uint8_t)response[5] << 8) + response[4];
  read_register_group(RDCVE, response);
  cell_voltage_binary[12] = ((uint8_t)response[1] << 8) + response[0];
  cell_voltage_binary[13] = ((uint8_t)response[3] << 8) + response[2];
  cell_voltage_binary[14] = ((uint8_t)response[5] << 8) + response[4];
  read_register_group(RDCVF, response);
  cell_voltage_binary[15] = ((uint8_t)response[1] << 8) + response[0];
  cell_voltage_binary[16] = ((uint8_t)response[3] << 8) + response[2];
  cell_voltage_binary[17] = ((uint8_t)response[5] << 8) + response[4];

  float temp;
  for(int i = 0; i<18; i++){
    temp = cell_voltage_binary[i];
    cell_voltage[i] = 5*temp;
    Serial.println(cell_voltage_binary[i]*0.0001);    //LSB represents 100 uV
  }

}

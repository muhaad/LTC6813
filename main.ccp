#include <SPI.h>
#include "LTC681x.h"


#define CS 10   

void setup() {
  pinMode(CS,OUTPUT);
  SPI.begin();
}

void loop() {
  
  uint8_t return_data;

  SPI.beginTransaction(SPISettings(14000, MSBFIRST, SPI_MODE0));

  uint8_t comm_len = 2;
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;

uint8_t cmd0 = 0b00000000;
uint8_t cmd1 = 0b00000001;  //write config reg A

//   comm_arr[0] = cmd0;
//   comm_arr[1] = cmd1;
  
//   pec = pec15_calc(comm_len, comm_arr);

//   uint8_t data[6];

//   data[0] = 0b00000000;     //register 0 in group.    1 : GPIO high
//   data[1] = 0b11111111;
//   data[2] = 0b11111111;
//   data[3] = 0b11111111;
//   data[4] = 0b11111111;
//   data[5] = 0b11111111;

//   pec1 = pec >> 0;
//   pec0 = pec >> 8;

//   uint8_t data_pec0;
//   uint8_t data_pec1;
  
//   uint16_t data_pec = pec15_calc(6, data);
//   data_pec1 = data_pec >> 0;
//   data_pec0 = data_pec >> 8;

  
// wakeup_sleep(1);
// digitalWrite(CS, LOW);
// return_data = SPI.transfer(cmd0);
// return_data = SPI.transfer(cmd1);
// return_data = SPI.transfer(pec0);
// return_data = SPI.transfer(pec1);

// for(int i=0; i<6 ; i++){
//   SPI.transfer(data[i]);
// }
// return_data = SPI.transfer(data_pec0);
// return_data = SPI.transfer(data_pec1);


// digitalWrite(CS, HIGH);
//delay(10);
digitalWrite(CS, LOW);

cmd0 = 0b00000000;
cmd1 = 0b00000010;  //read config reg A

comm_arr[0] = cmd0;
comm_arr[1] = cmd1;
pec = pec15_calc(comm_len, comm_arr);
pec1 = pec >> 0;
pec0 = pec >> 8;

return_data = SPI.transfer(cmd0);
return_data = SPI.transfer(cmd1);
return_data = SPI.transfer(pec0);
return_data = SPI.transfer(pec1);

delay(2);
   for (int i = 0; i < 16; ++i) {
    //delay(.01);
    return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
     Serial.println(return_data, BIN); 
   }
  delay(1);
digitalWrite(CS, HIGH);

//while(1){}

}

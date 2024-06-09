#include <SPI.h>
#include <cmath>
#include <string>
#include "LTC681x.h"

#include "COMMANDS.h"
#include "HEADER.h"
#include "LUTS.h"

#include "Watchdog_t4.h"
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <algorithm>
#include <IntervalTimer.h>

#include <SD.h>
#include <TeensyThreads.h>

#define CRX3 23
#define CTX3 22
#define STBY 21     //CAN Transceiver Standby

uint16_t CHG_voltage = 588;
uint16_t CHG_current = 4;

#define FREQ_PIN 33
//pre-charge threshold
#define THRESHOLD 0.5

const int chipSelect = BUILTIN_SDCARD;

int start_time = 0;
bool CHG_EN = 0; //0: enable charging, 1: disable charging

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

IntervalTimer ADC;

WDT_T4<WDT1> wdt;     //watchdog 1 holds output pin low until power-on-reset. This is desired for a shutdown circuit


// // Shared variables
float currentSum = 0.0;
int currentCount1 = 0;
float averagedCurrent = 0.0;
Threads::Mutex currentMutex;

//state of charge
float soc = 0.00000000;
//total capacity( coulumbs): total capacity (Ah) * 60s/1hr
float _qt = 12.6 * 60;

// Threads::Mutex ADC;

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
float die_temps[num_boards];
float current;
float GPIO_open_wire[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];

float OV = 4.15;       //over-voltage limit (spelled with an "oh" not zero) (V)
float UV = 2.8;       //under-voltage limit       (V)

//balancing parameters
float balance_threshold = 3.4;    //will not balance cells below this threshold (V)
float max_differnce = 0.3;    //will not continue charging if max-min cell exceeds this threshold

float current_offset = 0;

//timinig threads
IntervalTimer curr_meas;
IntervalTimer volt_meas;
IntervalTimer write_SD;
IntervalTimer meas_temp;

void setup() {
  //open shutdown circuit
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  delay(20);
  //dump data from SD card to external program--Arduino IDE serial monitor will need to be off
  //*******
  Serial.begin(9600);
  delay(5000);
  dumpDataToSerial();
  delay(1000);
  Serial.println("startup");

  //SPI
  pinMode(CS,OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  //SD card calls;
  initializeSDCard();
  /*will set soc to previous known value. Must manually delete soc.txt file
  *from sd card at first start up (when battery is fully charged)- or add switch/push button that we could use to reset soc
  */
  manage_soc();
  start_time = millis();
  
  //CAN
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);
  can.begin();
  can.setBaudRate(250000);
  can.enableFIFO();

  //Watchdog
  WDT_timings_t config;
  config.trigger = 4; /* in seconds, 0->128 */    //time until watchdog callback function is triggered. 
  config.timeout = 5; /* in seconds, 0->128 */   //time until watchdog reset
  config.pin = 20;                                //pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
  //config.callback = myCallback;
  //wdt.begin(config);

  //current compensation
  measure_current();
  current_offset = current;

  //Bring up references on sense boards
  configure_sense();
}

void loop() {

  digitalWrite(20, LOW);
  measure_voltage();
  measure_temp();
  sense_status();
  //wait for ready to drive to start measurement threads
  float curr = measure_current();
  while(curr < THRESHOLD){
    curr = measure_current();
    delay(1000);
  }
  //begin measurments
  curr_meas.begin(measure_current, 1000);
  volt_meas.begin(measure_voltage,1000000);
  write_SD.begin(writeDataToSD,1300000);
  while(1){
    // measure_voltage();
    measure_temp();
    // measure_current();
    send_CAN();
    reset_watchdog();
    //RX_CAN();
    // writeDataToSD();
    delay(1000);  
  }

}

void print_min_max(){   //This function prints the min and max parameters
  float min_cell_voltage = cell_voltage[0][0];
  float max_cell_voltage = cell_voltage[0][0];
  float min_cell_temp = cell_temp[0][0];
  float max_cell_temp = cell_temp[0][0];
  float min_die_temp = die_temps[0];
  float max_die_temp = die_temps[0];
  min_max<num_boards,num_cells>(cell_voltage, &min_cell_voltage, &max_cell_voltage);
  min_max<num_boards,9>(cell_temp, &min_cell_temp, &max_cell_temp);
  min_max<1, num_boards>(&die_temps, &min_die_temp, &max_die_temp);       //This is how you pass a 1D array to the min_max function
  Serial.print("Max cell voltage: "); Serial.println(max_cell_voltage);
  Serial.print("Min cell_voltage: "); Serial.println(min_cell_voltage);
  Serial.print("Max cell_temp: "); Serial.println(max_cell_temp);
  Serial.print("Min cell_temp: "); Serial.println(min_cell_temp);
  Serial.print("Max die temp: "); Serial.println(max_die_temp);
  Serial.print("Min die temp: "); Serial.println(min_die_temp);
}



// // dump CSV data from last run to the serial port and delete the file
// //should be called in setup()
void dumpDataToSerial() {
  if (!SD.begin(chipSelect)) {
    // Serial.println("SD card initialization failed!");
    return;
  }

  // Open the CSV file for reading
  File dataFile = SD.open("data.csv");
  
  if (dataFile) {
    // Send file size as header
    unsigned long fileSize = dataFile.size();
    Serial.println(fileSize);

    // Send file content
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();

    // Delete the file after sending its contents
    SD.remove("data.csv");
  } else {
    Serial.println("Error opening data.csv for reading");
  }
}

// Function to extract the charge value from a line
float extractChargeValue(const String& line) {
  int startIndex = line.indexOf("Charge: ");
  if (startIndex != -1) {
    startIndex += 8; // Move past "Charge: "
    String chargeString = line.substring(startIndex);
    return chargeString.toFloat();
  }
  return 100.00; // Return full charge value if "Charge: " is not found
}

void manage_soc(){
  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to read SD card for soc.");
    return;
  }
  if(SD.exists("soc.txt")){
    File socFile = SD.open("soc.txt", FILE_READ);
    if (socFile) {
      //get last value of soc
      String lastLine;
      while (socFile.available()) {
        lastLine = socFile.readStringUntil('\n');
      }
      String soc_str = extractChargeValue(lastLine);
      Serial.print("Read Charge: " + soc_str);
      soc = soc_str.toFloat();
      socFile.close();
      Serial.print("State of charge: ");
      Serial.println(soc);
    }else {
      Serial.println("Error opening soc.txt for reading");
    }
  }else {
    File socFile = SD.open("soc.txt", FILE_WRITE);
    Serial.println("Initializing state of charge to 100%");
    soc = 100.0;
    socFile.println("Time: 0, Charge: 100.00000000");
    socFile.close();
    Serial.println("soc.txt initialized.");
  }
}

void update_soc(float curr_sample, String time){
  soc -= curr_sample / _qt;
  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to open SD card to update soc.");
    return;
  }
  File socFile = SD.open("soc.txt", FILE_WRITE);
  if (socFile) {
    //add current charge and time to end of file 
    socFile.print("Time: ");
    socFile.print(time);
    socFile.print(", Charge: ");
    socFile.println(soc);
    socFile.close();
    Serial.print("State of charge: ");
    Serial.println(soc);
  }else {
    Serial.println("Error opening soc.txt for writing");
  }
}

void writeDataToSD() {
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Voltage:\n");
    //write voltage data
    for (int i = 0; i < num_boards; i++) {
      for (int j = 0; j < num_cells; j++) {
        dataFile.print(cell_voltage[i][j], 4);
        if (i < num_boards - 1 || j < num_cells - 1) {
          dataFile.print(", ");
        }
      }
    }
    dataFile.print("\nTemperature:\n");

    // //write temperature data
    for (int i = 0; i < num_boards; i++) {
      for (int j = 0; j < 9; j++) {
        dataFile.print(cell_temp[i][j], 2);
        if (i < num_boards - 1 || j < num_cells - 1) {
          dataFile.print(", ");
        }
      }
    }
    //Current Measurement
    dataFile.print("\nCurrent: ");
    float current = avgCurrent();
    dataFile.print(current);

    float curr_time_ms = millis()-start_time;
    
    //format ms into HH:MM:SS
    unsigned long seconds = curr_time_ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    seconds = seconds % 60;
    minutes = minutes % 60;
    hours = hours % 24; // This will keep the time within 24 hours
    char time_string[9]; // HH:MM:SS is 8 characters + null terminator
    sprintf(time_string, "%02lu:%02lu:%02lu", hours, minutes, seconds);

    dataFile.print("\nTime:\n");

    //time stamp
    dataFile.print(time_string);
    dataFile.println();

    dataFile.close();
    Serial.print("Time: ");
    Serial.println(time_string);
    //write to soc.txt
    update_soc(current, time_string);
    Serial.println("Data written to SD card");
  } else {
    Serial.println("Error opening data.csv for writing");
  }
}

void initializeSDCard() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  SD.remove("data.csv");
  File dataFile = SD.open("data.csv", FILE_WRITE);
  
  if (dataFile) {
    // Write the header row
    for (int i = 1; i <= num_cells*num_boards; i++) {
      dataFile.print("Cell ");
      dataFile.print(i);
      if (i < num_cells*num_boards) {
        dataFile.print(", ");
      }
    }
    dataFile.println();
    dataFile.close();
    Serial.println("SD Card Init Successful.");
  } else {
    Serial.println("Error opening data.csv for writing");
  }
}

float avgCurrent() {
  currentMutex.lock();
  float average = (currentCount1 == 0) ? 0.0 : currentSum / currentCount1;
  currentSum = 0.0;
  currentCount1 = 0;
  currentMutex.unlock();

  // Serial.println(average);
  
  return average;
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

  //do not interuppt during SPI communication
  // noInterrupts();

  send_command(command);

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
    
    if(response_pec0 != pec0 || response_pec1 != pec1){
      read_register_group(command, response);
    }
  // interrupts();
  
  if(response_pec0 != pec0 || response_pec1 != pec1){
    read_register_group(command, response);
  }

      //Serial.println("Response");
      //Serial.println(response_pec0);
      //Serial.println(response_pec1);
      //Serial.println("Calc");
      //Serial.println(pec0);
      //Serial.println(pec1); 
  //Serial.println('\n');
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

void write_register_group(uint16_t command, uint8_t data[num_boards][6]){

  uint8_t return_data;
  uint8_t data_pec0;
  uint8_t data_pec1;
  uint16_t data_pec;

  send_command(command);

  for(int i = 0 ; i<num_boards; i++){
  data_pec = pec15_calc(6, data[i]);
  data_pec1 = data_pec >> 0;
  data_pec0 = data_pec >> 8;

  for(int j=0; j<6 ; j++){
    SPI.transfer(data[i][j]);
  }
    SPI.transfer(data_pec0);
    SPI.transfer(data_pec1);
  }
  digitalWrite(CS, HIGH);
}

void poll_ADC(uint16_t command){
  uint8_t return_data = 0;

  //do not interuppt during SPI communication
  // noInterrupts();
  send_command(command);

  int num_polls = 0;
  while (return_data == 0) {
    return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
    num_polls++;
  }
  // interrupts();
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
  volt_meas.end();
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

  volt_meas.begin(measure_voltage,1000000);

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

void measure_current(){
    digitalWrite(FREQ_PIN,HIGH);

    float R1 = 10000;   //bottom resistor in voltage divider (ohms)
    float R2 = 5100;    //top resistor in voltage divier (ohms)
    int ADC_in;
    float ADC_volt;
    float Hall_volt;
    int num_bits = 12;   //ADC resolution
    analogReadResolution(num_bits);
    ADC_in = analogRead(A11);       // 0-1023 integer
    Serial.println(ADC_in);
    ADC_volt = float(ADC_in)/(pow(2,num_bits)-1)*3.3;
    Serial.println(ADC_volt);
    Hall_volt = ADC_volt*(R1+R2)/R1;
    Serial.println(Hall_volt);
    current = ((Hall_volt-0.25)/(4.5)*(100)-50) - current_offset; 
    Serial.println("Current");       
    Serial.println(current);
    Serial.println();

    float current;

    ADC_in = analogRead(A11);       // 0-1023 integer
    //Serial.println(ADC_in);
    ADC_volt = float(ADC_in)/1023*3.3;
    //Serial.println(ADC_volt);
    Hall_volt = ADC_volt*(10000+5100)/10000;
    //Serial.println(Hall_volt);
    current = (Hall_volt-0.25)/(4.5)*(100)-50; 
    // Serial.println("Current");       
    // Serial.println(current);
    // Serial.println();

    digitalWrite(FREQ_PIN,LOW);
    //update current 
    currentMutex.lock();
    currentSum += current;
    currentCount1++;
    currentMutex.unlock();
}

void send_CAN(){
  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);
  delay(1);
  digitalWrite(CTX3, LOW);
  CAN_message_t CHGR_EN;
  CHGR_EN.id = 0x1806E5F4;  // Set the CAN message ID     //datasheet
  CHGR_EN.flags.extended = 1;
  CHGR_EN.len = 5;     // Set the data length

  CHGR_EN.buf[0] = (uint8_t)(CHG_voltage*10 >> 8);
  CHGR_EN.buf[1] = (uint8_t)(CHG_voltage*10);
  CHGR_EN.buf[2] = (uint8_t)(CHG_current*10 >> 8);
  CHGR_EN.buf[3] = (uint8_t)(CHG_current*10);
  CHGR_EN.buf[4] = (uint8_t)(CHG_EN);
  CHGR_EN.buf[5] = 0;
  CHGR_EN.buf[6] = 0;
  CHGR_EN.buf[7] = 0;

  if(can.write(CHGR_EN)){
    Serial.println("CAN message sent");
  }
  else{
    Serial.println("CAN message TX Failed");
  }
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
    msg.len = 16;
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], BIN);
      Serial.print(" ");
    }
    received = true;
    Serial.print('\n');
  }
}

void configure_sense(){     
  uint8_t data[6];        
  uint8_t data_arr[num_boards][6];  //contains identicle copies of data for each board
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV/(16*0.0001)-1;     //Comparison Voltage = (VUV + 1) • 16 • 100μV  (pg. 68 in datasheet)
  VOV = OV/(16*0.0001);       //Comparison Voltage = VOV • 16 • 100μV        (pg. 68 in datasheet)

  Serial.println(VUV, BIN);
  Serial.println(VOV, BIN);

  data[0] = 0b11111100;     //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
  data[1] = (uint8_t) VUV;
  data[2] = (uint8_t) (VOV & 0b11110000) | (VUV>>8 & 0b00001111);
  data[3] = (uint8_t) VOV>>4;
  data[4] = 0b00000000;
  data[5] = 0b00000000;

  // data[0] = 0b11111110;     //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
  // data[1] = (uint8_t) VUV;
  // data[2] = (uint8_t) (VOV & 0b11110000) | (VUV>>8 & 0b00001111);
  // data[3] = (uint8_t) VOV>>4;
  // data[4] = 0b11111111;
  // data[5] = 0b11111111;

  for(int i = 0; i< num_boards; i++){
    std::copy(data, data + 6, data_arr[i]);
  }
  write_register_group(WRCFGA, data_arr);
}

void balance(){
  bool discharge[num_boards][18] = {0};  //'1': needs dischaged, '0': does not need discharged
  float min = balance_threshold;
  float max = cell_voltage[0][0];
  
  ////mark cells to be discharged////
  min_max<num_boards,num_cells>(cell_voltage, &min, &max);
  for(int i = 0; i<num_boards; i++){
    for(int j =0; j<num_cells; j++){
        discharge[i][j] = cell_voltage[i][j] > balance_threshold;
    }
  }
  discharge_cells(discharge);
}

void sense_status(){
  uint8_t response[num_boards][6];
  poll_ADC(ADSTAT);
  read_register_group(RDSTATA, response);
  for(int i = 0; i<num_boards;i++){
    die_temps[i]= (response[i][2] | response[i][3]<<8) * (0.0001/.0076) - 276;
    Serial.println(die_temps[i]);
  }
}

// void sense_status(){
//   uint8_t response[num_boards][6];
//   read_register_group(RDSTATB , response);

//   undervoltage_flag[0] = response[2]>>0 & 0b1;
//   undervoltage_flag[1] = response[2]>>2 & 0b1;
//   undervoltage_flag[2] = response[2]>>4 & 0b1;
//   undervoltage_flag[3] = response[2]>>6 & 0b1;
//   undervoltage_flag[4] = response[3]>>0 & 0b1;
//   undervoltage_flag[5] = response[3]>>2 & 0b1;
//   undervoltage_flag[6] = 0;
//   undervoltage_flag[7] = 0;
//   undervoltage_flag[8] = 0;
//   undervoltage_flag[9] = 0;
//   undervoltage_flag[10] = 0;
//   undervoltage_flag[11] = 0;
//   undervoltage_flag[12] = 0;
//   undervoltage_flag[13] = 0;
//   undervoltage_flag[14] = 0;
//   undervoltage_flag[15] = 0;
//   Serial.println("voltage flags");
//   for(int i = 0; i<=5; i++){
//     Serial.println(undervoltage_flag[i]);
//   }
//   Serial.println("done");

// }

void flash_leds(){                        //Flashes each discharge resistor sequentially
  int time_on = 0;                        //Time each led is on in milliseconds
  bool discharge[num_boards][18] = {0};  //'1': needs dischaged, '0': does not need discharged
  for(int i = num_boards; i>=0; i--){
    if(i % 4 < 2){
      for(int j = 0; j<num_cells;j++){
        discharge[i][j] = true;
        discharge_cells(discharge);
        delay(time_on);
        discharge[i][j] = false;
        discharge_cells(discharge);
      }
    }
    else{
      for(int j = num_cells; j>=0;j--){
        discharge[i][j] = true;
        discharge_cells(discharge);
        delay(time_on);
        discharge[i][j] = false;
        discharge_cells(discharge);
      }
    }
  }
}

void discharge_cells(bool discharge[num_boards][18]){      //this function takes a 2D boolean array which is NOT dependent on num_cells.
  uint8_t data[6];
  uint8_t data_arr[num_boards][6];
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV/(16*0.0001)-1;     //Comparison Voltage = (VUV + 1) • 16 • 100μV  (pg. 68 in datasheet)
  VOV = OV/(16*0.0001);       //Comparison Voltage = VOV • 16 • 100μV        (pg. 68 in datasheet)
  ////configuration register group A////
  for(int i; i< num_boards; i++){
    data[0] = 0b11111100;     //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
    data[1] = (uint8_t) VUV;
    data[2] = (uint8_t) (VOV & 0b11110000) | (VUV>>8 & 0b00001111);
    data[3] = (uint8_t) VOV>>4;
    data[4] = (uint8_t) discharge[i][7]<<7 | discharge[i][6]<<6 | discharge[i][5]<<5 | discharge[i][4]<<4 | discharge[i][3]<<3 | discharge[i][2]<<2 | discharge[i][1]<<1 | discharge[i][0]<<0;
    data[5] = (uint8_t) discharge[i][11]<<3 | discharge[i][10]<<2 | discharge[i][9]<<1 | discharge[i][8]<<0;
    std::copy(data, data + 6, data_arr[i]);
  }

  write_register_group(WRCFGA, data_arr);
  ////configuration register group B/////
    for(int i; i< num_boards; i++){
    data[0] = (uint8_t) discharge[i][15]<<7 | discharge[i][14]<<6 | discharge[i][13]<<5 |discharge[i][12]<<4;
    data[1] = (uint8_t) discharge[i][17] | discharge[i][16];
    data[2] = (uint8_t) 0b00000000;
    data[3] = (uint8_t) 0b00000000;
    data[4] = (uint8_t) 0b00000000;
    data[5] = (uint8_t) 0b00000000;

    std::copy(data, data + 6, data_arr[i]);
  }
  write_register_group(WRCFGB, data_arr);
}

void myCallback() {               
  measure_voltage();
  reset_watchdog();
}

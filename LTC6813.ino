#include <SPI.h>
#include <cmath>
#include <string>

#include "COMMANDS.h"
#include "CONFIGURE.h"
#include "HEADER.h"
#include "LUTS.h"

#include "Watchdog_t4.h"
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <algorithm>
#include <IntervalTimer.h>

#include <SD.h>
#include <TeensyThreads.h>

//CAN pins
#define CRX3 23
#define CTX3 22
#define STBY 21     //CAN Transceiver Standby

//SD card pins
const int chipSelect = BUILTIN_SDCARD;

//SPI pins
#define CS 10   //chip select pin isoSPI
#define CS2 38   //3nd chip select pin isoSPI
#define CS1 0   //chip select for ADC

//flags
int wire_cut = 0;
bool memory_fault = 0;
bool comms_fault = 0;
bool watchdog_callback = 0;
bool watchdog_reset = 0;
bool debug = 0;

unsigned int start_time = 0;

float memory = 0;
bool CHG_EN = 0; //0: enable charging, 1: disable charging

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;      //    https://github.com/tonton81/FlexCAN_T4/tree/master

WDT_T4<WDT1> wdt;     //watchdog 1 holds output pin low until power-on-reset. This is desired for a shutdown circuit

// // Shared variables
float current;
float currentSum = 0.0;
int currentCount1 = 0;
float averagedCurrent = 0.0;
float current_offset = 0;
float gCurrent = 0;   //global variable to hold current current.

//state of charge
float soc = 0.0;

// data.csv enumeration
int data_file_num = 0;

//LTC6813 minimum supply voltage is 16V
float cell_voltage[num_boards][num_cells];     //most recent cell voltages
float cell_temp[num_boards][9];                //most recent cell temperatures. Contans raw voltage data for the duration of open wire checks
float die_temps[num_boards];                   //most recent sense board LTC6813 die temps

//sense board flags
float GPIO_open_wire[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];

void setup() {
  //open shutdown circuit
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);

  //startup timestamp
  start_time = millis();
  
  //dump data from SD card to external program-- Arduino IDE serial monitor will need to be off
  //*******
  Serial.begin(9600);
  //delay(5000);
  //dumpDataToSerial();
  //delay(1000);
  Serial.println("startup");
  Serial.print("Start Time: "); Serial.println(start_time);

  //SPI (isoSPI)
  pinMode(CS,OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  //SPI1 (ADC)
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  //SD card calls;
  //initializeSDCard();
  /*will set soc to previous known value. Must manually delete soc.txt file
  *from sd card at first start up (when battery is fully charged)- or add switch/push button that we could use to reset soc
  */

  //CAN
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);
  can.begin();
  can.setBaudRate(250000);
  can.setMaxMB(3);        //number of CAN message mailboxes
  digitalWrite(STBY, LOW);

  //    https://github.com/tonton81/FlexCAN_T4/blob/master/examples/mailbox_filtering_example_with_interrupts/mailbox_filtering_example_with_interrupts.ino
  // Mailboxes must be configured for all messages - both TX and RX
  can.setMB((FLEXCAN_MAILBOX)0,RX,STD);   //Standard mailbox for Inverter ID
  can.setMB((FLEXCAN_MAILBOX)1,RX,EXT);   //Extended id for charger
  can.setMB((FLEXCAN_MAILBOX)2,TX,EXT);   //BMS TX -> charger id


  can.setMBFilter(MB0, INV_TX_ID);  //Mailbox for Inverter CAN messages
  can.setMBFilter(MB1, CHG_TX_ID);  //Mailbox for Charger CAN Messages
  can.setMBFilter(MB2, 0x1806E5F4);  //Mailbox for Charger CAN Messages


  //Watchdog
  if(watchdog_timeout != 0){
    WDT_timings_t config;
    config.trigger = max(1,  watchdog_timeout - 1);   /* in seconds, 0->128 */    //time until watchdog callback function is triggered. 
    config.timeout = watchdog_timeout;               /* in seconds, 0->128 */   //time until watchdog reset
    config.pin = 20;                                //pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
    config.callback = myCallback;
    wdt.begin(config);              //This needs moved to the main loop
  }

  //current offset compensation
  //measure_current();
  //current_offset = current;

  //Bring up references on sense boards
  //configure_sense();

  //Bring up ADC
  //initialize_ADC();


 
  check_memory();
  
  //get_SOC();
  //check_memory();
  
  measure_voltage();
  measure_temp();
  reset_watchdog();
  // while(1){
  //   RX_CAN();
  //   //charger_enable(true);
  //   delay(20);
  // }


  if(mode == ""){
    CAN_message_t msg;
    while(1){
      msg = RX_CAN();
      String input = Serial.readStringUntil('\n');
      input.trim();
      if(msg.id == INV_TX_ID){    //Always check msg id
        mode = "standy";
        //can.setMBFilter(MB1, 0);  //Disable Charger Mailbox
        break;
      }
      else if(msg.id == CHG_TX_ID){
        mode = "charge";
        //can.setMBFilter(MB0, 0);  //Disable Inverter Mailbox
        break;
      }
      else if(input == "debug"){
        mode = "debug";
        break;
      }
  
      // if(mode == "charge" || "standby"){                 //example of flushing 2 CAN mailboxes. Dont need this code because inverter and charger don't use can bus at the same time. 
      //   // Stop mailbox interrupts (pauses reception)
      //   can.disableMBInterrupts();
      //   for(int i = 0; i<2; i++){     //flush both mailboxes
      //     RX_CAN();
      //   }
      //   can.enableMBInterrupts()     //enable reception
      //   break;
      // }

    }
  }

}

void loop() {
  if(mode == "charge"){
    Serial.println("Charge Mode Entered");      //if charger hardware fault exit charge mode

    CAN_message_t msg;
    String filename = "data" + String(data_file_num) + ".csv";
    File file = SD.open(filename.c_str(), FILE_WRITE);
    file.close();

    delay(6000);  //cause comm fault on charger. Power cycling the BMS without ensuring the charger fully powers down would otherwise can cause the BMS to enter the charge cycle agian

    //clear comm fault on charger
    while(1){
      charger_enable(true);                 //send turn-off message and clear comm fault on charger
      msg = RX_CAN();
      if(msg.id == CHG_TX_ID && msg.buf[4] == 0){
        break;   
      }
    }
    //00100 low ac power on charger flag
    delay(1000);    //delay so that another Charger CAN message is sent to the BMS

    //enter charge cycle
    while(1){
      measure_voltage();
      measure_temp();
      measure_current();
      if(reset_watchdog()){
        msg = RX_CAN();
        if(true || msg.id == CHG_TX_ID && msg.buf[4] == 0){
          charger_enable(false);   
        }
        else{
          digitalWrite(20, LOW);
          charger_enable(true);
          break;
        }
      }
      if(!memory_fault){
        SD_data_write(filename);
      }
      delay(2000);
    }

    //charger_falut
    while(1){
      Serial.println("Charger Fault");
      delay(1000);
    }
   
  }

  if(mode == "standby"){                  //waiting to drive
    Serial.println("Standby Mode Entered");
    measure_voltage();
    measure_temp();
    measure_current();
    //charger_enable();
    reset_watchdog();
    //RX_CAN();
    // writeDataToSD();
    delay(1000);  
  }

  if(mode == "drive"){
    
  }

  if(mode == "debug"){
    Serial.println("Debug Mode Entered");
    while(1){
      
    }
  }
 
  
}

void initialize_ADC(){
  //ADC sampling time constant (without external filter) = 50 ohms * 40 pF
  //CFR.B6 = 0 : uses external voltage reference
  //CFR.B9 = 0, FSR_ADC_A = 0 to VREF_A and FSR_ADC_B = 0 to VREF_B
  //CFR.B7 = 0 : single ended measurements
  //CFR.B11 = 0 and CFR.B10 = 1  Single-SDO Mode
  //CFR.B15:B11 = 1000 (write) or 0011 (read)
  uint8_t CFR_reg_MSB;
  uint8_t CFR_reg_LSB;
  uint8_t CFR_readback_MSB;                     
  uint8_t CFR_readback_LSB;                     
  CFR_reg_MSB = 0b10000100;  //CFR [B15:B8]
  CFR_reg_LSB = 0b01000000;  //CFR [B7:B0]

  //Send write CFR register command
  digitalWrite(CS1, LOW);
  SPI1.transfer(CFR_reg_MSB);
  SPI1.transfer(CFR_reg_LSB);
  for(int i = 0; i<6; i++){   //clock ADC
    SPI1.transfer(0b00000000);
  }
  digitalWrite(CS1, HIGH);
  delay(2);

  //Send read CFR register command while clocking the write config
  digitalWrite(CS1, LOW);
  SPI1.transfer(0b00110000);
  for(int i = 0; i<6; i++){
    SPI1.transfer(0b00000000);    //clock ADC
  }
  digitalWrite(CS1, HIGH);
  delay(2);

  //readback the CFR configuration
  digitalWrite(CS1, LOW);
  CFR_readback_MSB = SPI1.transfer(0b00000000);
  CFR_readback_LSB = SPI1.transfer(0b00000000);
  for(int i = 0; i<6; i++){
    SPI1.transfer(0b00000000);  //clock ADC
  }
  digitalWrite(CS1, HIGH);

  //the 4 MSBs of the CFR register (read/write command bits) are cleared in Frame F+2 which is not consistant with the datasheet
  if((uint8_t)(CFR_reg_MSB<<4) != (uint8_t)(CFR_readback_MSB<<4) || (uint8_t)(CFR_reg_LSB<<4) != (uint8_t)(CFR_readback_LSB<<4)){   //bit-shifts to mask the 4 MSBs
    Serial.println("ADC_initialization ERROR");
    Serial.println((CFR_reg_MSB<<4), BIN);
    Serial.println((CFR_reg_LSB<<4), BIN);
  }
}

void read_ADC(){
  uint16_t ADC_A;
  uint16_t ADC_B;
  float A_volt;
  float B_volt;
  uint16_t temp;
  digitalWrite(CS1, LOW);
  for(int i = 0; i<2; i++){
    temp = SPI1.transfer(0b00000000); //clock ADC
    Serial.print("temp: "); Serial.println(temp, BIN);

  }
  ADC_A = SPI1.transfer(0b00000000);
  ADC_A = ADC_A << 8;
  ADC_A = ADC_A | SPI1.transfer(0b00000000);
  Serial.print("ADC_A: "); Serial.println(ADC_A);
  ADC_B = SPI1.transfer(0b00000000);
  ADC_B = ADC_B << 8;
  ADC_B = ADC_B | SPI1.transfer(0b00000000);
  Serial.print("ADC_B: "); Serial.println(ADC_B);
  temp = SPI1.transfer(0b00000000);
  Serial.print("temp: "); Serial.println(temp, BIN);
  digitalWrite(CS1, HIGH);
  delay(2);
  A_volt = (float)(ADC_A)/65535*5;
  B_volt = (float)(ADC_B)/65535*5;
  Serial.print("A Voltage: "); Serial.println(A_volt);
  Serial.print("B Voltage: "); Serial.println(B_volt);
}


void print_min_max(float* max_voltage){   //This function prints the min and max parameters
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


// // we should be able to create a file per power cycle
// //should be called in setup()
void dumpDataToSerial() {
  while(1){
    String input = Serial.readStringUntil('\n');
    input.trim();
    if(input == "debug"){
      break;
    }
  }
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    memory_fault = 1;
    return;
  }

  // Open the CSV file for reading

  File dataFile = SD.open("data.csv");
  
  if (dataFile) {
    // Send file content
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
    Serial.println("serial dump done");

  } 
  else {
    Serial.println("Error opening data.csv for reading");
  }
}

void check_memory(){    //this should check all files
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    memory_fault = 1;
    return;
  }

  if(!SD.exists("state.txt")) {
    File file = SD.open("state.txt", FILE_WRITE);
    file.close();
  }

  File root = SD.open("/");
  File entry = root.openNextFile();
  uint64_t memory_usage = 0;
  int num_files = 0;
  while (entry) {
    num_files++;
    Serial.print(entry.name());
    Serial.print("\t");
    Serial.print(entry.size());
    Serial.println(" bytes");

    memory_usage += entry.size();
    entry.close();
    //SD.remove(entry.name());
    entry = root.openNextFile();
  }
  root.close();

  if(memory > 0.9*SD_card_size){
    Serial.println("SD card over 90% full");
    memory_fault = 1;
    return;
  }

  for(int i = 0; i < num_files + 3; i++){
    String filename = "data" + String(i) + ".csv";
    if(!SD.exists(filename.c_str())){
      data_file_num = i;
      break;
    }
  }

  // if(SD.exists("state.txt")){
  //   File dataFile = SD.open("data.csv");
  //   unsigned long fileSize = dataFile.size();   //file size in bytes
  //   Serial.println(fileSize);
  //   memory = fileSize / pow(10,6);
  //   Serial.println(memory);
  //   if(memory > 0.9*SD_card_size){
  //     Serial.println("SD card over 90% full");
  //     memory_fault = 1;
  //   }
  // }

  // else{
  //   Serial.println("data.csv not found");
  //   memory_fault = 1;
  // }
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

void get_SOC(){
  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to read SD card");
    memory_fault = 1;
    return;
  }
  if(SD.exists("state.txt")){
    File file = SD.open("state.txt", FILE_READ);
    if (file) {
      String line;
      while (file.available()) {
        line = file.readStringUntil('\n');
        Serial.println(line);
        int delim_index = line.indexOf(':');
        String name = line.substring(0, delim_index);
        String value = line.substring(delim_index + 1);
        map_text2var(name, value);
      }
      file.close();
    }
  }
  else {
    File file = SD.open("state.txt", FILE_WRITE);
    Serial.println("Initializing state of charge to 100%");
    file.close();
    Serial.println("state.txt initialized.");
  }
}

void map_text2var(String name, String value){     //map text name and value to a variable
  if(name == "SOC:"){
    soc = value.toFloat();
    Serial.println(soc);
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

void SD_data_write(String filename) {
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  // error checking goes here
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
    //update_soc(current, time_string);
    //Serial.println("Data written to SD card");
  } else {
    Serial.println("Error opening data.csv for writing");
    memory_fault = 1;
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
  //currentMutex.lock();
  float average = (currentCount1 == 0) ? 0.0 : currentSum / currentCount1;
  currentSum = 0.0;
  currentCount1 = 0;
  //currentMutex.unlock();

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

  wakeup_sleep(num_boards + 1);

  //delay(2);          
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
  
  // interrupts();
  
  if(response_pec0 != pec0 || response_pec1 != pec1){   //this recursion needs fixed
    wakeup_sleep(num_boards);
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

  for(int i = num_boards-1; i>=0; i--){
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

  //do not interupt during SPI communication
  // noInterrupts();
  send_command(command);

  int num_polls = 0;
  while (return_data == 0) {                                                      //This needs a timeout condition
    return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
    num_polls++;
  }
  // interrupts();

  // Serial.println("ADC Conversion Done!");
  // Serial.println(num_polls);
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

  if(debug){
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
  
}

float map_temp(float V){
   int i;
  int size = sizeof(NTC_LUT) / sizeof(NTC_LUT[0]);
  float R_bias = 10000;
  float V_ref = 3.00;

  if(V_ref == V){   //divide by zero case
    return -55;
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
  float temperature = float(i)/float(size)*(150+55)-55;
  //temperature = V;
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

  for(int i=0; i<num_boards; i++){
    for(int j=0; j<9; j++){
        cell_temp[i][j] = map_temp(cell_temp[i][j]);
    }
  }

  if(debug){
    Serial.println("Tempearatures:");
    for(int i=0; i<num_boards; i++){
      Serial.print("board: "); Serial.println(i+1);
      for(int j=0; j<9; j++){
        Serial.print(cell_temp[i][j]);
      }
      Serial.println('\n');
    }
  }
}

bool reset_watchdog(){

  for(int i = 0; i < num_boards; i++){
    for(int j = 0; j< num_cells; j++){
      if(cell_voltage[i][j] < OV && cell_voltage[i][j] > UV){
        continue;
      }
      else{
          Serial.println("invalid voltage");
          Serial.println(cell_voltage[i][j]);
          digitalWrite(20, LOW);
          return false;
      }
    }
  }

  for(int i = 0; i < num_boards; i++){
    for(int j = 0; j < 9; j++){
      if(cell_temp[i][j] > min_temp && cell_temp[i][j] < max_temp){   //board 8 temp sensor 8 open
        continue;
      }
      else{
          Serial.print("invalid temp Board:  "); Serial.print(i+1); Serial.print("Num: "); Serial.println(j+1);
          digitalWrite(20, LOW);
          return false;
      }
    }
  }
  digitalWrite(20, HIGH);
  wdt.feed();  
  return true;
}

void measure_current(){
  uint16_t ADC;
  float volt;
  digitalWrite(CS1, LOW);

  for(int i = 0; i<2; i++){
    SPI1.transfer(0b00000000); //clock ADC
  }

  ADC = SPI1.transfer(0b00000000);
  ADC = ADC << 8;
  ADC = ADC | SPI1.transfer(0b00000000);
  volt = (float)(ADC)/65535*5;
  current = (volt-0.25)/(4.5)*(100)-50;   //this needs checked
  
  if(current > 50){                        //so does this
  ADC = SPI1.transfer(0b00000000);
  ADC = ADC << 8;
  ADC = ADC | SPI1.transfer(0b00000000);
  volt = (float)(ADC)/65535*5;
  current = (volt-0.25)/(4.5)*(100)-50;   //this needs checked
  }

  digitalWrite(CS1, HIGH);
  gCurrent = current;

  //update current 
  currentSum += current;
  currentCount1++;
}


void charger_enable(bool enable){
  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);
  delay(1);
  CAN_message_t CHGR_EN;
  //CHGR_EN.id = 0x1806E5F4;  // Set the CAN message ID     //datasheet
  CHGR_EN.id = 0x1806E5F4;  // Set the CAN message ID     //datasheet
  //CHGR_EN.id = 0x18FF50E5;  //charger send can id?
  CHGR_EN.flags.extended = 1; 
  CHGR_EN.len = 8;     // Set the data length
  //7FF max CAN ID
  
  CHGR_EN.buf[0] = (uint8_t)(CHG_voltage*10);
  CHGR_EN.buf[1] = (uint8_t)(CHG_voltage*10 >> 8);
  CHGR_EN.buf[2] = (uint8_t)(CHG_current*10);
  CHGR_EN.buf[3] = (uint8_t)(CHG_current*10 >> 8);
  CHGR_EN.buf[4] = (uint8_t)(enable);
  CHGR_EN.buf[5] = 0;
  CHGR_EN.buf[6] = 0;
  CHGR_EN.buf[7] = 0;
  
  bool message_sent = can.write(CHGR_EN);

  digitalWrite(CTX3, LOW);

  if(message_sent && debug){
    Serial.println("CAN message sent");
  }
  else if(debug){
    Serial.println("CAN message TX Failed");
  }
}

void TX_CAN(){
  float min_cell_voltage = cell_voltage[0][0];
  float max_cell_voltage = cell_voltage[0][0];
  float min_cell_temp = cell_temp[0][0];
  float max_cell_temp = cell_temp[0][0];
  min_max<num_boards,num_cells>(cell_voltage, &min_cell_voltage, &max_cell_voltage);
  min_max<num_boards,9>(cell_temp, &min_cell_temp, &max_cell_temp);

  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);
  delay(1);
  digitalWrite(CTX3, LOW);
  CAN_message_t BMS_data;
  BMS_data.id = BMS_ID;
  BMS_data.flags.extended = 0; 
  BMS_data.len = 8;     // Set the data length

  BMS_data.buf[0] = float_2_uint8_t(soc, 0, 100);               //SOC
  BMS_data.buf[1] = float_2_uint8_t(12, 0, 12);                 //current
  BMS_data.buf[2] = float_2_uint8_t(max_cell_voltage, 0, 5);    //max cell
  BMS_data.buf[3] = float_2_uint8_t(min_cell_temp, 0, 150);
  BMS_data.buf[4] = 0;
  BMS_data.buf[5] = 0;
  BMS_data.buf[6] = 0;
  BMS_data.buf[7] = 0;

  if(can.write(BMS_data)){
    Serial.println("CAN message sent");
  }
  else{
    Serial.println("CAN message TX Failed");
  }

}

uint8_t float_2_uint8_t(float float_val, float min, float max){   //float to uint8_t, clips values under/over min or max
  if(max == min) {return(0);}   //divide by zero risk
  if(float_val >= max){         //overflow risk
    return max;
  }
  if(float_val <= min){         //overflow risk
    return min;
  }
  uint8_t scaled = (uint8_t)(((float_val - min) / (max - min)) * 255.0);    //float decoded = ((float)scaled / 255.0) * (max - min) + min;
  return(scaled);
}

CAN_message_t RX_CAN(){     //grabs the first message in the FIFO. 
  static int curr_time = 0;

  //left bit in charger flag is highest bit (bit 4)
  CAN_message_t msg = {};
  digitalWrite(STBY, LOW);
  bool recieved = false;
  can.read(msg);
  //can.readMB(msg);
  if(msg.id != 0){
    Serial.print("ID: ");
    Serial.print(msg.id, HEX);
    Serial.println(" Data: ");
    //msg.len = 8;
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], BIN);
      Serial.print(" ");
    }
    Serial.print('\n');
    Serial.println(millis() - curr_time);
    curr_time = millis();
  }
  return msg;   //always check the ID of the returned message. No messages in buffer returns 0 ID with 8 byte of zero data
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

void balance(bool keep_going){
  bool discharge[num_boards][18] = {0};  //'1': needs dischaged, '0': does not need discharged
  float min = cell_voltage[0][0];
  float max = cell_voltage[0][0];

  sense_status();
  ////mark cells to be discharged////
  min_max<num_boards,num_cells>(cell_voltage, &min, &max);
  Serial.print("min cell voltage: "); Serial.println(min);
  Serial.print("max cell voltage: "); Serial.println(max);
  Serial.println("Cells to be discharged");
  if(keep_going){
    for(int i = 0; i<num_boards; i++){
      for(int j =0; j<num_cells; j++){
          discharge[i][j] = cell_voltage[i][j] > min && cell_voltage[i][j] > balance_threshold && die_temps[i] < 60.0f;
          if(discharge[i][j]){
          Serial.print("Board: "); Serial.print(i+1); Serial.print("  Cell: "); Serial.print(j+1); Serial.print(" Volt: "); Serial.println(cell_voltage[i][j]);
          Serial.print("die temp: "); Serial.println(die_temps[i]);
          }
      }
    }
  }
  discharge_cells(discharge);
  if(balance_threshold < min){
    balance_threshold = min;
  }
}

void sense_status(){              //really should be the measure die temp function
  uint8_t response[num_boards][6];
  poll_ADC(ADSTAT);
  read_register_group(RDSTATA, response);
  Serial.println("die_temps");
  for(int i = 0; i<num_boards;i++){
    die_temps[i]= (response[i][2] | response[i][3]<<8) * (0.0001/.0076) - 276;
    Serial.println(die_temps[i]);
  }
  Serial.println();
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
  for(int i = 0; i< num_boards; i++){
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
    for(int i = 0; i< num_boards; i++){
    data[0] = (uint8_t) discharge[i][15]<<7 | discharge[i][14]<<6 | discharge[i][13]<<5 |discharge[i][12]<<4 | 0b1111;
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
  //Serial.println("Callback Called");   
  measure_voltage();
  reset_watchdog();
}


//Analog Devices provided Functions

void wakeup_sleep(uint8_t total_ic) //Number of ICs in the system. This function needs some work
{
	for (int i =0; i<total_ic; i++)
	{
	   digitalWrite(CS, LOW);
	   delay(3); // Guarantees the LTC681x will be in standby
	   digitalWrite(CS, HIGH);
	   delay(1);
	}
}

uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
	uint16_t remainder,addr;
	remainder = 16;//initialize the PEC
	
	for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
	{
		addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
		#ifdef MBED
			remainder = (remainder<<8)^crc15Table[addr];
		#else
			remainder = (remainder<<8)^pgm_read_word_near(crc15Table+addr);
		#endif
	}
	
	return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

void wakeup_idle(uint8_t total_ic){ //Number of ICs in the system
	for (int i =0; i<total_ic + 2; i++){    //+2 for the LTC6820s
  digitalWrite(CS, LOW);
  SPI.transfer(0b11111111);     //Guarantees the isoSPI will be in ready mode
  digitalWrite(CS, HIGH);
	}
}


// void LTC681x_adow(uint8_t MD, //ADC Mode
// 				  uint8_t PUP,//Pull up/Pull down current
// 				  uint8_t CH, //Channels
// 				  uint8_t DCP//Discharge Permit
// 				 )
// {
// 	uint8_t cmd[2];
// 	uint8_t md_bits;
	
// 	md_bits = (MD & 0x02) >> 1;
// 	cmd[0] = md_bits + 0x02;
// 	md_bits = (MD & 0x01) << 7;
// 	cmd[1] =  md_bits + 0x28 + (PUP<<6) + CH+(DCP<<4);
	
// 	cmd_68(cmd);
// }

// /* Start GPIOs open wire ADC conversion */
// void LTC681x_axow(uint8_t MD, //ADC Mode
// 				  uint8_t PUP //Pull up/Pull down current
// 				 )
// {
// 	uint8_t cmd[2];
// 	uint8_t md_bits;
	
// 	md_bits = (MD & 0x02) >> 1;
// 	cmd[0] = md_bits + 0x04;
// 	md_bits = (MD & 0x01) << 7;
// 	cmd[1] =  md_bits + 0x10+ (PUP<<6) ;//+ CH;
	
// 	cmd_68(cmd);
// }

// /* Runs the data sheet algorithm for open wire for single cell detection */
// void LTC681x_run_openwire_single(uint8_t total_ic, // Number of ICs in the daisy chain
// 								cell_asic ic[] // A two dimensional array that will store the data
// 								)
// {				  
// 	uint16_t OPENWIRE_THRESHOLD = 4000;
// 	const uint8_t  N_CHANNELS = ic[0].ic_reg.cell_channels;
	
// 	uint16_t pullUp[total_ic][N_CHANNELS];
// 	uint16_t pullDwn[total_ic][N_CHANNELS];
// 	int16_t openWire_delta[total_ic][N_CHANNELS];
	
// 	int8_t error;
// 	int8_t i;
// 	uint32_t conv_time=0;

// 	wakeup_sleep(total_ic);
// 	LTC681x_clrcell();
	
// 	// Pull Ups
// 	for (i = 0; i < 3; i++)
// 	{ 
// 	  wakeup_idle(total_ic);
// 	  LTC681x_adow(MD_26HZ_2KHZ,PULL_UP_CURRENT,CELL_CH_ALL,DCP_DISABLED);
// 	  conv_time =LTC681x_pollAdc();
// 	} 
	
// 	wakeup_idle(total_ic);
// 	error=LTC681x_rdcv(0, total_ic,ic);
	
// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 	    for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 		  pullUp[cic][cell] = ic[cic].cells.c_codes[cell];
// 		}	
// 	}

// 	// Pull Downs
// 	for (i = 0; i < 3; i++)
// 	{  
// 	  wakeup_idle(total_ic);
// 	  LTC681x_adow(MD_26HZ_2KHZ,PULL_DOWN_CURRENT,CELL_CH_ALL,DCP_DISABLED);
// 	  conv_time =LTC681x_pollAdc();
// 	}
	
// 	wakeup_idle(total_ic);
// 	error=LTC681x_rdcv(0, total_ic,ic); 
	
// 	for (int cic=0; cic<total_ic; cic++)
// 	{		  
// 	    for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 		   pullDwn[cic][cell] = ic[cic].cells.c_codes[cell];
// 		}
// 	}

// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 	  ic[cic].system_open_wire = 0xFFFF;
	  
// 		for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 			if (pullDwn[cic][cell] < pullUp[cic][cell])                   
// 			{
// 				openWire_delta[cic][cell] = (pullUp[cic][cell] - pullDwn[cic][cell]);
// 			}
// 			else
// 			{
// 				openWire_delta[cic][cell] = 0;                                             
// 			}  
				
// 			if (openWire_delta[cic][cell]>OPENWIRE_THRESHOLD)
// 			{
// 				ic[cic].system_open_wire = cell+1;
// 			}
// 		}
		
// 		if (pullUp[cic][0] == 0)
// 		{
// 		  ic[cic].system_open_wire = 0;
// 		}
		
// 		if (pullUp[cic][(N_CHANNELS-1)] == 0)//checking the Pull up value of the top measured channel
// 		{
// 		  ic[cic].system_open_wire = N_CHANNELS;
// 		}	
// 	}
// }

// /* Runs the data sheet algorithm for open wire for multiple cell and two consecutive cells detection */
//  void LTC681x_run_openwire_multi(uint8_t total_ic, // Number of ICs in the daisy chain
// 						  cell_asic ic[] // A two dimensional array that will store the data
// 						  )
// {              
// 	uint16_t OPENWIRE_THRESHOLD = 4000;
// 	const uint8_t  N_CHANNELS = ic[0].ic_reg.cell_channels;

// 	uint16_t pullUp[total_ic][N_CHANNELS];
// 	uint16_t pullDwn[total_ic][N_CHANNELS];
// 	uint16_t openWire_delta[total_ic][N_CHANNELS];

// 	int8_t error;
// 	int8_t opencells[N_CHANNELS];
// 	int8_t n=0;
// 	int8_t i,j,k;
// 	uint32_t conv_time=0;

// 	wakeup_sleep(total_ic);
// 	LTC681x_clrcell();

// 	// Pull Ups
// 	for (i = 0; i < 5; i++)
// 	{ 
// 		wakeup_idle(total_ic);
// 		LTC681x_adow(MD_26HZ_2KHZ,PULL_UP_CURRENT,CELL_CH_ALL,DCP_DISABLED);
// 		conv_time =LTC681x_pollAdc();
// 	} 

// 	wakeup_idle(total_ic);
// 	error = LTC681x_rdcv(0, total_ic,ic);

// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 	    for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 		  pullUp[cic][cell] = ic[cic].cells.c_codes[cell];
// 		}
// 	}

// 	// Pull Downs
// 	for (i = 0; i < 5; i++)
// 	{  
// 	  wakeup_idle(total_ic);
// 	  LTC681x_adow(MD_26HZ_2KHZ,PULL_DOWN_CURRENT,CELL_CH_ALL,DCP_DISABLED);
// 	  conv_time =   LTC681x_pollAdc();
// 	}

// 	wakeup_idle(total_ic);
// 	error = LTC681x_rdcv(0, total_ic,ic); 

// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 		for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 		   pullDwn[cic][cell] = ic[cic].cells.c_codes[cell];
// 		}
// 	}

// 	for (int cic=0; cic<total_ic; cic++)
// 	{			  
// 		for (int cell=0; cell<N_CHANNELS; cell++)
// 		{
// 			if (pullDwn[cic][cell] < pullUp[cic][cell])                   
// 				{
// 					openWire_delta[cic][cell] = (pullUp[cic][cell] - pullDwn[cic][cell]);
// 				}
// 				else
// 				{
// 					openWire_delta[cic][cell] = 0;                                             
// 				}
// 		}  
// 	}

// 	for (int cic=0; cic<total_ic; cic++)
// 	{ 
// 		n=0;
						
// 		Serial.print("IC:");
// 		Serial.println(cic+1, DEC);
		
// 		for (int cell=0; cell<N_CHANNELS; cell++)
// 		{  
		 
// 		  if (openWire_delta[cic][cell]>OPENWIRE_THRESHOLD)
// 			{
// 				opencells[n] = cell+1;
// 				n++;
// 				for (int j = cell; j < N_CHANNELS-3 ; j++)                       
// 				{
// 					if (pullUp[cic][j + 2] == 0)
// 					{
// 					opencells[n] = j+2;
// 					n++;
// 					}
// 				}
// 				if((cell==N_CHANNELS-4) && (pullDwn[cic][N_CHANNELS-3] == 0))
// 				{
// 					  opencells[n] = N_CHANNELS-2;
// 					  n++;
// 				}
// 			}
// 		}
// 		if (pullDwn[cic][0] == 0)
// 		{
// 		  opencells[n] = 0;
// 		  Serial.println("Cell 0 is Open and multiple open wires maybe possible.");
// 		  n++;
// 		}
					
// 		if (pullDwn[cic][N_CHANNELS-1] == 0)
// 		{
// 		  opencells[n] = N_CHANNELS;
// 		  n++;
// 		}
					
// 		if (pullDwn[cic][N_CHANNELS-2] == 0)
// 		{  
// 		  opencells[n] = N_CHANNELS-1;
// 		  n++;
// 		}
		
// 	//Removing repetitive elements
// 		for(i=0;i<n;i++)
// 		{
// 			for(j=i+1;j<n;)
// 			{
// 				if(opencells[i]==opencells[j])
// 				{
// 					for(k=j;k<n;k++)
// 						opencells[k]=opencells[k+1];
						
// 					n--;
// 				}
// 				else
// 					j++;
// 			}
// 		}
					
// 	// Sorting open cell array
// 		for(int i=0; i<n; i++)
// 		{
// 			for(int j=0; j<n-1; j++)
// 			{
// 				if( opencells[j] > opencells[j+1] )
// 				{
// 					k = opencells[j];
// 					opencells[j] = opencells[j+1];
// 					opencells[j+1] = k;
// 				}
// 			}
// 		}
					
// 	//Checking the value of n				
// 		Serial.println("Number of Open wires:");
// 		Serial.println(n);
		   
// 	//Printing open cell array
// 		Serial.println("OPEN CELLS:");
// 		if(n==0)
// 		{
// 			Serial.println("No Open wires");
// 		}
// 		else
// 		{				
// 			for(i=0;i<n;i++)
// 			{
// 					Serial.println(opencells[i]);	
// 			}
// 		}
// 	}
// 	Serial.println("\n");
// }

// /* Runs open wire for GPIOs */
// void LTC681x_run_gpio_openwire(uint8_t total_ic, // Number of ICs in the daisy chain
// 								cell_asic ic[] // A two dimensional array that will store the data
// 								)
//  {				  
// 	uint16_t OPENWIRE_THRESHOLD = 150;
// 	const uint8_t  N_CHANNELS = ic[0].ic_reg.aux_channels +1;
	
// 	uint16_t aux_val[total_ic][N_CHANNELS];
// 	uint16_t pDwn[total_ic][N_CHANNELS];
// 	uint16_t ow_delta[total_ic][N_CHANNELS];
	
// 	int8_t error;
// 	int8_t i;
// 	uint32_t conv_time=0;

// 	wakeup_sleep(total_ic); 
// 	LTC681x_clraux();
	 
// 	for (i = 0; i < 3; i++)
// 	{ 
// 	   wakeup_idle(total_ic);
// 	   LTC681x_adax(MD_7KHZ_3KHZ, AUX_CH_ALL);
// 	   conv_time= LTC681x_pollAdc();
// 	}
	
// 	wakeup_idle(total_ic);
// 	error = LTC681x_rdaux(0, total_ic,ic);
	
// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 	    for (int channel=0; channel<N_CHANNELS; channel++)
// 		{
// 			aux_val[cic][channel]=ic[cic].aux.a_codes[channel];
// 		}
// 	}	
// 	LTC681x_clraux();
	
// 	// pull downs
// 	for (i = 0; i < 3; i++)
// 	{ 
// 	   wakeup_idle(total_ic);
// 	   LTC681x_axow(MD_7KHZ_3KHZ,PULL_DOWN_CURRENT);
// 	   conv_time =LTC681x_pollAdc();
// 	} 
	
// 	wakeup_idle(total_ic);
// 	error = LTC681x_rdaux(0, total_ic,ic);
	
// 	for (int cic=0; cic<total_ic; cic++)
// 	{
// 	   for (int channel=0; channel<N_CHANNELS; channel++)
// 		{
// 			pDwn[cic][channel]=ic[cic].aux.a_codes[channel] ;
// 		}
// 	}
	
// 	for (int cic=0; cic<total_ic; cic++)
// 	{  
// 		ic[cic].system_open_wire = 0xFFFF;
		
// 		for (int channel=0; channel<N_CHANNELS; channel++)
// 		{
// 			if (pDwn[cic][channel] > aux_val[cic][channel])                   
// 			{
// 				ow_delta[cic][channel] = (pDwn[cic][channel] - aux_val[cic][channel]);
// 			}
// 			else
// 			{
// 				ow_delta[cic][channel] = 0;                                             
// 			} 
			
// 			if(channel<5)
// 			{
// 				if (ow_delta[cic][channel] > OPENWIRE_THRESHOLD) 
// 				{
// 					ic[cic].system_open_wire= channel+1;
					
// 				}  
// 			}
// 			else if(channel>5)
// 			{
// 				if (ow_delta[cic][channel] > OPENWIRE_THRESHOLD) 
// 				{
// 					ic[cic].system_open_wire= channel;
					
// 				}  
// 			}	
// 		}
// 	}	  
// }


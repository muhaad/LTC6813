void send_command(uint16_t command){
    uint8_t comm_arr[2];
    uint16_t pec;
    uint8_t pec0;
    uint8_t pec1;
    uint8_t cmd0;
    uint8_t cmd1;

    cmd0 = command >> 8;
    cmd1 = command >> 0;

    wakeup_sleep(NUM_BOARDS);

    delay(2);                 //small delay is needed after wake to bring up power supply
    digitalWrite(CS, LOW);

    comm_arr[0] = cmd0;
    comm_arr[1] = cmd1;

    pec = pec15_calc(2, comm_arr);
    pec1 = pec >> 0;
    pec0 = pec >> 8;

    // SPI.transfer(cmd0);
    // SPI.transfer(cmd1);
    // SPI.transfer(pec0);
    // SPI.transfer(pec1);

    uint8_t transfer_arr[4] = {cmd0,cmd1,pec0,pec1};
    /*testing if this version of SPI.transfer will work
    **it is of the form: void SPIClass::transfer(const void * buf, void * retbuf, uint32_t count)
    **requires this version https://github.com/PaulStoffregen/SPI.git
    */
    SPI.transfer(transfer_arr,nullptr,sizeof(transfer_arr));
}

bool check_pec(uint16_t actual, uint16_t calculated){
    return (actual == calculated);
}

void read_register_group2(uint16_t command, uint8_t response[NUM_BOARDS][6]){      //register group is always 6 bytes 

    uint16_t pec;
    uint8_t response_pec0;
    uint8_t response_pec1;
    char msg[27];

    uint8_t recv_buff[6];
    uint8_t dummy = 0xFF;

    send_command(command);

    for (int i = 0; i < NUM_BOARDS; i++){
        // for (int j = 0; j < 6; j++) {
        //     response[i][j] = SPI.transfer(0b11111111); // Send dummy byte to receive data
        // }
        //Test for SPI.transfer -- see send command function
        SPI.transfer(nullptr,recv_buff,sizeof(recv_buff));
        //copy data into response array
        memcpy(response[i], recv_buff, sizeof(recv_buff));
        /*
        ** could also extract the pecs from the SPI.transfer call along
        ** with the data if this syntax works
        */
        response_pec0 = SPI.transfer(0xFF);
        response_pec1 = SPI.transfer(0xFF);
        pec = pec15_calc(6, recv_buff);
        // pec = pec15_calc(6, response[i]);

        /*see if returned and calculated pec are same
        **may have appened pec1 and pec0 in wrong order
        */
        if(!check_pec(pec,(uint16_t)(response_pec1 << 8) | response_pec0)){
            snprintf(msg,sizeof(msg),"Board %d data corruption.\n",i);
            Serial.print(msg);
        }
    }
    digitalWrite(CS, HIGH);
}

int main(){
  return 0;
}

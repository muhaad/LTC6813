
// dump CSV data from last run to the serial port and delete the file
//should be called in setup()
void dumpDataToSerial() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
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

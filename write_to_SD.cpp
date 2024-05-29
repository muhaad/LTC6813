void initializeSDCard() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  
  File dataFile = SD.open("data.csv", FILE_WRITE);
  
  if (dataFile) {
    // Write the header row
    for (int i = 1; i <= numCells; i++) {
      dataFile.print("Cell ");
      dataFile.print(i);
      if (i < numCells) {
        dataFile.print(", ");
      }
    }
    dataFile.println();
    dataFile.close();
  } else {
    Serial.println("Error opening data.csv for writing");
  }
}

void writeVoltageDataToSD() {
  File dataFile = SD.open("data.csv", FILE_WRITE);
  
  if (dataFile) {
    for (int i = 0; i < numBoards; i++) {
      for (int j = 0; j < numCells; j++) {
        // Print voltage with 4 decimal places
        dataFile.print(cell_voltage[i][j], 4);
        if (j < numCells - 1) {
          dataFile.print(", ");
        }
      }
      dataFile.println();
    }
    dataFile.close();
  } else {
    Serial.println("Error opening data.csv for writing");
  }
}

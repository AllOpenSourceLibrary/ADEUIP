#include "UIP_ADE7912.h"



UIP_ADE7912::UIP_ADE7912(int slaveSelectPin,SPIClass &spiRef, Stream &serialPort) : slaveSelectPin(slaveSelectPin), spi(spiRef), serial(serialPort), spiSettings(8192000, MSBFIRST, SPI_MODE3) {
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  // SPI settings for the ADE7913, use min. SCLK speed of 250kHz for debugging:
  //SPISettings spiSettings(4096000, MSBFIRST, SPI_MODE3);
 

}

void UIP_ADE7912::startup(){


spi.beginTransaction(spiSettings);

  // Read STATUS0 register, until Bit 0 (RESET_ON) is cleared:
  STATUS0[0] = 0b11111111;
  int nTry = 0;
  do {
    readMultBytesADE7913(STATUS0_READ, STATUS0, 1);
    nTry++;
  } while (bitRead(STATUS0[0], 0) && nTry < nMaxWriteTry);

  // Check if bit succusfully cleared
  if (bitRead(STATUS0[0], 0)) {
    serial.print("ERROR: RESET_ON bit NOT cleared, nTry: "); Serial.println(nTry);
    serial.print("STATUS0[0]: "); Serial.println(STATUS0[0], BIN);
    while (true) {};  // LOOP forever  on failure
  } else {
    serial.print("RESET_ON bit cleared, nTry: "); Serial.println(nTry);
    serial.print("STATUS0[0]: "); Serial.println(STATUS0[0], BIN);
  }

  // Unlock CONFIG registers, NB this cannot be read back
  writeADE7913(LOCK_KEY_WRITE, UNLOCK_BYTE);
  serial.println("Registers unlocked!");

  // Initialize CONFIG register with bit 0 (CLKOUT_EN) cleared (to 0)
  // as CLKOUT unecessary (we provide it from ardiuno)
  // also SET TEMP_EN (bit 3) so temperature can be measured (we're not using V2P)
  // SET ADC_FREQ (bit 5:4) to 11 (1kHz for debugging), otherwise 00 (8kHx) for running
  boolean writeSuccess = writeADE7913_check(CONFIG_WRITE, 0b00001001, CONFIG_READ);
  delay(100);
  readMultBytesADE7913(CONFIG_READ, CONFIG, 1);
  if (writeSuccess) {
    serial.println("CONFIG write success!");
    serial.print("CONFIG[0]: "); serial.println(CONFIG[0], BIN);
  } else {
    serial.println("ERROR: CONFIG Write Failed");
    serial.print("CONFIG[0]: "); serial.println(CONFIG[0], BIN);
    while (true) {};  // LOOP forever  on failure
  }

  // Read temperature offset register:
  readMultBytesADE7913(TEMPOS_READ, TEMPOS, 1);
  serial.print("TEMPOS: "); serial.println((int8_t) TEMPOS[0], DEC);

  // Set the EMI_CTRL register; and check written correctly:
  writeSuccess = writeADE7913_check(EMI_CTRL_WRITE, 0b01010101, EMI_CTRL_READ);
  delay(100);
  readMultBytesADE7913(EMI_CTRL_READ, EMI_CTRL, 1);
  if (writeSuccess) {
    serial.println("EMI_CTRL write success!");
    serial.print("EMI_CTRL[0]: "); serial.println(EMI_CTRL[0], BIN);
  } else {
    serial.println("ERROR: EMI_CTRL Write Failed");
    serial.print("EMI_CTRL[0]: "); serial.println(EMI_CTRL[0], BIN);
    while (true) {};  // LOOP forever  on failure
  }

  // Execute a SYNC_SNAP = 0x01 write broadcast, NB will be cleared to 0x00 after 1 CLK cycle
  writeADE7913(SYNC_SNAP_WRITE, 0b00000001);
  serial.println("SYNC_SNAP Register Set!");

  // Execute a LOCK_KEY = 0xCA (to lock CONFIG registers). This cannot be read back
  writeADE7913(LOCK_KEY_WRITE, LOCK_BYTE);
  serial.println("Registers locked!");

  spi.endTransaction();


  serial.println();
  serial.println(" --- SETUP COMPLETE ---");
  serial.println();
}




void UIP_ADE7912::writeADE7913(byte writeTo, byte writeMsg) { // Write a register to ADE7913, assume SPI.beginTransaction already called
  digitalWrite(slaveSelectPin, LOW);
  spi.transfer(writeTo);
  spi.transfer(writeMsg);
  digitalWrite(slaveSelectPin, HIGH);
}

boolean UIP_ADE7912::writeADE7913_check(byte writeTo, byte writeMsg, byte readFrom) {
  boolean success = false;
  int nTry = 0;
  do {
    digitalWrite(slaveSelectPin, LOW);
    spi.transfer(writeTo);
    spi.transfer(writeMsg);
    digitalWrite(slaveSelectPin, HIGH);
    delay(1);
    // Read-back register to confirm write success
    byte readBack[1];
    readMultBytesADE7913(readFrom, readBack, 1);
    success = (readBack[0] == writeMsg);
    nTry++;
  } while ((!success) && nTry < nMaxWriteTry);

  return success;
}


void UIP_ADE7912::readMultBytesADE7913(byte readFrom, volatile byte readTo[], int nBytes) {
  int idx = nBytes - 1; 
  digitalWrite(slaveSelectPin, LOW);
  spi.transfer(readFrom);
  while (idx >= 0) {
    readTo[idx] = SPI.transfer(DUMMY_MSG);
    idx--;
  }
  digitalWrite(slaveSelectPin, HIGH);
}


// Extend sign byte of 3-byte word to create signed 4-byte word
void UIP_ADE7912::extendSignBit(volatile threeByteWord *wordIn) {
  boolean signBit = bitRead(wordIn->bytes[2], 7);
  if (signBit == 0) {
    wordIn->bytes[3] = 0b00000000;
  } else {
    wordIn->bytes[3] = 0b11111111;
  }
}

uint16_t UIP_ADE7912::calculate_crc16_ccitt_for_three_words(volatile threeByteWord *word1, volatile threeByteWord *word2, volatile threeByteWord *word3) {
    uint8_t data[9] = {word1->bytes[0], word1->bytes[1], word1->bytes[2],   word2->bytes[0], word2->bytes[1], word2->bytes[2],   word3->bytes[0], word3->bytes[1], word3->bytes[2]    };
    return calculate_crc16_ccitt(data, sizeof(data));
}



uint16_t UIP_ADE7912::calculate_crc16_ccitt(const uint8_t* data, size_t length) {
    const uint16_t crc16_ccitt_polynomial = 0x1021;  
    uint16_t crc = 0xFFFF; 
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8; 
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ crc16_ccitt_polynomial; 
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool UIP_ADE7912::readdata() {

  SPI.beginTransaction(spiSettings);
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(IWV_READ);
  IWV.bytes[2] = SPI.transfer(DUMMY_MSG);
  IWV.bytes[1] = SPI.transfer(DUMMY_MSG);
  IWV.bytes[0] = SPI.transfer(DUMMY_MSG);
  V1WV.bytes[2] = SPI.transfer(DUMMY_MSG);
  V1WV.bytes[1] = SPI.transfer(DUMMY_MSG);
  V1WV.bytes[0] = SPI.transfer(DUMMY_MSG);
  V2WV.bytes[2] = SPI.transfer(DUMMY_MSG);
  V2WV.bytes[1] = SPI.transfer(DUMMY_MSG);
  V2WV.bytes[0] = SPI.transfer(DUMMY_MSG);
  ADC_CRC[1] = SPI.transfer(DUMMY_MSG);
  ADC_CRC[0] = SPI.transfer(DUMMY_MSG);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.endTransaction();
  extendSignBit(&IWV);
  extendSignBit(&V1WV);
  extendSignBit(&V2WV);

  uint16_t minucrc = calculate_crc16_ccitt_for_three_words(&IWV,&V1WV,&V2WV);
  uint16_t crc= ((uint16_t)ADC_CRC[1] << 8) | ADC_CRC[0];  
  if (crc==minucrc) return true;
  return false;

}

void UIP_ADE7912::bgcalc(uint8_t maxms){
  //int32_t minI=8388607,maxI=-8388607,minU=8388607,maxU=-8388607; 
  //int crcerrors=0; // 
  int32_t readingoffset=344287;   
  
  unsigned long start = millis();
  double sqU,sqI,instP;
  int32_t measuredI,measuredU;
  
  while ( (millis()-start)<maxms) {  // 
    if (!readdata()) continue;
    numberOfSamples++;
    measuredU=(V1WV.value-readingoffset)/100;
    measuredI=(IWV.value-readingoffset)/1000;
    sqU= measuredU * measuredU;
    sumU += sqU;
    sqI = measuredI * measuredI;
    sumI += sqI;
    instP = measuredU * measuredI;
    sumP +=instP;
  }

  if (numberOfSamples<500 || (millis()-lastcalc)<3000  ) return;   

  //serial.print("             sumI= "); serial.println(sumI, DEC);
  //serial.print("             sumU= "); serial.println(sumU);
  //serial.print("             sumP= "); serial.println(sumP);
  serial.print("             numberOfSamples= "); serial.println(numberOfSamples, DEC);


  double Urms = 0.00905 * sqrt(sumU / numberOfSamples);
  double Irms = 0.00029 * sqrt(sumI / numberOfSamples);

  //Calculation power values
  double realPower = 0.00905 * (sumP / numberOfSamples) * 0.00029;
  double apparentPower = Urms * Irms;
  double powerFactor=realPower / apparentPower;


  numberOfSamples=0;
  sumU = 0;
  sumI = 0;
  sumP = 0;
  lastcalc=millis();

LUrms=Urms;
LIrms=Irms;
LrealPower=realPower;
LapparentPower=apparentPower;
LpowerFactor=powerFactor;


}



double UIP_ADE7912::getUrms() const {    return LUrms;}
double UIP_ADE7912::getIrms() const {    return LIrms;}
double UIP_ADE7912::getRealPower() const {    return LrealPower;}
double UIP_ADE7912::getApparentPower() const {    return LapparentPower;}
double UIP_ADE7912::getPowerFactor() const {    return LpowerFactor;}


#ifndef UIP_ADE7912_H
#define UIP_ADE7912_H

#include <Arduino.h>
#include <SPI.h>

// DEFINE COMMAND BYTES FOR ADE7913, 5-bit address. reads end 100:
#define STATUS0_READ      (0x9  << 3 | 0b100)
#define CONFIG_READ       (0x8  << 3 | 0b100)
#define TEMPOS_READ       (0x18 << 3 | 0b100)
#define IWV_READ          (0x0  << 3 | 0b100)  // Also starts 'burst' read of (IWV, V1WV, V2WV, ADC_CRC, STATUS0, CNT_SNAPSHOT)
#define EMI_CTRL_READ     (0xE  << 3 | 0b100)
#define V1WV_READ         (0x1  << 3 | 0b100)
#define V2WV_READ         (0x2  << 3 | 0b100)
#define ADC_CRC_READ      (0x4  << 3 | 0b100)
#define CNT_SNAPSHOT_READ (0x7  << 3 | 0b100)

// writes end 000:
#define CONFIG_WRITE     (0x8 << 3 | 0b000)
#define EMI_CTRL_WRITE   (0xE << 3 | 0b000)
#define SYNC_SNAP_WRITE  (0xB << 3 | 0b000)
#define LOCK_KEY_WRITE   (0xA << 3 | 0b000)

// miscelaneous bytes:
#define DUMMY_MSG        0x00               // Unused argument to SPI.Transfer()
#define LOCK_BYTE        0xCA
#define UNLOCK_BYTE      0x9C







class UIP_ADE7912 {
public:
    UIP_ADE7912(int slaveSelectPin,SPIClass &spi = SPI, Stream &serialPort = Serial); 
    void startup();
    void bgcalc(uint8_t maxms);
    double getUrms() const;
    double getIrms() const;
    double getRealPower() const;
    double getApparentPower() const;
    double getPowerFactor() const;



    
private:
    int slaveSelectPin;
    SPIClass &spi;
    boolean writeADE7913_check(byte writeTo, byte writeMsg, byte readFrom);
    SPISettings spiSettings;
    Stream &serial;
    // Setup signed 3-byte word to store ADC results, NB: use 4 bytes to match int32_t (signed 32bit integer)
    union threeByteWord {   int32_t value;   byte bytes[4];}; 
    uint16_t calculate_crc16_ccitt(const uint8_t* data, size_t length);
    uint16_t UIP_ADE7912::calculate_crc16_ccitt_for_three_words(volatile threeByteWord *word1, volatile threeByteWord *word2, volatile threeByteWord *word3);
    void readMultBytesADE7913(byte readFrom, volatile byte readTo[], int nBytes);
    void writeADE7913(byte writeTo, byte writeMsg);
    bool readdata();
    void extendSignBit(volatile threeByteWord *wordIn);

    double LUrms = 0;
    double LIrms = 0;
    double LrealPower = 0;
    double LapparentPower = 0;
    double LpowerFactor=1;

    unsigned int numberOfSamples=0; 
    unsigned long long sumU=0;
    unsigned long long sumI=0;
    unsigned long long sumP=0;
    unsigned long lastcalc = millis();

    const int nMaxWriteTry = 100;
    volatile threeByteWord IWV;
    volatile threeByteWord V1WV;
    volatile threeByteWord V2WV;
    volatile byte ADC_CRC[2];
    volatile byte STATUS0[1];
    volatile byte CNT_SNAPSHOT[2];
    volatile byte ADC_CRC_burst[2];
    volatile byte CONFIG[1];
    volatile byte TEMPOS[1];
    volatile byte EMI_CTRL[1];



};

#endif
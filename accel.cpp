// +-----+-----+---------+------+---+---Pi 3A+-+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
// |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
// |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
// |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
// |     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
// |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
// |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
// |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
// |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
// |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
// |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
// |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
// |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
// |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
// |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
// |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
// |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
// |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
// |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
// |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+---Pi 3A+-+---+------+---------+-----+-----+

// g++ -Wall -o  accel accel.cpp -lbcm2835

#include <stdio.h>
#include <bcm2835.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include "accel.hpp"
#include "registers.hpp"

#define BUTTON_PIN  RPI_GPIO_P1_16


typedef struct accelCpyStruct{
  
  int16_t xAs;
  int16_t yAs;
  int16_t zAs;
  
} axes_t;

int spi_init(){
  
  
  if (!bcm2835_spi_begin())
  {
    printf("bcm2835_spi_begin failed. Are you running as root??\n");
    exit(1);
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // The default
  bcm2835_spi_chipSelect(SPIPORT);                              // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
  
  //costum CSN
  bcm2835_gpio_fsel(CSN_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(CSN_pin, HIGH);
  return 0;
  
}


uint8_t readRegister(uint8_t address){
  
  uint8_t txData = 0;
  
  txData |= 1 << RW_bp;
  txData |= address;
  
  bcm2835_gpio_write(CSN_pin, LOW);
  bcm2835_spi_transfer(txData);
  uint8_t result = bcm2835_spi_transfer(0);
  bcm2835_gpio_write(CSN_pin, HIGH);
  
  return result;
  
}

void readMultiRegister(uint8_t iAddress, int n, uint8_t *dataOut){
  
  uint8_t txData = 0;
  
  txData |= 1 << RW_bp;
  txData |= 1 << MS_bp;
  txData |= iAddress;
  
  bcm2835_gpio_write(CSN_pin, LOW);
  bcm2835_spi_transfer(txData);
  
  for(int i = 0; i < n; i++){
    dataOut[i] = bcm2835_spi_transfer(0);
  }
  
  bcm2835_gpio_write(CSN_pin, HIGH);
  
}

axes_t readAxes(){
  
  uint8_t dataOut[6] = {0};
  axes_t result = {0};
  
  readMultiRegister(OUT_X_L_REG, 6, dataOut);
  
  result.xAs = (dataOut[0] + (dataOut[1] << 8)) >> 4;
  result.yAs = (dataOut[2] + (dataOut[3] << 8)) >> 4;
  result.zAs = (dataOut[4] + (dataOut[5] << 8)) >> 4;
  
  if(result.xAs >= FULLSCALE/2){
    result.xAs -= FULLSCALE;
  }
  if(result.yAs >= FULLSCALE/2){
    result.yAs -= FULLSCALE;
  }
  if(result.zAs >= FULLSCALE/2){
    result.zAs -= FULLSCALE;
  }
  
  result.xAs *= -1;
  result.yAs *= -1;
  result.zAs *= -1;
  
  
  return result;
  
}

void printAxes(axes_t uut){
  
  printf("sensorOutput X = %d\n", uut.xAs);fflush(stdout);
  printf("sensorOutput Y = %d\n", uut.yAs);fflush(stdout);
  printf("sensorOutput Z = %d\n", uut.zAs);fflush(stdout);
  
}


void writeRegister(uint8_t address, uint8_t value){
  
  uint8_t txData = 0;
  
  txData |= 0 << RW_bp;
  txData |= 0 << MS_bp;
  txData |= address;
  
  bcm2835_gpio_write(CSN_pin, LOW);
  bcm2835_spi_transfer(txData);
  bcm2835_spi_transfer(value);
  bcm2835_gpio_write(CSN_pin, HIGH);
  
}

void accel_init(){
  
  //enable
  bcm2835_gpio_fsel(VCC_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(VCC_pin, HIGH);
  
  //interrupt
  bcm2835_gpio_fsel(INT_pin, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(INT_pin, BCM2835_GPIO_PUD_DOWN);
  
  
  writeRegister(CTRL_REG1, CTRL_REG1_VALUE);
  writeRegister(CTRL_REG2, CTRL_REG2_VALUE);
  writeRegister(CTRL_REG3, CTRL_REG3_VALUE);
  writeRegister(CTRL_REG4, CTRL_REG4_VALUE);
  //interrupt
  writeRegister(INT1_CFG, INT1_CFG_VALUE);
  writeRegister(INT1_THS, INT1_THS_VALUE);
  writeRegister(INT1_DURATION, INT1_DURATION_VALUE);
  
  sleep(0.1);
  
  uint8_t dataOut = readRegister(CTRL_REG1);
  
  
  dataOut = readRegister(CTRL_REG1);
  while(dataOut != CTRL_REG1_VALUE){
    printf("reg 1 incorrect [%d] != %d\n", dataOut, CTRL_REG1_VALUE);
    writeRegister(CTRL_REG1, CTRL_REG1_VALUE);
    dataOut = readRegister(CTRL_REG1);
  }
  
  dataOut = readRegister(CTRL_REG2);
  while(dataOut != CTRL_REG2_VALUE){
    printf("reg 2 incorrect [%d] != %d\n", dataOut, CTRL_REG2_VALUE);
    writeRegister(CTRL_REG2, CTRL_REG2_VALUE);
    dataOut = readRegister(CTRL_REG2);
  }
  
  dataOut = readRegister(CTRL_REG3);
  while(dataOut != CTRL_REG3_VALUE){
    printf("reg 3 incorrect [%d] != %d\n", dataOut, CTRL_REG3_VALUE);
    writeRegister(CTRL_REG3, CTRL_REG3_VALUE);
    dataOut = readRegister(CTRL_REG3);
  }
  
  dataOut = readRegister(CTRL_REG4);
  while(dataOut != CTRL_REG4_VALUE){
    printf("reg 4 incorrect [%d] != %d\n", dataOut, CTRL_REG4_VALUE);
    writeRegister(CTRL_REG4, CTRL_REG4_VALUE);
    dataOut = readRegister(CTRL_REG4);
  }
  
  dataOut = readRegister(CTRL_REG5);
  while(dataOut != CTRL_REG5_VALUE){
    printf("reg 5 incorrect [%d] != %d\n", dataOut, CTRL_REG5_VALUE);
    writeRegister(CTRL_REG5, CTRL_REG5_VALUE);
    dataOut = readRegister(CTRL_REG5);
  }
  
  dataOut = readRegister(CTRL_REG6);
  while(dataOut != CTRL_REG6_VALUE){
    printf("reg 6 incorrect [%d] != %d\n", dataOut, CTRL_REG6_VALUE);
    writeRegister(CTRL_REG6, CTRL_REG6_VALUE);
    dataOut = readRegister(CTRL_REG6);
  }
  
  dataOut = readRegister(INT1_CFG);
  while(dataOut != INT1_CFG_VALUE){
    printf("INT1 CFG incorrect [%d] != %d\n", dataOut, INT1_CFG_VALUE);
    writeRegister(INT1_CFG, INT1_CFG_VALUE);
    dataOut = readRegister(INT1_CFG);
  }
  
  dataOut = readRegister(INT1_THS);
  while(dataOut != INT1_THS_VALUE){
    printf("INT1 THS incorrect [%d] != %d\n", dataOut, INT1_THS_VALUE);
    writeRegister(INT1_THS, INT1_THS_VALUE);
    dataOut = readRegister(INT1_THS);
  }
  
  dataOut = readRegister(INT1_DURATION);
  while(dataOut != INT1_DURATION_VALUE){
    printf("INT1 DURATION incorrect [%d] != %d\n", dataOut, INT1_DURATION_VALUE);
    writeRegister(INT1_DURATION, INT1_DURATION_VALUE);
    dataOut = readRegister(INT1_DURATION);
  }
  
  dataOut = readRegister(ACT_THS);
  while(dataOut != ACT_THS_VALUE){
    printf("ACT THS incorrect [%d] != %d\n", dataOut, ACT_THS_VALUE);
    writeRegister(ACT_THS, ACT_THS_VALUE);
    dataOut = readRegister(ACT_THS);
  }
  
  dataOut = readRegister(ACT_DURATION);
  while(dataOut != ACT_DURATION_VALUE){
    printf("ACT DURATION incorrect [%d] != %d\n", dataOut, ACT_DURATION_VALUE);
    writeRegister(ACT_DURATION, ACT_DURATION_VALUE);
    dataOut = readRegister(ACT_DURATION);
  }
  
  
}

bool newDataAvailable(){
  
  uint8_t dataOut = readRegister(STATUS_REG);
  if(dataOut & 0x08)
    return 1;
  
  return 0;
  
}

uint8_t clearInt(){
  
  uint8_t dataOut = readRegister(INT1_SRC);
  //dataOut &= (INT1_CFG_VALUE + 64);
  //printf("data out = %d\n", dataOut);
 
  return dataOut;
}

bool accel_checkInt(){
  
  if(bcm2835_gpio_lev(INT_pin) == HIGH){
    return true;
  }
    
  return false;
}

double getTime_ms(){
  time_t currentSec;
  timespec tCurrent;
  
  currentSec = time(NULL);
  clock_gettime(CLOCK_REALTIME, &tCurrent);
  double time_ms = 0;
  time_ms = (double)floor(tCurrent.tv_nsec/1000000.0);
  time_ms /= 1000.0;
  time_ms += currentSec;
  return time_ms;
  
}

void init_button(){
    
  // Set RPI pin P1-15 to be an input
  bcm2835_gpio_fsel(BUTTON_PIN, BCM2835_GPIO_FSEL_INPT);
  //  with a pullup
  bcm2835_gpio_set_pud(BUTTON_PIN, BCM2835_GPIO_PUD_DOWN);
  
}


int buttonPressed(){
  
  
	uint8_t value = bcm2835_gpio_lev(BUTTON_PIN);
  return value == HIGH;
  
}

int main(int argc, char *argv[]){

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }

  spi_init();
  accel_init();
  
  
  axes_t sensorOutput;
  
  double nRecordings = 10000;
  double intTime = 0;
  double currentTime = 0;
  int recordingIndex = 0;
  int nInterrupts = 0;
  int inInterrupt = 0;
  
  while(!buttonPressed());
  
  printf("starting program\n");fflush(stdout);
  sleep(1);
  
  
  double startTime = getTime_ms();
  char fileName[40] = {0};
  sprintf(fileName, "/home/pi/accelero/data/%d.csv", (int) startTime);
  FILE *fp = fopen(fileName, "w");
  if(fp == NULL)
    printf("cant open File\n");
  
  
  double deltaTime = 0;
  while(!buttonPressed()){
    currentTime = getTime_ms() - startTime;
    
    
    if(accel_checkInt() && !inInterrupt){
      //printf("amount of interrupts = %d\n", nInterrupts);
      inInterrupt = 100;
      intTime = getTime_ms() - startTime;
      nInterrupts++;
      
    }
    
    printf("%f - %f = %f || %d\n", currentTime, intTime, (currentTime - intTime), (currentTime - intTime > 1.0));
    if(inInterrupt == 100 && (currentTime - intTime > 1.0)){
      clearInt();
      sleep(0.01);
      inInterrupt = 0;
    }
    
    
    if( newDataAvailable()){
      sensorOutput = readAxes();
      
      fprintf(fp, "%.3f,%d,%d,%d,%d\n", currentTime, sensorOutput.xAs, sensorOutput.yAs, sensorOutput.zAs, inInterrupt);
      //printf("%.3f,%d,%d,%d,%d\n", currentTime, sensorOutput.xAs, sensorOutput.yAs, sensorOutput.zAs, inInterrupt);
      recordingIndex++;
    }
    
    if(buttonPressed()){
      printf("ending program\n");fflush(stdout);
    }
  }
  
  
  bcm2835_gpio_write(VCC_pin, LOW);
  
  fclose(fp);
  bcm2835_spi_end();
  bcm2835_close();

  return 0;
};






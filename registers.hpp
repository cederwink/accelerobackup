#ifndef _REGISTERS_HPP
#define _REGISTERS_HPP

  //key bit locations
  //rw bit 1 = r & 0 = w
  #define RW_bp  7
  //multiple read bit 1 = increment every read action
  #define MS_bp  6



  #define WHO_AM_I_REG 0x0F
  
  //data rate & power mode register
  #define CTRL_REG1  0x20
  #define CTRL_REG1_VALUE  0b01110111
  //high pass configurations
  #define CTRL_REG2  0x21
  #define CTRL_REG2_VALUE  0b10000000
  //interrupt enable register
  #define CTRL_REG3  0x22
  #define CTRL_REG3_VALUE  0b01000000
  //resulotion select register
  #define CTRL_REG4  0x23
  #define CTRL_REG4_VALUE  0b00001000
  //interrupt configurations
  #define CTRL_REG5  0x24
  #define CTRL_REG5_VALUE  0b00001000 //0b00001010
  
  #define CTRL_REG6  0x25
  #define CTRL_REG6_VALUE  0b00001010
  
  #define REF_REG     0x26
  #define STATUS_REG  0x27  
  
  #define INT1_CFG        0x30
  #define INT1_SRC        0x31
  #define INT1_THS        0x32
  #define INT1_DURATION   0x33
  
  #define INT1_CFG_VALUE        0b01100000
  #define INT1_THS_VALUE        0b01001010
  #define INT1_DURATION_VALUE   0b00000110
  
  #define INT2_CFG        0x34
  #define INT2_SRC        0x35
  #define INT2_THS        0x36
  #define INT2_DURATION   0x37
  
  #define INT2_CFG_VALUE        0b00000000
  #define INT2_THS_VALUE        0b00000000
  #define INT2_DURATION_VALUE   0b00000000
  
  #define ACT_THS         0x3E  
  #define ACT_DURATION    0x3F  
  
  #define ACT_THS_VALUE         0b00100110  
  #define ACT_DURATION_VALUE    0b01011111  
  

  #define OUT_X_L_REG  0x28
  #define OUT_X_H_REG  0x29
  #define OUT_Y_L_REG  0x2A
  #define OUT_Y_H_REG  0x2B
  #define OUT_Z_L_REG  0x2C
  #define OUT_Z_H_REG  0x2D











#endif //_REGISTERS_HPP
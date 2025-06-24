#ifndef MT6835_H
#define MT6835_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <SPI.h>

// Operation codes
#define MT6835_OP_READ  0x3
#define MT6835_OP_WRITE 0x6
#define MT6835_OP_PROG  0xC
#define MT6835_OP_ZERO  0x5
#define MT6835_OP_ANGLE 0xA

// Masks(not needed in C driver, values kept for reference)
#define MT6835_CMD_MASK  0xF00000
#define MT6835_ADDR_MASK 0x0FFF00
#define MT6835_DATA_MASK 0x0000FF

#define MT6835_CPR 2097152U

// Status flags
#define MT6835_STATUS_OVERSPEED 0x01
#define MT6835_STATUS_WEAKFIELD 0x02
#define MT6835_STATUS_UNDERVOLT 0x04
#define MT6835_CRC_ERROR        0x08

#define MT6835_WRITE_ACK 0x55

// Register addresses
#define MT6835_REG_USERID      0x001
#define MT6835_REG_ANGLE1      0x003
#define MT6835_REG_ANGLE2      0x004
#define MT6835_REG_ANGLE3      0x005
#define MT6835_REG_ANGLE4      0x006
#define MT6835_REG_ABZ_RES1    0x007
#define MT6835_REG_ABZ_RES2    0x008
#define MT6835_REG_ZERO1       0x009
#define MT6835_REG_ZERO2       0x00A
#define MT6835_REG_OPTS0       0x00A
#define MT6835_REG_OPTS1       0x00B
#define MT6835_REG_OPTS2       0x00C
#define MT6835_REG_OPTS3       0x00D
#define MT6835_REG_OPTS4       0x00E
#define MT6835_REG_OPTS5       0x011
#define MT6835_REG_NLC_BASE    0x013
#define MT6835_REG_CAL_STATUS  0x113

// SPI defaults
#define MT6835_BITORDER MSBFIRST
static const SPISettings MT6835_SPI_SETTINGS = {1000000, MT6835_BITORDER, SPI_MODE3};

// Data structures

typedef union {
    struct Fields {
        uint32_t unused :8;
        uint32_t data   :8;
        uint32_t addr   :12;
        uint32_t cmd    :4;
    } fields;
    uint32_t val;
} MT6835Command;

// Driver state
typedef struct {
    SPIClass*    spi;
    SPISettings  settings;
    int          nCS;
    uint8_t      laststatus;
    uint8_t      lastcrc;
    bool         checkcrc;
} MT6835;

// API functions
void        MT6835_create(MT6835* self, SPISettings settings, int nCS);
void        MT6835_init(MT6835* self, SPIClass* spi);
float       MT6835_getCurrentAngle(MT6835* self);
uint32_t    MT6835_readRawAngle21(MT6835* self);
uint8_t     MT6835_getStatus(MT6835* self);
uint8_t     MT6835_getCalibrationStatus(MT6835* self);
bool        MT6835_setZeroFromCurrentPosition(MT6835* self);
bool        MT6835_writeEEPROM(MT6835* self);
uint8_t     MT6835_getBandwidth(MT6835* self);
void        MT6835_setBandwidth(MT6835* self, uint8_t bw);
uint8_t     MT6835_getHysteresis(MT6835* self);
void        MT6835_setHysteresis(MT6835* self, uint8_t hyst);
uint8_t     MT6835_getRotationDirection(MT6835* self);
void        MT6835_setRotationDirection(MT6835* self, uint8_t dir);
uint16_t    MT6835_getABZResolution(MT6835* self);
void        MT6835_setABZResolution(MT6835* self, uint16_t res);
bool        MT6835_isABZEnabled(MT6835* self);
void        MT6835_setABZEnabled(MT6835* self, bool enabled);
bool        MT6835_isABSwapped(MT6835* self);
void        MT6835_setABSwapped(MT6835* self, bool swapped);
uint16_t    MT6835_getZeroPosition(MT6835* self);
void        MT6835_setZeroPosition(MT6835* self, uint16_t pos);
MT6835Options1 MT6835_getOptions1(MT6835* self);
void        MT6835_setOptions1(MT6835* self, MT6835Options1 opts);
MT6835Options2 MT6835_getOptions2(MT6835* self);
void        MT6835_setOptions2(MT6835* self, MT6835Options2 opts);
MT6835Options3 MT6835_getOptions3(MT6835* self);
void        MT6835_setOptions3(MT6835* self, MT6835Options3 opts);
MT6835Options4 MT6835_getOptions4(MT6835* self);
void        MT6835_setOptions4(MT6835* self, MT6835Options4 opts);
uint8_t     MT6835_readRegister(MT6835* self, uint16_t reg);
bool        MT6835_writeRegister(MT6835* self, uint16_t reg, uint8_t value);
void        MT6835_transfer24(MT6835* self, MT6835Command* cmd);
uint8_t     MT6835_calcCrc(uint32_t angle, uint8_t status);
uint32_t    swap_bytes(uint32_t net);

#endif // MT6835_H // MT6835_H

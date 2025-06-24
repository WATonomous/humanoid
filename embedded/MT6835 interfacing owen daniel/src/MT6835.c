#include "MT6835.h"
#include "foc_utils.h"

// Byte-swap helper
uint32_t swap_bytes(uint32_t net) {
    return __builtin_bswap32(net);
}

// Low-level 24-bit transfer over SPI
void MT6835_transfer24(MT6835* self, MT6835Command* cmd) {
    uint32_t buff = swap_bytes(cmd->val);
    self->spi->beginTransaction(self->settings);
    if (self->nCS >= 0) digitalWrite(self->nCS, LOW);
    self->spi->transfer(&buff, 3);
    if (self->nCS >= 0) digitalWrite(self->nCS, HIGH);
    self->spi->endTransaction();
    cmd->val = swap_bytes(buff);
}

// Read a single register
uint8_t MT6835_readRegister(MT6835* self, uint16_t reg) {
    MT6835Command cmd = { .cmd = MT6835_OP_READ, .addr = reg, .data = 0 };
    MT6835_transfer24(self, &cmd);
    return cmd.data;
}

// Write a single register
bool MT6835_writeRegister(MT6835* self, uint16_t reg, uint8_t value) {
    MT6835Command cmd = { .cmd = MT6835_OP_WRITE, .addr = reg, .data = value };
    MT6835_transfer24(self, &cmd);
    return cmd.data == MT6835_WRITE_ACK;
}

// Create / initialize the struct
void MT6835_create(MT6835* self, SPISettings settings, int nCS) {
    self->settings = settings;
    self->nCS = nCS;
    self->spi = NULL;
    self->checkcrc = false;
    self->laststatus = 0;
    self->lastcrc = 0;
}

// Destructor placeholder (no dynamic allocations here)
void MT6835_destroy(MT6835* self) {
    // no-op
}

// Begin communication
void MT6835_init(MT6835* self, SPIClass* spi) {
    self->spi = spi;
    if (self->nCS >= 0) {
        pinMode(self->nCS, OUTPUT);
        digitalWrite(self->nCS, HIGH);
    }
    spi->begin();
}

// Calculate CRC8 over 21-bit angle + 3 status bits
uint8_t MT6835_calcCrc(uint32_t angle, uint8_t status) {
    uint8_t crc = 0x00;
    uint8_t input;
    for (int shift = 13; shift >= 5; shift -= 8) {
        input = (angle >> shift) & 0xFF;
        crc ^= input;
        for (int k = 0; k < 8; k++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    input = ((angle << 3) & 0xF8) | (status & 0x07);
    crc ^= input;
    for (int k = 0; k < 8; k++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    return crc;
}

// Read raw 21-bit angle value
uint32_t MT6835_readRawAngle21(MT6835* self) {
    uint8_t data[6] = {0};
    data[0] = (MT6835_OP_ANGLE << 4);
    data[1] = MT6835_REG_ANGLE1;
    self->spi->beginTransaction(self->settings);
    if (self->nCS >= 0) digitalWrite(self->nCS, LOW);
    self->spi->transfer(data, 6);
    if (self->nCS >= 0) digitalWrite(self->nCS, HIGH);
    self->spi->endTransaction();
    self->laststatus = data[4] & 0x07;
    self->lastcrc = data[5];
    return (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
}

// Public API

float MT6835_getCurrentAngle(MT6835* self) {
    uint32_t raw = MT6835_readRawAngle21(self);
    if (self->checkcrc) {
        if (self->lastcrc != MT6835_calcCrc(raw, self->laststatus)) {
            self->laststatus |= MT6835_CRC_ERROR;
            return -1;
        }
    }
    return raw / (float)MT6835_CPR * _2PI;
}

uint8_t MT6835_getStatus(MT6835* self) {
    return self->laststatus;
}

uint8_t MT6835_getCalibrationStatus(MT6835* self) {
    uint8_t data[3] = {0};
    data[0] = (MT6835_OP_READ << 4) | (MT6835_REG_CAL_STATUS >> 8);
    data[1] = MT6835_REG_CAL_STATUS & 0xFF;
    self->spi->beginTransaction(self->settings);
    if (self->nCS >= 0) digitalWrite(self->nCS, LOW);
    self->spi->transfer(data, 3);
    if (self->nCS >= 0) digitalWrite(self->nCS, HIGH);
    self->spi->endTransaction();
    return data[2] >> 6;
}

bool MT6835_setZeroFromCurrentPosition(MT6835* self) {
    MT6835Command cmd = { .cmd = MT6835_OP_ZERO, .addr = 0 };
    MT6835_transfer24(self, &cmd);
    return cmd.data == MT6835_WRITE_ACK;
}

bool MT6835_writeEEPROM(MT6835* self) {
    delay(1);
    MT6835Command cmd = { .cmd = MT6835_OP_PROG, .addr = 0 };
    MT6835_transfer24(self, &cmd);
    return cmd.data == MT6835_WRITE_ACK;
}

uint8_t MT6835_getBandwidth(MT6835* self) {
    MT6835Options5 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS5) };
    return opts.bw;
}

void MT6835_setBandwidth(MT6835* self, uint8_t bw) {
    MT6835Options5 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS5) };
    opts.bw = bw;
    MT6835_writeRegister(self, MT6835_REG_OPTS5, opts.reg);
}

uint8_t MT6835_getHysteresis(MT6835* self) {
    MT6835Options3 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS3) };
    return opts.hyst;
}

void MT6835_setHysteresis(MT6835* self, uint8_t hyst) {
    MT6835Options3 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS3) };
    opts.hyst = hyst;
    MT6835_writeRegister(self, MT6835_REG_OPTS3, opts.reg);
}

uint8_t MT6835_getRotationDirection(MT6835* self) {
    MT6835Options3 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS3) };
    return opts.rot_dir;
}

void MT6835_setRotationDirection(MT6835* self, uint8_t dir) {
    MT6835Options3 opts = { .reg = MT6835_readRegister(self, MT6835_REG_OPTS3) };
    opts.rot_dir = dir;
    MT6835_writeRegister(self, MT6835_REG_OPTS3, opts.reg);
}

uint16_t MT6835_getABZResolution(MT6835* self) {
    uint8_t hi = MT6835_readRegister(self, MT6835_REG_ABZ_RES1);
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    return (hi << 6) | lo.abz_res_low;
}

void MT6835_setABZResolution(MT6835* self, uint16_t res) {
    uint8_t hi = res >> 6;
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    lo.abz_res_low = res & 0x3F;
    MT6835_writeRegister(self, MT6835_REG_ABZ_RES1, hi);
    MT6835_writeRegister(self, MT6835_REG_ABZ_RES2, lo.reg);
}

bool MT6835_isABZEnabled(MT6835* self) {
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    return lo.abz_off == 0;
}

void MT6835_setABZEnabled(MT6835* self, bool enabled) {
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    lo.abz_off = enabled ? 0 : 1;
    MT6835_writeRegister(self, MT6835_REG_ABZ_RES2, lo.reg);
}

bool MT6835_isABSwapped(MT6835* self) {
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    return lo.ab_swap == 1;
}

void MT6835_setABSwapped(MT6835* self, bool swapped) {
    MT6835ABZRes lo = { .reg = MT6835_readRegister(self, MT6835_REG_ABZ_RES2) };
    lo.ab_swap = swapped ? 1 : 0;
    MT6835_writeRegister(self, MT6835_REG_ABZ_RES2, lo.reg);
}

uint16_t MT6835_getZeroPosition(MT6835* self) {
    uint8_t hi = MT6835_readRegister(self, MT6835_REG_ZERO1);
    MT6835Options0 lo = { .reg = MT6835_readRegister(self, MT6835_REG_ZERO2) };
    return (hi << 4) | lo.zero_pos_low;
}

void MT6835_setZeroPosition(MT6835* self, uint16_t pos) {
    uint8_t hi = pos >> 4;
    MT6835Options0 lo = { .reg = MT6835_readRegister(self, MT6835_REG_ZERO2) };
    lo.zero_pos_low = pos & 0x0F;
    MT6835_writeRegister(self, MT6835_REG_ZERO1, hi);
    MT6835_writeRegister(self, MT6835_REG_ZERO2, lo.reg);
}

MT6835Options1 MT6835_getOptions1(MT6835* self) {
    return (MT6835Options1){ .reg = MT6835_readRegister(self, MT6835_REG_OPTS1) };
}

void MT6835_setOptions1(MT6835* self, MT6835Options1 opts) {
    MT6835_writeRegister(self, MT6835_REG_OPTS1, opts.reg);
}

MT6835Options2 MT6835_getOptions2(MT6835* self) {
    return (MT6835Options2){ .reg = MT6835_readRegister(self, MT6835_REG_OPTS2) };
}

void MT6835_setOptions2(MT6835* self, MT6835Options2 opts) {
    MT6835Options2 val = MT6835_getOptions2(self);
    val.nlc_en = opts.nlc_en;
    val.pwm_fq = opts.pwm_fq;
    val.pwm_pol = opts.pwm_pol;
    val.pwm_sel = opts.pwm_sel;
    MT6835_writeRegister(self, MT6835_REG_OPTS2, val.reg);
}

MT6835Options3 MT6835_getOptions3(MT6835* self) {
    return (MT6835Options3){ .reg = MT6835_readRegister(self, MT6835_REG_OPTS3) };
}

void MT6835_setOptions3(MT6835* self, MT6835Options3 opts) {
    MT6835Options3 val = MT6835_getOptions3(self);
    val.rot_dir = opts.rot_dir;
    val.hyst = opts.hyst;
    MT6835_writeRegister(self, MT6835_REG_OPTS3, val.reg);
}

MT6835Options4 MT6835_getOptions4(MT6835* self) {
    return (MT6835Options4){ .reg = MT6835_readRegister(self, MT6835_REG_OPTS4) };
}

void MT6835_setOptions4(MT6835* self, MT6835Options4 opts) {
    MT6835Options4 val = MT6835_getOptions4(self);
    val.gpio_ds = opts.gpio_ds;
    val.autocal_freq = opts.autocal_freq;
    MT6835_writeRegister(self, MT6835_REG_OPTS4, val.reg);
}

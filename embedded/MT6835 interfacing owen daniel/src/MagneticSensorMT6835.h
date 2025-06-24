#pragma once

#include "Sensor.h"      // your C‐style “base class”
#include "MT6835.h"      // the C driver you already wrote

typedef struct MagneticSensorMT6835 {
    // “Base classes” as members
    Sensor           sensor;
    MT6835           encoder;
} MagneticSensorMT6835;

// Constructor / init
void MagneticSensorMT6835_create(
    MagneticSensorMT6835* self,
    int nCS,
    SPISettings settings,
    SPIClass* spi
);

// Virtual override of getSensorAngle
float MagneticSensorMT6835_getSensorAngle(MagneticSensorMT6835* self);

// Virtual override of init()
void MagneticSensorMT6835_init(MagneticSensorMT6835* self, SPIClass* spi);

#include "MagneticSensorMT6835.h"

// Constructor: initialize MT6835 driver and Sensor base
void MagneticSensorMT6835_create(MagneticSensorMT6835* self,
                                 int nCS,
                                 SPISettings settings,
                                 SPIClass* spi) {
    // MT6835 “superclass”
    MT6835_create(&self->encoder, settings, nCS);
    MT6835_init  (&self->encoder, spi);

    // Sensor “superclass”
    Sensor_init(&self->sensor);

    // Override virtual methods
    self->sensor.instance = self;
    self->sensor.getAngle = (float (*)(void*))MagneticSensorMT6835_getSensorAngle;
    self->sensor.init     = (void (*)(void*, SPIClass*))MagneticSensorMT6835_init;
}

// Destructor placeholder
void MagneticSensorMT6835_destroy(MagneticSensorMT6835* self) {
    // nothing to free
}

// Forward getSensorAngle → MT6835_getCurrentAngle
float MagneticSensorMT6835_getSensorAngle(MagneticSensorMT6835* self) {
    return MT6835_getCurrentAngle(&self->encoder);
}

// init override: reinitialize encoder and base sensor
void MagneticSensorMT6835_init(MagneticSensorMT6835* self, SPIClass* spi) {
    MT6835_init(&self->encoder, spi);
    Sensor_init(&self->sensor);
}

#ifndef GY33_H
#define GY33_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Endereço padrão do GY-33 (TCS34725)
#define GY33_I2C_ADDR 0x29

// Registradores do TCS34725
#define ENABLE_REG   0x80
#define ATIME_REG    0x81
#define CONTROL_REG  0x8F
#define ID_REG       0x92
#define STATUS_REG   0x93
#define CDATA_REG    0x94
#define RDATA_REG    0x96
#define GDATA_REG    0x98
#define BDATA_REG    0x9A

// Estrutura para armazenar os valores lidos
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c; // Clear (intensidade total de luz)
} gy33_color_t;

// Funções públicas
void gy33_init(i2c_inst_t *i2c_port);
void gy33_read_color(i2c_inst_t *i2c_port, gy33_color_t *color);

#endif

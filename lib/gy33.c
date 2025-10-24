#include "gy33.h"

// Função auxiliar para escrever em um registrador
static void gy33_write_register(i2c_inst_t *i2c_port, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c_port, GY33_I2C_ADDR, buffer, 2, false);
}

// Função auxiliar para ler de um registrador (16 bits)
static uint16_t gy33_read_register(i2c_inst_t *i2c_port, uint8_t reg) {
    uint8_t buffer[2];
    i2c_write_blocking(i2c_port, GY33_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_port, GY33_I2C_ADDR, buffer, 2, false);
    return (buffer[1] << 8) | buffer[0];
}

// Inicialização do GY-33 (TCS34725)
void gy33_init(i2c_inst_t *i2c_port) {
    gy33_write_register(i2c_port, ENABLE_REG, 0x03);  // Power ON + ADC enable
    gy33_write_register(i2c_port, ATIME_REG, 0xF5);   // Tempo integração ~ 103 ms
    gy33_write_register(i2c_port, CONTROL_REG, 0x00); // Ganho 1x
}

// Leitura dos valores RGB + Clear
void gy33_read_color(i2c_inst_t *i2c_port, gy33_color_t *color) {
    color->c = gy33_read_register(i2c_port, CDATA_REG);
    color->r = gy33_read_register(i2c_port, RDATA_REG);
    color->g = gy33_read_register(i2c_port, GDATA_REG);
    color->b = gy33_read_register(i2c_port, BDATA_REG);
}

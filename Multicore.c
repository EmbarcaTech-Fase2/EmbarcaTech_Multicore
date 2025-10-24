#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "ws2812.pio.h"
#include "lib/ssd1306.h"
#include "lib/bh1750_light_sensor.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/matriz_leds.h"
#include "lib/led_rgb.h"
#include "gy33.h"
#include <string.h>
#include "pico/multicore.h"

#define I2C_SDA_DISPLAY 14    //Pino SDA - Dados para display SSD1306
#define I2C_SCL_DISPLAY 15    //Pino SCL - Clock para display SSD1306
#define WS2812_PIN 7  //Pino do WS2812
#define BUZZER_PIN 21 //Pino do buzzer
#define BOTAO_A 5       //Pino do botão A
#define BOTAO_B 6       //Pino do botão B
#define BOTAO_JOYSTICK 22 //Pino do botão joystick
#define IS_RGBW false //Maquina PIO para RGBW
#define LUX_MIN 100      // Limite de luminosidade em lux
#define RED_ALERT 200    // Limite para vermelho intenso

//Definições do sensor GY-33
#define GY33_I2C_ADDR 0x29 //Endereço do GY-33 no barramento I2C
#define I2C_PORT_COR i2c0
#define SDA_PIN 0
#define SCL_PIN 1

//Para o sensor de luz BH1750. Endereço 0x23
#define I2C_PORT_LUX i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA_LUX 2                   // 0 ou 2
#define I2C_SCL_LUX 3                   // 1 ou 3

enum colors {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK};

typedef struct{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
    uint16_t lux;
} sensor_data_t;

sensor_data_t sensor_data;

//Variáveis globais
uint buzzer_slice; //Slice para o buzzer
ssd1306_t ssd;
volatile uint8_t tela_atual = 0; //0 = GY-33, 1 = BH1750, etc
volatile uint32_t last_press_time = 0; //Para debounce
volatile int cor_atual = RED;  //Cor atual da matriz de LED

void buzzer_beep(uint slice, int duracao_ms){
    pwm_set_enabled(slice, true);   // Liga o buzzer
    sleep_ms(duracao_ms);
    pwm_set_enabled(slice, false);  // Desliga
}

void botao_a_callback(uint gpio, uint32_t events){
    uint32_t agora = to_ms_since_boot(get_absolute_time());
    if(agora - last_press_time > 250){   // debounce de 250ms
        last_press_time = agora;
        tela_atual++;
        if(tela_atual > 1){ //verifica e alterna a tela
            tela_atual = 0;
        }
    }
}

void botao_b_callback(uint gpio, uint32_t events){
    uint32_t agora = to_ms_since_boot(get_absolute_time());
    if(agora - last_press_time > 250){   // debounce de 250ms
        last_press_time = agora;
        cor_atual++;
        switch (cor_atual)
        {
        case RED:
            red();
            break;
        case GREEN:
            green();
            break;
        case BLUE:
            blue();
            break;
        case YELLOW:
            yellow();
            break;
        case CYAN:
            cyan();
            break;
        case MAGENTA:
            magenta();
            break;
        case WHITE:
            white();
            break;
        case BLACK:
            black();
            break;
        default:
            cor_atual = RED;
            red();
            break;
        }
    }
}

void botao_joystick_callback(uint gpio, uint32_t events){
    uint32_t agora = to_ms_since_boot(get_absolute_time());
    if(agora - last_press_time > 250){   // debounce de 250ms
        last_press_time = agora;
        //Desliga os leds da matriz
        set_one_led(0, 0, 0, 0);
        black();
        reset_usb_boot(0, 0);
    }
}

void gpio_irq_callback(uint gpio, uint32_t events){
    if(gpio == BOTAO_A){
        botao_a_callback(gpio, events);
    } else if(gpio == BOTAO_B){
        botao_b_callback(gpio, events);
    } else if(gpio == BOTAO_JOYSTICK){
        botao_joystick_callback(gpio, events);
    }
}

void inicializar_componentes(){
    stdio_init_all();

    //Inicializa os LEDs RGB
    led_init_all();
    red();

    //Inicializa o botão A com pull-up
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);

    //Inicializa o botão B com pull-up
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);

    //Inicializa o botão joystick com pull-up
    gpio_init(BOTAO_JOYSTICK);
    gpio_set_dir(BOTAO_JOYSTICK, GPIO_IN);
    gpio_pull_up(BOTAO_JOYSTICK);

    //Interrupção no botão A
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    //Interrupção no botão B
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    //Interrupção no botão joystick
    gpio_set_irq_enabled_with_callback(BOTAO_JOYSTICK, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);

    //Inicializa o pio
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    //Inicializa I2C para o display SSD1306
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA_DISPLAY, GPIO_FUNC_I2C); // Dados
    gpio_set_function(I2C_SCL_DISPLAY, GPIO_FUNC_I2C); // Clock
    gpio_pull_up(I2C_SDA_DISPLAY);
    gpio_pull_up(I2C_SCL_DISPLAY);
    //Inicializa display
    ssd1306_init(&ssd, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    //Inicializa buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);              // Slice para o buzzer
    float clkdiv = 125.0f;                                         // Clock divisor
    uint16_t wrap = (uint16_t)((125000000 / (clkdiv * 1000)) - 1); // Valor do Wrap
    pwm_set_clkdiv(buzzer_slice, clkdiv);                          // Define o clock
    pwm_set_wrap(buzzer_slice, wrap);                              // Define o wrap
    pwm_set_gpio_level(BUZZER_PIN, wrap * 0.3f);                   // Define duty
    pwm_set_enabled(buzzer_slice, false);                          // Começa desligado

    //Inicializa o sensor de cor - GY33
    i2c_init(I2C_PORT_COR, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("Iniciando GY-33...\n");
    gy33_init(I2C_PORT_COR);

    //Inicializa o sensor de luz BH1750
    bh1750_power_on(I2C_PORT_LUX);
}

void core1_entry(){
    char str_red[5];  
    char str_green[5]; 
    char str_blue[5];  
    char str_clear[5]; 
    char str_lux[10]; 
    while (true){
       int has_data = multicore_fifo_pop_blocking();
       if(has_data){
            uint16_t r, g, b, c, lux;
            r = sensor_data.red;
            g = sensor_data.green;
            b = sensor_data.blue;
            c = sensor_data.clear;
            lux = sensor_data.lux;

            r = r > 255 ? 255 : r;
            g = g > 255 ? 255 : g;
            b = b > 255 ? 255 : b;

            printf("Lux = %d\n", lux);

            printf("Cor detectada - R: %d, G: %d, B: %d, Clear: %d\n", r, g, b, c);
            sprintf(str_red, "%d R", r); 
            sprintf(str_green, "%d G", g); 
            sprintf(str_blue, "%d B", b);  
            sprintf(str_clear, "%d C", c);

            sprintf(str_lux, "%d Lux", lux);  

            if(lux < LUX_MIN)
                buzzer_beep(buzzer_slice, 100);   
            
            update_led_matrix_by_sensors(r, g, b, lux);
            if(r > RED_ALERT && r > g * 2 && r > b * 2)
                buzzer_beep(buzzer_slice, 500); 
            
            ssd1306_fill(&ssd, false);                          
            if(tela_atual == 0) //Sensor de cor GY33
                tela0(&ssd, r, g, b, str_red, str_green, str_blue, str_clear);
            else if(tela_atual == 1) //Sensor de luz BH1750
                tela1(&ssd, lux, str_lux);
            sleep_ms(500);
       }
    }
}

int main(){
    inicializar_componentes();
    multicore_launch_core1(core1_entry);

    while(true){
        //Leitura do sensor de cor
        gy33_color_t color;
        gy33_read_color(I2C_PORT_COR, &color);

        //Leitura do sensor de Luz BH1750
        uint16_t lux = bh1750_read_measurement(I2C_PORT_LUX);

        sensor_data.red = color.r;
        sensor_data.green = color.g;
        sensor_data.blue = color.b;
        sensor_data.clear = color.c;
        sensor_data.lux = lux;

        multicore_fifo_push_blocking(1);
    }
}

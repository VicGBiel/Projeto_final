#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"

// Definições de pinos e configurações do hardware
#define btn_a 5 // Definição do botão A
#define btn_b 6 // Definição do botão B
#define WS2812_PIN 7 // Pino onde os LEDs WS2812 estão conectados
#define BUZZER_PIN 10  // Defina o pino do buzzer
#define led_pin_green 11
#define led_pin_blue 12
#define led_pin_red 13
#define I2C_SDA 14
#define I2C_SCL 15
#define BTN_JOY 22
#define ADC_PIN_Y 26
#define ADC_PIN_X 27
#define NUM_PIXELS 25 // Número de LEDs na matriz 
#define I2C_PORT i2c1  
#define IS_RGBW false // Define se os LEDs são RGBW ou apenas RGB
#define ADC_MAX 4090
#define ADC_MIN 22
#define ADC_CENTER 1890

//Variáveis Globais
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)
volatile int estado = 0;
bool criticLvl = 0;
bool led_buffer[NUM_PIXELS]= {}; // Inicializa o buffer de LEDs com zeros
bool cor = true;
uint joy_x;
uint slice_buz;
uint8_t led_r = 0; // Intensidade do vermelho
uint8_t led_g = 0;  // Intensidade do verde
uint8_t led_b = 0;  // Intensidade do azul
ssd1306_t ssd; // Inicializa a estrutura do display

//Protótipos das funções
void initGPIO(); // Inicializa os pinos GPIO
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b); // Função para representar a cor em formato RGB
static inline void put_pixel(uint32_t pixel_grb); // Envia um pixel para o barramento WS2812
void atualizaFita(uint8_t r, uint8_t g, uint8_t b); // Atualiza todos os LEDs com base no buffer
static void gpio_irq_handler(uint gpio, uint32_t events); // Trata as interrupções 
void WS2812_setup(); // Configura a matriz de LEDS
void SSD1306_setup(); // Configura o display SSD1306
void PWM_setup();
void ADC_setup();
float map_adc(uint16_t adc_value);
void print_float_to_oled(ssd1306_t *ssd, float value, uint8_t x, uint8_t y, int change);
void atualizaEstado(int estado);
void play_tone(uint slice_num, uint freq, uint duty_cycle);
void updateSensors();
void nivelCritico();

//Função principal
int main(){
    stdio_init_all();
    initGPIO();
    WS2812_setup();
    i2c_init(I2C_PORT, 400 * 1000); //Inicializa o i2c em 400kHz
    SSD1306_setup();
    ADC_setup();
    PWM_setup();   

    // Configuração das interrupções dos botões
    gpio_set_irq_enabled_with_callback(btn_a, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(btn_b, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_JOY, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    ssd1306_draw_string(&ssd, "INICIALIZANDO", 5, 15);
    ssd1306_draw_string(&ssd, "Seja bem vindo", 5, 35);
    ssd1306_send_data(&ssd);
    sleep_ms(3000);
    ssd1306_fill(&ssd, !cor);
    ssd1306_send_data(&ssd);
    

    while (true) {
        updateSensors();
        nivelCritico();
        sleep_ms(300);
    }
}

void initGPIO(){
    //inicialização dos leds pwm e buzzer
        uint pwm_leds[] = {led_pin_green, led_pin_red, led_pin_blue, BUZZER_PIN};
        for (int i = 0; i < 4; i++) {
            gpio_set_function(pwm_leds[i], GPIO_FUNC_PWM);
        }
    //inicialização do botão A
    gpio_init(btn_a);
    gpio_set_dir(btn_a, GPIO_IN);
    gpio_pull_up(btn_a);  
    //inicialização do botão B
    gpio_init(btn_b);
    gpio_set_dir(btn_b, GPIO_IN);
    gpio_pull_up(btn_b);  
    //inicialização do botão do joystick
    gpio_init(BTN_JOY);
    gpio_set_dir(BTN_JOY, GPIO_IN);
    gpio_pull_up(BTN_JOY);  
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

void atualizaFita(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = urgb_u32(r, g, b); // Define a cor
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (led_buffer[i]) {
            put_pixel(color); // Liga o LED conforme o buffer
        } else {
            put_pixel(0);  // Desliga os LEDs
        }
    }
}

static void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    static int estadoAnterior = -1;

    if (current_time - last_time > 200000) { // 200ms de debounce
        last_time = current_time;

        if (gpio == btn_a) {
            last_time = current_time;
            if(estado >=0 && estado < 2){
                estado++;
                atualizaEstado(estado);
                estadoAnterior = estado;
            } else if (estado == 2){
                estado = 0;
                atualizaEstado(estado);
                estadoAnterior = estado;
            }
        } else if (gpio == btn_b){
            last_time = current_time;
            criticLvl = !criticLvl;

        } else if (gpio == BTN_JOY){
            last_time = current_time;
            reset_usb_boot(0,0);
        }
    }
}

void WS2812_setup(){
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
    atualizaFita(led_r, led_g, led_b);
}

void SSD1306_setup(){
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, 0x3C, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void ADC_setup(){
    adc_init();
    adc_gpio_init(ADC_PIN_Y);
    adc_gpio_init(ADC_PIN_X);
}

void PWM_setup(){
    slice_buz = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0);
    pwm_init(slice_buz, &config, true);

    uint slice_r = pwm_gpio_to_slice_num(led_pin_red);
    uint slice_g = pwm_gpio_to_slice_num(led_pin_green);
    uint slice_b = pwm_gpio_to_slice_num(led_pin_blue);
    pwm_set_enabled(slice_r, true);
    pwm_set_enabled(slice_g, true);
    pwm_set_enabled(slice_b, true);
    pwm_set_gpio_level(led_pin_red, 0);
    pwm_set_gpio_level(led_pin_green, 0);
    pwm_set_gpio_level(led_pin_blue, 0);   
}

float map_adc(uint16_t adc_value) {
    float humidity = 100.0f - (abs((int)adc_value - ADC_CENTER) * 100.0f / ADC_CENTER);
    
    // Garante que a umidade fique entre 0 e 100%
    if (humidity < 0) humidity = 0;
    if (humidity > 100) humidity = 100;
    
    return humidity;
}

void print_float_to_oled(ssd1306_t *ssd, float value, uint8_t x, uint8_t y, int change) {
    char buffer[20];                           // Buffer para a string formatada
    switch (change)
    {
    case 0:
        sprintf(buffer, "Umidade:%.f%%", value); // Formata o float para a string
        break;
    case 1:
        sprintf(buffer, "Luz:%.f%%", value); // Formata o float para a string
    }
    ssd1306_draw_string(ssd, buffer, x, y);    // Desenha a string na tela
}

void atualizaEstado(int estado) {

    switch(estado){
        case 0:         
            led_r = 21;
            led_g = 40;
            led_b = 4;
            
            bool led_buffer_0 [] = { 
                0, 0, 1, 0, 0, 
                0, 0, 1, 1, 0, 
                1, 0, 1, 1, 0, 
                1, 0, 1, 0, 0, 
                0, 1, 0, 1, 0
            };
            memcpy(led_buffer, led_buffer_0, sizeof(led_buffer));
            atualizaFita(led_r, led_g, led_b);
            break;

        case 1:      
            led_r = 27;
            led_g = 24;
            led_b = 5;
            
            bool led_buffer_1 [NUM_PIXELS] = {
                1, 0, 1, 0, 1, 
                0, 1, 1, 1, 0, 
                1, 1, 0, 1, 1, 
                0, 1, 1, 1, 0, 
                1, 0, 1, 0, 1
            };
            memcpy(led_buffer, led_buffer_1, sizeof(led_buffer));
            atualizaFita(led_r, led_g, led_b);
            break;

        case 2:         
            led_r = 6;
            led_g = 10;
            led_b = 31;

            bool led_buffer_2 [NUM_PIXELS] = {
                0, 1, 1, 1, 0, 
                1, 0, 0, 1, 1, 
                1, 1, 0, 0, 0, 
                0, 0, 0, 1, 1, 
                0, 1, 1, 0, 0
            };
            memcpy(led_buffer, led_buffer_2, sizeof(led_buffer));
            atualizaFita(led_r, led_g, led_b);
            break;
    }
}

void play_tone(uint slice_num, uint freq, uint duty_cycle) {
    uint32_t clock_divider = 4;
    uint32_t wrap = 125000000 / (clock_divider * freq);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, wrap * duty_cycle / 100);
}

void updateSensors(){
    ssd1306_fill(&ssd, !cor); //Limpa o display
    atualizaEstado(estado);

    // Recebe dados do ADC
    adc_select_input(0);
    uint16_t adc_value = adc_read();
    adc_select_input(1);
    joy_x = adc_read();
    
    //Mapeia os dados
    float humidity = map_adc(adc_value);
    float lux = map_adc(joy_x);
    
    // Configura de acordo com o modo utilizado
    switch (estado)
    {
    case 0:
        ssd1306_draw_string(&ssd, "Modo padrao", 5, 15);
        if(humidity <= 35){
            play_tone(slice_buz, 270, 2);
        } else {
            play_tone(slice_buz, 270, 0);
        }
        break;
    case 1:
        ssd1306_draw_string(&ssd, "Modo diurno", 5, 15);
        if(humidity <= 35 && lux >= 25){
            play_tone(slice_buz, 270, 2);
        } else {
            play_tone(slice_buz, 270, 0);
        }
        break;
    case 2:
        ssd1306_draw_string(&ssd, "Modo noturno", 5, 15);
        if(humidity <= 35 && lux <= 25){
            play_tone(slice_buz, 270, 2);
        } else {
            play_tone(slice_buz, 270, 0);
        }
        break;
    }

    print_float_to_oled(&ssd, humidity, 5, 35, 0); //Atualiza a matriz com os dados do ADC
    print_float_to_oled(&ssd, lux, 5, 55, 1);
    ssd1306_send_data(&ssd);

    //Simula um sensor de luminosidade 
    pwm_set_gpio_level(led_pin_red, 2120 - abs((int)joy_x - 2120) * 65535 / 2120);
    pwm_set_gpio_level(led_pin_green, 2120 - abs((int)joy_x - 2120) * 65535 / 2120);
    pwm_set_gpio_level(led_pin_blue, abs((int)joy_x - 2120) * 65535 / 2120);
}

void nivelCritico(){

    while (criticLvl == 1){
        play_tone(slice_buz, 270, 2);
        ssd1306_fill(&ssd, !cor);
        ssd1306_draw_string(&ssd, "AGUA EM NIVEL", 8, 10);
        ssd1306_draw_string(&ssd, "BAIXO", 40, 25);
        ssd1306_draw_string(&ssd, "LIGANDO MOTOR", 8, 40);
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor);
        ssd1306_send_data(&ssd);
                
        led_r = 0;
        led_g = 5;
        led_b = 28;

        bool led_buffer_3 [NUM_PIXELS] = {
            0, 1, 1, 1, 0,
            1, 1, 1, 1, 1, 
            1, 1, 1, 1, 1, 
            0, 1, 1, 1, 0,
            0, 0, 1, 0, 0                 
        };
        memcpy(led_buffer, led_buffer_3, sizeof(led_buffer));
        atualizaFita(led_r, led_g, led_b);
    }
}

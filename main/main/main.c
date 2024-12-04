#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// Configuración del pin del DHT11
#define DHT_PIN 21
#define MAX_RETRIES 5  // Número máximo de reintentos

// Configuración de SPI
#define MISO 4
#define CS   5
#define SCLK 2
#define MOSI 3
#define SPI_PORT spi0

// Variables de compensación para el sensor de presión
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

// Función para esperar en microsegundos
void wait_us(uint32_t us) {
    sleep_us(us);
}

// Función para leer datos del DHT11
bool read_dht11(uint8_t *humidity, uint8_t *temperature) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    gpio_set_dir(DHT_PIN, GPIO_OUT);
    gpio_put(DHT_PIN, 0);
    wait_us(20000);  // Espera 20ms para inicializar el DHT11
    gpio_put(DHT_PIN, 1);
    wait_us(50);
    gpio_set_dir(DHT_PIN, GPIO_IN);

    uint32_t timeout = 0;
    while (gpio_get(DHT_PIN) == 1) {
        if (++timeout > 1000) return false;
    }
    timeout = 0;
    while (gpio_get(DHT_PIN) == 0) {
        if (++timeout > 1000) return false;
    }
    timeout = 0;
    while (gpio_get(DHT_PIN) == 1) {
        if (++timeout > 1000) return false;
    }

    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (gpio_get(DHT_PIN) == 0) {
            if (++timeout > 1000) return false;
        }

        uint32_t start_time = time_us_32();
        timeout = 0;
        while (gpio_get(DHT_PIN) == 1) {
            if (++timeout > 1000) return false;
        }
        uint32_t pulse_duration = time_us_32() - start_time;

        data[i / 8] <<= 1;
        if (pulse_duration > 50) {
            data[i / 8] |= 1;
        }
    }

    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        *humidity = data[0];
        *temperature = data[2];
        return true;
    }

    return false;
}

// Funciones para el sensor BMP280
int32_t compPress(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (int32_t)p;
}

void read_press_comp() {
    uint8_t buffer[24], reg;

    // Leer coeficientes de compensación desde el registro 0x88
    reg = 0x88 | 0x80;  // Registro de compensación con lectura múltiple
    gpio_put(CS, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buffer, 24);
    gpio_put(CS, 1);

    // Asignar coeficientes correctamente
    dig_P1 = buffer[0] | (buffer[1] << 8);
    dig_P2 = buffer[2] | (buffer[3] << 8);
    dig_P3 = buffer[4] | (buffer[5] << 8);
    dig_P4 = buffer[6] | (buffer[7] << 8);
    dig_P5 = buffer[8] | (buffer[9] << 8);
    dig_P6 = buffer[10] | (buffer[11] << 8);
    dig_P7 = buffer[12] | (buffer[13] << 8);
    dig_P8 = buffer[14] | (buffer[15] << 8);
    dig_P9 = buffer[16] | (buffer[17] << 8);
}

void config_bmp280() {
    uint8_t data[2];

    // Configuración del registro 0xF4 (sobremuestreo de presión y temperatura, modo normal)
    data[0] = 0xF4 & 0x7F;  // Indicar escritura
    data[1] = 0x27;  // osrs_p=x1, osrs_t=x1, modo normal
    gpio_put(CS, 0);
    spi_write_blocking(SPI_PORT, data, 2);
    gpio_put(CS, 1);

    // Configuración del registro 0xF5 (filtro y tiempo de standby)
    data[0] = 0xF5 & 0x7F;
    data[1] = 0xA0;  // Filtro = 4, standby = 1000ms
    gpio_put(CS, 0);
    spi_write_blocking(SPI_PORT, data, 2);
    gpio_put(CS, 1);
}

int main() {
    stdio_init_all();
    gpio_init(DHT_PIN);
    gpio_pull_up(DHT_PIN);

    spi_init(SPI_PORT, 500000);
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);

    // Leer coeficientes de compensación y configurar BMP280
    read_press_comp();
    config_bmp280();

    uint8_t humidity, temperature;
    int32_t pressure, rawpress;
    uint8_t reg, buffer[3];

    while (true) {
        printf("Leyendo datos del DHT11...\n");
        bool success = false;
        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
            if (read_dht11(&humidity, &temperature)) {
                success = true;
                break;
            }
            printf("Intento %d fallido, reintentando...\n", attempt + 1);
            sleep_ms(1000);
        }

        if (success) {
            printf("Humedad: %d%%, Temperatura: %d°C\n", humidity, temperature);
        } else {
            printf("Error al leer el sensor DHT11 tras %d intentos.\n", MAX_RETRIES);
        }

        // Leer presión RAW desde el BMP280
        reg = 0xF7 | 0x80;
        gpio_put(CS, 0);
        spi_write_blocking(SPI_PORT, &reg, 1);
        spi_read_blocking(SPI_PORT, 0, buffer, 3);
        gpio_put(CS, 1);

        // Procesar presión RAW y calcular presión compensada
        rawpress = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((uint32_t)buffer[2] >> 4);
        pressure = compPress(rawpress);

        float pressure_hpa = pressure / 256.0 / 100.0;

        // Convertir a hPa y mostrar
        printf("Pressure: %.2f hPa\n", pressure / 256.0);

        sleep_ms(3000);
    }
}

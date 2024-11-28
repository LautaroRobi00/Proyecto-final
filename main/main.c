#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Configuración del pin del DHT11
#define DHT_PIN 21

// Función para esperar en microsegundos
void wait_us(uint32_t us) {
    sleep_us(us);
}

// Función para leer datos del DHT11
bool read_dht11(uint8_t *humidity, uint8_t *temperature) {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Enviar señal de inicio al DHT11
    gpio_set_dir(DHT_PIN, GPIO_OUT);
    gpio_put(DHT_PIN, 0);
    wait_us(18000);  // Espera al menos 18ms para inicializar el DHT11
    gpio_put(DHT_PIN, 1);
    wait_us(40);
    gpio_set_dir(DHT_PIN, GPIO_IN);

    // Leer la respuesta del DHT11
    uint32_t time_count = 0;
    while (gpio_get(DHT_PIN) == 1) {
        if (++time_count > 1000) return false;  // Tiempo de espera agotado
    }

    // Comienza la lectura de datos (40 bits)
    for (int i = 0; i < 40; i++) {
        // Esperar a que comience la señal de datos
        while (gpio_get(DHT_PIN) == 0);
        uint32_t start_time = time_us_32();

        // Medir la duración del pulso alto
        while (gpio_get(DHT_PIN) == 1);
        uint32_t pulse_duration = time_us_32() - start_time;

        // Un pulso mayor a 50us indica un '1'
        data[i / 8] <<= 1;
        if (pulse_duration > 50) {
            data[i / 8] |= 1;
        }
    }

    // Comprobar el checksum
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        *humidity = data[0];
        *temperature = data[2];
        return true;
    }

    return false;
}

int main() {
    stdio_init_all();
    gpio_init(DHT_PIN);
    gpio_pull_up(DHT_PIN);

    uint8_t humidity, temperature;

    while (true) {
        printf("Leyendo datos del DHT11...\n");

        if (read_dht11(&humidity, &temperature)) {
            printf("Humedad: %d%%, Temperatura: %d°C\n", humidity, temperature);
        } else {
            printf("Error al leer el sensor DHT11.\n");
        }

        sleep_ms(2000);  // Esperar 2 segundos antes de la próxima lectura
        

    }
}
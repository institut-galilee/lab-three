#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "bme280.h"

#define DUMMY	-1

#define PIN_NUM_MISO DUMMY
#define PIN_NUM_MOSI DUMMY
#define PIN_NUM_CLK  DUMMY
#define PIN_NUM_CS   DUMMY

int8_t initialize_spi_communication();
int8_t initialize_i2c_communication();
int8_t initialize_bme_device(struct bme280_dev *dev);

/* fonctions de lecture et écriture via SPI */
int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t period);

spi_device_handle_t spi;

int
app_main()
{
	struct bme280_dev dev;
	uint8_t sensor_comp;
	struct bme280_data comp_data;
	int rc;

	// failwith "Students, this is your job!
	// make the right calls in order to get sensor-readings from the bme280
	rc = ENOSYS;

	printf("L'initialisation s'est très bien déroulée\ndev_");

	sensor_comp = 7; //Get all measurements, i.e. temp, hum and pres
	while (1) {
		rc = bme280_get_sensor_data(sensor_comp, &comp_data, &dev);
		if (rc < 0) {
			printf("Error: failled to read data from sensor\n");
			exit(EXIT_FAILURE);
		}

		printf("temperature = %f DegC\n", (float)comp_data.temperature/100);
		printf("humidity = %f %%RH\n", (float)comp_data.humidity/1024);
		printf("pressure = %f Pa\n", (float)comp_data.pressure/256);
	}

	return rc;
}

/* définitions des fonctions */
int8_t
initialize_spi_sensor()
{
    int8_t rc;

    spi_bus_config_t buscfg = {
		// failwith "Students, this is your job!
    };

    spi_device_interface_config_t devcfg={
		// failwith "Students, this is your job!
    };

    //Initialize the SPI bus
	// failwith "Students, this is your job!
    rc = ENOSYS;
    if (rc < 0) {
        printf("[IoT-Labs] Error while initializing SPI bus\n");
        return rc;
    }

    //Attach the device to the SPI bus
	// failwith "Students, this is your job!
    rc = ENOSYS;
    if (rc < 0) {
        printf("[IoT-Labs] Error while adding device to SPI bus\n");
        return rc;
    }

    return rc;
}

int8_t
initialize_i2c_communication()
{
    // failwith "Students, this is your job!
    int8_t rc;
    rc = ENOSYS;
    return rc;
}


int8_t
initialize_bme_device(struct bme280_dev *dev)
{
    int8_t rc;

	// Fill appropriate fields in bme280_dev structure
    dev->intf = BME280_SPI_INTF;
	// ...

    //Initialize the BME280
    rc = bme280_init(dev);
    printf("[initialize_bme_device] rc = %d\n", rc);
    if (rc < 0) {
        printf("[IoT-Labs] Error: bme280 device initialization failled\n");
        return rc;
    }

    return rc;
}

int8_t
read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int rc;
    static struct spi_transaction_t trans;

	// failwith "Students, this is your job!
	// fill spi_transaction_t structure with the right parameters
	// ...

	// failwith "Students, this is your job!
    rc = ENOSYS;
    if (rc < 0) {
        printf("[IoT-Labs] Error: transaction transmission failled\n");
        return rc;
    }

    memcpy(data, trans.rx_buffer, len);

    return rc;
}

int8_t
write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t rc;
    static struct spi_transaction_t trans;

    trans.tx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);
    // FIXME do we need to allocate memory for rx_buffer for write stub?
    trans.rx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);

    trans.flags = 0;
    trans.addr = reg_addr;
    trans.length = len*8;

    memcpy(trans.tx_buffer, data, len);

    rc = spi_device_transmit(spi, &trans);
    if (rc < 0) {
        printf("[IoT-Labs] Error: transaction transmission failled\n");
        return rc;
    }

    return rc;
}

void
delay_ms(uint32_t period)
{
	vTaskDelay(period/portTICK_RATE_MS);
}

#ifndef _EDGE_CONFIG_H_
#define _EDGE_CONFIG_H_
#include <stdint.h>

#define CONF_OF(name) struct name##_conf

/* for device conf */
enum {
	GPIO_ATTR_NUMBER,
	GPIO_ATTR_STATUS,
	__GPIO_ATTR_MAX,
};

enum {
	SERIAL_ATTR_DEVICE,
	SERIAL_ATTR_BAUDRATE,
    SERIAL_ATTR_STATUS,
	__SERIAL_ATTR_MAX,
};

enum {
    SPI_ATTR_DEVICE,
    SPI_ATTR_MODE,
    SPI_ATTR_SPEED,
    SPI_ATTR_MSB,
    SPI_ATTR_DATA_WIDTH,
    SPI_ATTR_STATUS,
	__SPI_ATTR_MAX,
};

/* for edge conf */
enum {
    MODBUS_RTU_ATTR_PROTO,
    MODBUS_RTU_ATTR_DEVICE,
    MODBUS_RTU_ATTR_SLAVE,
    MODBUS_RTU_ATTR_MODE,
    MODBUS_RTU_ATTR_RTS,
    MODBUS_RTU_ATTR_REGISTER,
    MODBUS_RTU_ATTR_RANGE,
    MODBUS_RTU_ATTR_PARITY,
    MODBUS_RTU_ATTR_DATA,
    MODBUS_RTU_ATTR_STOP,
    MODBUS_RTU_ATTR_STATUS,
    __MODBUS_RTU_ATTR_MAX
};

enum {
    LORA_ATTR_SPIDEV,
    LORA_ATTR_GPIO_PWR,
    LORA_ATTR_GPIO_DIO,
    LORA_ATTR_GPIO_RST,
    LORA_ATTR_GPIO_NSS,
    LORA_ATTR_MODEM,
    LORA_ATTR_FREQ,
    LORA_ATTR_BW,
    LORA_ATTR_SF,
    LORA_ATTR_CR,
    LORA_ATTR_NOCRC,
    LORA_ATTR_PRLEN,
    LORA_ATTR_SYNCWORD,
    LORA_ATTR_INVERTIQ,
    LORA_ATTR_POWER,
    LORA_ATTR_STATUS,
    __LORA_ATTR_MAX
};

enum {
    RFID_ATTR_SPIDEV,
    RFID_ATTR_GPIO_DIO,
    RFID_ATTR_GPIO_RST,
    RFID_ATTR_GPIO_NSS,
    RFID_ATTR_FREQ,
    RFID_ATTR_BW,
    RFID_ATTR_SF,
    RFID_ATTR_CR,
    RFID_ATTR_NOCRC,
    RFID_ATTR_PRLEN,
    RFID_ATTR_SYNCWORD,
    RFID_ATTR_INVERTIQ,
    RFID_ATTR_POWER,
    RFID_ATTR_STATUS,
    __RFID_ATTR_MAX
};


struct edge_context;

struct gpio_conf {
    int number;
    char dev[32];
};

struct serial_conf {
    //int (*diff)(struct serial_conf *conf, void *selecter);
    char dev[32];
    int baudrate;
    /*
    int slave;
    int mode;
    int rts;
    int reg[32];
    int nb[32];
    char parity[8];
    int data_bit;
    int stop_bit;
    */
};

struct spi_conf {
    char dev[32];

    int mode;
    int speed;
    int msb;
    int bits_per_word;
};

struct lora_conf {
    char spidev[32];
    char gpio_pwr[32];
    char gpio_rst[32];
    char gpio_dio[32];
    char gpio_nss[32];

    uint32_t freq;
    uint32_t bw;
    uint8_t sf;
    uint8_t cr;
    uint8_t nocrc;
    uint8_t prlen;
    uint8_t syncword;
    uint8_t invertiq;
    uint8_t power;
    uint8_t modem;
    char desc[32];

    int (*diff)(struct serial_conf *conf, void *selecter);

};

struct rfid_conf {
    char spidev[32];
    char gpio_rst[32];
    char gpio_dio[32];
    char gpio_nss[32];

    uint32_t freq;
    uint32_t bw;
    uint8_t sf;
    uint8_t cr;
    uint8_t nocrc;
    uint8_t prlen;
    uint8_t syncword;
    uint8_t invertiq;
    uint8_t power;
    char desc[32];

    int (*diff)(struct serial_conf *conf, void *selecter);

};


struct conf {
    int (*diff)(struct serial_conf *conf, void *selecter);

};
 

int config_init(struct edge_context *edge);
int config_parse_device(void);
int config_parse_edge(void);
#endif

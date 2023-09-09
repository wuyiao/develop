#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <uci.h>
#include <edge.h>
#include <edev.h>
#include <edrv.h>
#include <debug.h>
#include <config.h>

#include <uci_blob.h>
#include <libubox/blobmsg.h>
#include <libubox/blobmsg_json.h>

static struct blob_buf b;
static struct uci_context *uci_ctx;
static struct uci_package *uci_device;
static struct uci_package *uci_edge;

struct list config;
char *config_path = NULL;

static const struct blobmsg_policy gpio_policy[__GPIO_ATTR_MAX] = {
	[GPIO_ATTR_NUMBER] = { .name = "number", .type = BLOBMSG_TYPE_INT32 },
	[GPIO_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

static const struct uci_blob_param_list gpio_param = {
	.n_params = ARRAY_SIZE(gpio_policy),
	.params = gpio_policy,
};

static const struct blobmsg_policy serial_policy[__SERIAL_ATTR_MAX] = {
	[SERIAL_ATTR_DEVICE] = { .name = "device", .type = BLOBMSG_TYPE_STRING },
	[SERIAL_ATTR_BAUDRATE] = { .name = "baudrate", .type = BLOBMSG_TYPE_INT32 },
	[SERIAL_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

// static const struct uci_blob_param_info serial_info[__SERIAL_ATTR_MAX] = {
    // [SERIAL_ATTR_REGISTER] = { .type = BLOBMSG_TYPE_INT32 },
    // [SERIAL_ATTR_RANGE] = { .type = BLOBMSG_TYPE_INT32 },
// };

static const struct uci_blob_param_list serial_param = {
	.n_params = ARRAY_SIZE(serial_policy),
	.params = serial_policy,
};

static const struct blobmsg_policy spi_policy[__SPI_ATTR_MAX] = {
	[SPI_ATTR_DEVICE] = { .name = "device", .type = BLOBMSG_TYPE_STRING },
	[SPI_ATTR_MODE] = { .name = "mode", .type = BLOBMSG_TYPE_INT32 },
	[SPI_ATTR_SPEED] = { .name = "speed", .type = BLOBMSG_TYPE_INT32 },
	[SPI_ATTR_MSB] = { .name = "msb", .type = BLOBMSG_TYPE_INT32 },
	[SPI_ATTR_DATA_WIDTH] = { .name = "bits_per_word", .type = BLOBMSG_TYPE_INT32 },
	[SPI_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

static const struct uci_blob_param_list spi_param = {
	.n_params = ARRAY_SIZE(spi_policy),
	.params = spi_policy,
};

static const struct blobmsg_policy modbus_rtu_policy[__MODBUS_RTU_ATTR_MAX] = {
	[MODBUS_RTU_ATTR_PROTO] = { .name = "proto", .type = BLOBMSG_TYPE_STRING },
	[MODBUS_RTU_ATTR_DEVICE] = { .name = "device", .type = BLOBMSG_TYPE_ARRAY },
	[MODBUS_RTU_ATTR_SLAVE] = { .name = "slave", .type = BLOBMSG_TYPE_INT32 },
	[MODBUS_RTU_ATTR_MODE] = { .name = "mode", .type = BLOBMSG_TYPE_INT32 },
	[MODBUS_RTU_ATTR_RTS] = { .name = "rts", .type = BLOBMSG_TYPE_INT32 },
	[MODBUS_RTU_ATTR_REGISTER] = { .name = "register", .type = BLOBMSG_TYPE_ARRAY },
	[MODBUS_RTU_ATTR_RANGE] = { .name = "range", .type = BLOBMSG_TYPE_ARRAY },
	[MODBUS_RTU_ATTR_PARITY] = { .name = "parity", .type = BLOBMSG_TYPE_STRING },
	[MODBUS_RTU_ATTR_DATA] = { .name = "data_bit", .type = BLOBMSG_TYPE_INT32 },
	[MODBUS_RTU_ATTR_STOP] = { .name = "stop_bit", .type = BLOBMSG_TYPE_INT32 },
	[MODBUS_RTU_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

static const struct uci_blob_param_list modbus_rtu_param = {
	.n_params = ARRAY_SIZE(modbus_rtu_policy),
	.params = modbus_rtu_policy,
};

static const struct blobmsg_policy lora_policy[__LORA_ATTR_MAX] = {
	[LORA_ATTR_SPIDEV] = { .name = "spidev", .type = BLOBMSG_TYPE_STRING },
	[LORA_ATTR_GPIO_PWR] = { .name = "gpio_pwr", .type = BLOBMSG_TYPE_STRING },
	[LORA_ATTR_GPIO_DIO] = { .name = "gpio_dio", .type = BLOBMSG_TYPE_STRING },
	[LORA_ATTR_GPIO_RST] = { .name = "gpio_rst", .type = BLOBMSG_TYPE_STRING },
	[LORA_ATTR_GPIO_NSS] = { .name = "gpio_nss", .type = BLOBMSG_TYPE_STRING },
	[LORA_ATTR_MODEM] = { .name = "modem", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_FREQ] = { .name = "freq", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_BW] = { .name = "bw", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_SF] = { .name = "sf", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_CR] = { .name = "cr", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_NOCRC] = { .name = "nocrc", .type = BLOBMSG_TYPE_BOOL },
	[LORA_ATTR_PRLEN] = { .name = "prlen", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_SYNCWORD] = { .name = "syncword", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_INVERTIQ] = { .name = "invertiq", .type = BLOBMSG_TYPE_BOOL },
	[LORA_ATTR_POWER] = { .name = "power", .type = BLOBMSG_TYPE_INT32 },
	[LORA_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

static const struct uci_blob_param_list lora_param = {
	.n_params = ARRAY_SIZE(lora_policy),
	.params = lora_policy,
};

static const struct blobmsg_policy rfid_policy[__RFID_ATTR_MAX] = {
	[RFID_ATTR_SPIDEV] = { .name = "spidev", .type = BLOBMSG_TYPE_STRING },
	[RFID_ATTR_GPIO_DIO] = { .name = "gpio_dio", .type = BLOBMSG_TYPE_STRING },
	[RFID_ATTR_GPIO_RST] = { .name = "gpio_rst", .type = BLOBMSG_TYPE_STRING },
	[RFID_ATTR_GPIO_NSS] = { .name = "gpio_nss", .type = BLOBMSG_TYPE_STRING },
	[RFID_ATTR_FREQ] = { .name = "freq", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_BW] = { .name = "bw", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_SF] = { .name = "sf", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_CR] = { .name = "cr", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_NOCRC] = { .name = "nocrc", .type = BLOBMSG_TYPE_BOOL },
	[RFID_ATTR_PRLEN] = { .name = "prlen", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_SYNCWORD] = { .name = "syncword", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_INVERTIQ] = { .name = "invertiq", .type = BLOBMSG_TYPE_BOOL },
	[RFID_ATTR_POWER] = { .name = "power", .type = BLOBMSG_TYPE_INT32 },
	[RFID_ATTR_STATUS] = { .name = "disabled", .type = BLOBMSG_TYPE_BOOL },
};

static const struct uci_blob_param_list rfid_param = {
	.n_params = ARRAY_SIZE(rfid_policy),
	.params = rfid_policy,
};


static struct uci_package *config_init_package(const char *config)
{
	struct uci_context *ctx = uci_ctx;
	struct uci_package *p = NULL;

	if (!ctx) {
		ctx = uci_alloc_context();
		uci_ctx = ctx;

		ctx->flags &= ~UCI_FLAG_STRICT;
		if (config_path)
			uci_set_confdir(ctx, config_path);

#ifdef DUMMY_MODE
		uci_set_savedir(ctx, "./tmp");
#endif
	}
    else {
		p = uci_lookup_package(ctx, config);
		if (p)
			uci_unload(ctx, p);
	}

	if (uci_load(ctx, config, &p))
		return NULL;

	return p;
}

void config_parse_gpio(struct uci_section *s)
{
    struct blob_attr *tb[__GPIO_ATTR_MAX];
	//struct blob_attr *cur;
    struct gpio_conf *conf;
    const char *status;

    status = uci_lookup_option_string(uci_ctx, s, "disabled");
    /* enabled if disabled field is not set */
	if (status) {
        if (status[0] == '1')
            return;
    }
    
    blob_buf_init(&b, 0);
	uci_to_blob(&b, s, &gpio_param);

    blobmsg_parse(gpio_policy, __GPIO_ATTR_MAX, tb, blob_data(b.head), blob_len(b.head));

    conf = calloc(1, sizeof(struct gpio_conf));
    if (!conf) {
        ULOG_ERR("calloc err\n");
        return;
    }

    conf->number = blobmsg_get_u32(tb[GPIO_ATTR_NUMBER]); 
    ULOG_DEBUG("gpio: %d\n", conf->number);
    sprintf(conf->dev, "gpio%d", conf->number);

    blob_buf_free(&b);
	edev_create_dev(GPIO, conf->dev, conf);
}

void config_parse_serial(struct uci_section *s)
{
    struct blob_attr *tb[__SERIAL_ATTR_MAX];
    struct serial_conf *conf;
    const char *status;

    status = uci_lookup_option_string(uci_ctx, s, "disabled");
	if (status) {
        if (status[0] == '1')
            return;
    }
    
    blob_buf_init(&b, 0);
	uci_to_blob(&b, s, &serial_param);
    blobmsg_parse(serial_policy, __SERIAL_ATTR_MAX, tb, blob_data(b.head), blob_len(b.head));

    conf = calloc(1, sizeof(struct serial_conf));
    if (!conf) {
        ULOG_ERR("calloc err\n");
        return;
    }

    if (tb[SERIAL_ATTR_DEVICE]) {
        strncpy(conf->dev, blobmsg_get_string(tb[SERIAL_ATTR_DEVICE]), sizeof(conf->dev)); 
        ULOG_DEBUG("serialdev: %s\n", conf->dev);
    }

    if (tb[SERIAL_ATTR_BAUDRATE]) {
        conf->baudrate = blobmsg_get_u32(tb[SERIAL_ATTR_BAUDRATE]);
        ULOG_DEBUG("baudrate: %d\n", conf->baudrate);
    }

    blob_buf_free(&b);
	edev_create_dev(SERIAL, conf->dev, conf);
}

void config_parse_spi(struct uci_section *s)
{
    struct blob_attr *tb[__SPI_ATTR_MAX];
    struct spi_conf *conf;
    const char *status;

    status = uci_lookup_option_string(uci_ctx, s, "disabled");
	if (status) {
        if (status[0] == '1')
            return;
    }
    
    blob_buf_init(&b, 0);
	uci_to_blob(&b, s, &spi_param);
    blobmsg_parse(spi_policy, __SPI_ATTR_MAX, tb, blob_data(b.head), blob_len(b.head));

    conf = calloc(1, sizeof(struct spi_conf));
    if (!conf) {
        ULOG_ERR("calloc err\n");
        return;
    }

    if (tb[SPI_ATTR_DEVICE]) {
        strncpy(conf->dev, blobmsg_get_string(tb[SPI_ATTR_DEVICE]), sizeof(conf->dev)); 
        ULOG_DEBUG("spidev: %s\n", conf->dev);
    }

    if (tb[SPI_ATTR_MODE]) {
        conf->mode = blobmsg_get_u32(tb[SPI_ATTR_MODE]);
        ULOG_DEBUG("mode: %d\n", conf->mode);
    }

    if (tb[SPI_ATTR_SPEED]) {
        conf->speed = blobmsg_get_u32(tb[SPI_ATTR_SPEED]);
        ULOG_DEBUG("speed: %d\n", conf->speed);
    }

    if (tb[SPI_ATTR_MSB]) {
        conf->msb = blobmsg_get_u32(tb[SPI_ATTR_MSB]);
        ULOG_DEBUG("msb: %d\n", conf->msb);
    }

    if (tb[SPI_ATTR_DATA_WIDTH]) {
        conf->bits_per_word = blobmsg_get_u32(tb[SPI_ATTR_DATA_WIDTH]);
        ULOG_DEBUG("bits_per_word: %d\n", conf->bits_per_word);
    }
    blob_buf_free(&b);
	edev_create_dev(SPI, conf->dev, conf);
}

int config_parse_device(void)
{
    struct uci_element *e;

	uci_foreach_element(&uci_device->sections, e) {
		struct uci_section *s = uci_to_section(e);
		//const char *type, *name;

        ULOG_DEBUG("section: %s\n", s->type);
        switch (edev_get_type(s->type)) {
            case GPIO:
                config_parse_gpio(s);
                break;
            case SERIAL:
                config_parse_serial(s);
                break;
            case SPI:
                config_parse_spi(s);
                break;
            case UNKNOWN:
            default:
                break;
        }
	}

    return 0;
}

void config_parse_sx127x(struct uci_section *s)
{
    struct blob_attr *tb[__LORA_ATTR_MAX];
    struct lora_conf *conf = NULL;
    const char *status;

    status = uci_lookup_option_string(uci_ctx, s, "disabled");
	if (status) {
        if (status[0] == '1')
            return;
    }
    
    blob_buf_init(&b, 0);
	uci_to_blob(&b, s, &lora_param);
    blobmsg_parse(lora_policy, __LORA_ATTR_MAX, tb, blob_data(b.head), blob_len(b.head));

    struct edge_driver *drv = edrv_get_drv(s->type);
    if (drv) {
        conf = drv->conf;
    }
    if (!conf) {
        ULOG_ERR("lora conf is null\n");
        return;
    }

    if (tb[LORA_ATTR_SPIDEV]) {
        strncpy(conf->spidev, blobmsg_get_string(tb[LORA_ATTR_SPIDEV]), sizeof(conf->spidev)); 
        ULOG_DEBUG("spidev: %s\n", conf->spidev);
    }
    if (tb[LORA_ATTR_GPIO_PWR]) {
        strncpy(conf->gpio_pwr, blobmsg_get_string(tb[LORA_ATTR_GPIO_PWR]), sizeof(conf->gpio_pwr)); 
        ULOG_DEBUG("gpio_pwr: %s\n", conf->gpio_pwr);
    }
    if (tb[LORA_ATTR_GPIO_RST]) {
        strncpy(conf->gpio_rst, blobmsg_get_string(tb[LORA_ATTR_GPIO_RST]), sizeof(conf->gpio_rst)); 
        ULOG_DEBUG("gpio_rst: %s\n", conf->gpio_rst);
    }
    if (tb[LORA_ATTR_GPIO_DIO]) {
        strncpy(conf->gpio_dio, blobmsg_get_string(tb[LORA_ATTR_GPIO_DIO]), sizeof(conf->gpio_dio)); 
        ULOG_DEBUG("gpio_dio: %s\n", conf->gpio_dio);
    }
    if (tb[LORA_ATTR_GPIO_NSS]) {
        strncpy(conf->gpio_nss, blobmsg_get_string(tb[LORA_ATTR_GPIO_NSS]), sizeof(conf->gpio_nss)); 
        ULOG_DEBUG("gpio_nss: %s\n", conf->gpio_nss);
    }

    if (tb[LORA_ATTR_MODEM]) {
        conf->modem = blobmsg_get_u32(tb[LORA_ATTR_MODEM]); 
        ULOG_DEBUG("modem: %d\n", conf->modem);
    }

    if (tb[LORA_ATTR_FREQ]) {
        conf->freq = blobmsg_get_u32(tb[LORA_ATTR_FREQ]); 
        ULOG_DEBUG("freq: %d\n", conf->freq);
    }

    if (tb[LORA_ATTR_BW]) {
        conf->bw = blobmsg_get_u32(tb[LORA_ATTR_BW]); 
        ULOG_DEBUG("bw: %d\n", conf->bw);
    }

    if (tb[LORA_ATTR_SF]) {
        conf->sf = blobmsg_get_u32(tb[LORA_ATTR_SF]); 
        ULOG_DEBUG("sf: %d\n", conf->sf);
    }

    if (tb[LORA_ATTR_CR]) {
        conf->cr = blobmsg_get_u32(tb[LORA_ATTR_CR]); 
        ULOG_DEBUG("cr: %d\n", conf->cr);
    }

    if (tb[LORA_ATTR_NOCRC]) {
        conf->nocrc = blobmsg_get_bool(tb[LORA_ATTR_NOCRC]); 
        ULOG_DEBUG("nocrc: %d\n", conf->nocrc);
    }

    if (tb[LORA_ATTR_PRLEN]) {
        conf->prlen = blobmsg_get_u32(tb[LORA_ATTR_PRLEN]); 
        ULOG_DEBUG("prlen: %d\n", conf->prlen);
    }

    if (tb[LORA_ATTR_SYNCWORD]) {
        conf->syncword = blobmsg_get_u32(tb[LORA_ATTR_SYNCWORD]); 
        ULOG_DEBUG("syncword: %d\n", conf->syncword);
    }

    if (tb[LORA_ATTR_INVERTIQ]) {
        conf->invertiq = blobmsg_get_bool(tb[LORA_ATTR_INVERTIQ]); 
        ULOG_DEBUG("invertiq: %d\n", conf->invertiq);
    }

    if (tb[LORA_ATTR_POWER]) {
        conf->power = blobmsg_get_u32(tb[LORA_ATTR_POWER]); 
        ULOG_DEBUG("power: %d\n", conf->power);
    }
    blob_buf_free(&b);
}

void config_parse_rfid(struct uci_section *s)
{
    struct blob_attr *tb[__RFID_ATTR_MAX];
    struct rfid_conf *conf = NULL;
    const char *status;

    status = uci_lookup_option_string(uci_ctx, s, "disabled");
	if (status) {
        if (status[0] == '1')
            return;
    }
    
    blob_buf_init(&b, 0);
	uci_to_blob(&b, s, &rfid_param);
    blobmsg_parse(rfid_policy, __RFID_ATTR_MAX, tb, blob_data(b.head), blob_len(b.head));

    struct edge_driver *drv = edrv_get_drv(s->type);
    if (drv) {
        conf = drv->conf;
    }
    if (!conf) {
        ULOG_ERR("rfid conf is null\n");
        return;
    }

    if (tb[RFID_ATTR_SPIDEV]) {
        strncpy(conf->spidev, blobmsg_get_string(tb[RFID_ATTR_SPIDEV]), sizeof(conf->spidev)); 
        ULOG_DEBUG("spidev: %s\n", conf->spidev);
    }
    if (tb[RFID_ATTR_GPIO_RST]) {
        strncpy(conf->gpio_rst, blobmsg_get_string(tb[RFID_ATTR_GPIO_RST]), sizeof(conf->gpio_rst)); 
        ULOG_DEBUG("gpio_rst: %s\n", conf->gpio_rst);
    }
    blob_buf_free(&b);
}


int config_parse_edge()
{
    struct uci_element *e;

	uci_foreach_element(&uci_edge->sections, e) {
		struct uci_section *s = uci_to_section(e);

		if (strcmp(s->type, "sx127x") == 0)
            config_parse_sx127x(s);
		if (strcmp(s->type, "rfid") == 0)
            config_parse_rfid(s);
	}

    return 0;
}

int config_init(struct edge_context *edge)
{
	int ret = 0;
	char *err;

	uci_device = config_init_package("device");
	if (!uci_device) {
		uci_get_errorstr(uci_ctx, &err, NULL);
		ULOG_ERR("Failed to load device config (%s)\n", err);
		free(err);
		return -1;
	}

	uci_edge = config_init_package("edge");
	if (!uci_edge) {
		uci_get_errorstr(uci_ctx, &err, NULL);
		ULOG_ERR("Failed to load edge config (%s)\n", err);
		free(err);
		return -1;
	}

    edge->uci_ctx = uci_ctx;
    edge->uci_edge = uci_edge; 
    edge->uci_device = uci_device; 

    ULOG_DEBUG("configuration initialization successful\n");

	return ret;
}

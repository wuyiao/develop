#include <edge.h>
#include <edev.h>
#include <edrv.h>
#include <adapter.h>
#include <config.h>
#include <sx127x.h>
#include <sx127x-board.h>
#include <radio.h>
#include <libubox/runqueue.h>
#include <linux/spi/spidev.h>
#include <thread.h>
#include <debug.h>

#define DRIVER_NAME "sx127x"
static struct edge_driver driver;
static struct edge_device *spi_dev, *gpio_pwr, *gpio_rst, *gpio_dio;
static struct lora_conf conf;

//static struct radio_operations radio_ops;
const struct Radio_s Radio = {
    SX127xInit,
    SX127xProbe,
    SX127xReset,
    SX127xGetStatus,
    SX127xSetModem,
    SX127xSetChannel,
    SX127xIsChannelFree,
    SX127xRandom,
    SX127xSetRxConfig,
    SX127xSetTxConfig,
    SX127xCheckRfFrequency,
    SX127xGetTimeOnAir,
    SX127xSend,
    SX127xAsyncSend,
    SX127xSetSleep,
    SX127xSetStby,
    SX127xSetRx,
    SX127xStartCad,
    SX127xSetTxContinuousWave,
    SX127xReadRssi,
    SX127xWrite,
    SX127xRead,
    SX127xWriteBuffer,
    SX127xReadBuffer,
    SX127xSetMaxPayloadLength,
    SX127xSetPublicNetwork,
    SX127xGetWakeupTime,
    //NULL, // void ( *IrqProcess )( void )
    //NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    //NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

static int init(struct edge_driver *drv)
{
    /* init driver data */
    struct lora_conf *conf = NULL;

    conf = drv->conf;
    spi_dev = edev_get_dev(conf->spidev);
    gpio_pwr = edev_get_dev(conf->gpio_pwr);
    gpio_rst = edev_get_dev(conf->gpio_rst);
    gpio_dio = edev_get_dev(conf->gpio_dio);
    //gpio_nss = edev_get_dev(conf->gpio_nss);

// -------------------------------------------------------------------------
    // gpio_m1 = edev_get_dev(conf->gpio_m1);
    // gpio_m0 = edev_get_dev(conf->gpio_m0);

    // struct edge_adapter *ada1 = edev_get_adapter(gpio_m1);
    // struct gpio_operations *ops1 = ada1->ops;
    // ops1->reserve(gpio_m1);
    // ops1->set_direction(gpio_m1, GPIO_OUT);
    // ops1->set_state(gpio_m1, HIGH);

    // struct edge_adapter *ada2 = edev_get_adapter(gpio_m0);
    // struct gpio_operations *ops2 = ada2->ops;
    // ops2->reserve(gpio_m0);
    // ops2->set_direction(gpio_m0, GPIO_OUT);
    // ops2->set_state(gpio_m0, LOW);
// -------------------------------------------------------------------------

    struct edge_adapter *ada = edev_get_adapter(gpio_pwr);
    struct gpio_operations *ops = ada->ops;
    ops->reserve(gpio_pwr);
    ops->set_direction(gpio_pwr, GPIO_OUT);
    ops->set_state(gpio_pwr, HIGH);

    /* dev associate to drv */
    edev_set_drv(spi_dev, drv);
    edev_set_drv(gpio_rst, drv);
    edev_set_drv(gpio_dio, drv);

    /* driver associate to device */
    edrv_set_main_dev(drv, spi_dev);

    SX127xIoInit(drv);

    /* scheduler */
    memset(&SX127x.S, 0, sizeof(SX127x.S));
    runqueue_init(&SX127x.S.q);
	SX127x.S.q.empty_cb = thread_q_done;
	SX127x.S.q.max_running_tasks = 1;

    //GPIO(rst).reserve(GPIO_RST);
    //GPIO(dio).reserve(GPIO_DIO);
    //GPIO(dio).set_direction(GPIO_DIO, 0);
    //GPIO(dio).set_edge(GPIO_DIO);
    //GPIO(rst).open(GPIO_RST);
    //GPIO(dio).open(GPIO_DIO);
    //SPI.open(SPIDEV);

    return 0;
}

static int fini(struct edge_driver *drv)
{
    SX127xIoDeInit(drv);
    //GPIO(rst).close(GPIO_RST);
    //GPIO(dio).close(GPIO_DIO);
    //GPIO(rst).release(GPIO_RST);
    //GPIO(dio).release(GPIO_DIO);
    //SPI.close(SPIDEV);

    return 0;
}

DRIVER_DECLARE(sx127x)
{
    //memset((void *)&Radio, 0, sizeof(Radio));
    memset(&driver, 0, sizeof(driver));

    driver.init = init;
    driver.fini = fini;
    driver.conf = &conf;
    driver.ops = (struct Radio_s *)&Radio;
    strcpy(driver.name, DRIVER_NAME); 

    edrv_register(&driver);
    if (DEBUG(drv, DRV))
        ULOG_DEBUG("(2) initialize sx127x driver");
}


/*!
 * \file      sx126x-board.c
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    xuzhou ( ei.link )
 */
#include <edge.h>
#include <edev.h>
#include <edrv.h>
#include <config.h>
#include <adapter.h>
#include <libubox/runqueue.h>
#include <linux/spi/spidev.h>
#include <radio.h>
#include <sx126x-board.h>


#if defined( USE_RADIO_DEBUG )
/*!
 * \brief Writes new Tx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinTxWrite( uint8_t state );

/*!
 * \brief Writes new Rx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinRxWrite( uint8_t state );
#endif

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

static struct uloop_fd gpio_irq;
static struct runqueue q;
static struct runqueue_task_type type[2];
static struct runqueue_task task[2];
/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntPow;
Gpio_t DeviceSel;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX126xIoInit( void )
{
    struct lora_conf *conf;
    struct gpio_operations *gpio_ops;
    struct spi_operations *spi_ops;

    conf = drv->conf;
    SX126x.Reset = edev_get_dev(conf->gpio_rst);
    SX126x.DIO1 = edev_get_dev(conf->gpio_dio);
    SX126x.Spi = edev_get_dev(conf->spidev);

    gpio_ops = SX126x.Reset->adapter->ops;
    spi_ops = SX126x.Spi->adapter->ops;

    gpio_ops->reserve(SX126x.Reset);
    gpio_ops->reserve(SX126x.DIO1);
    gpio_ops->open(SX126x.DIO1);
    gpio_ops->set_edge(SX126x.DIO1, "rising");
    spi_ops->open(SX126x.Spi);
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    char buf[32] = {0};
    memset(&gpio_irq, 0, sizeof(gpio_irq));

    gpio_irq.fd = SX126x.DIO1->fd[0];
    gpio_irq.cb = irqHandlers[0];

    /* consume any prior interrupt */
    lseek(gpio_irq.fd, 0, SEEK_SET);
    read(gpio_irq.fd, buf, sizeof buf);

    uloop_fd_add(&gpio_irq, ULOOP_READPRI | ULOOP_ERROR_CB);
}

void SX126xIoDeInit( void )
{
    struct lora_conf *conf;
    struct gpio_operations *gpio_ops;
    struct spi_operations *spi_ops;

    conf = drv->conf;
    SX126x.Reset = edev_get_dev(conf->gpio_rst);
    SX126x.DIO0 = edev_get_dev(conf->gpio_dio);
    SX126x.Spi = edev_get_dev(conf->spidev);

    gpio_ops = SX126x.Reset->adapter->ops;
    spi_ops = SX126x.Spi->adapter->ops;

    gpio_ops->close(SX126x.DIO1);
    spi_ops->close(SX126x.Spi);
    gpio_ops->release(SX126x.DIO1);
    gpio_ops->release(SX126x.Reset);
}

int SX126xProbe(void)
{
    uint8_t version = 0;
    struct spi_operations *spi_ops = SX126x.Spi->adapter->ops;

    //SX127xSetOpMode(RF_OPMODE_SLEEP);
    spi_ops->readb(SX126x.Spi, REG_VERSION, &version);

    if (version == 0x12) {
        ULOG_INFO("%s: SX1268 detected, starting.", SX126x.Spi->name);
        return true;
    }
    else {
        ULOG_ERR("%s: SX1268 Not Detected. ver(%d)", SX126x.Spi->name, version);
        return false;
    }
}

void SX126xIoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX126xIoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xIoRfSwitchInit( void )
{
    //SX126xSetDio2AsRfSwitchCtrl( true );
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
#if defined( USE_RADIO_DEBUG )
    switch( mode )
    {
        case MODE_TX:
            SX126xDbgPinTxWrite( 1 );
            SX126xDbgPinRxWrite( 0 );
            break;
        case MODE_RX:
        case MODE_RX_DC:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 1 );
            break;
        default:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 0 );
            break;
    }
#endif
}

void type_cancel(struct runqueue *q, struct runqueue_task *t, int type)
{
    runqueue_task_complete(t);
}

static void q_done(struct runqueue *q)
{
}

static void q_complete(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX126x.Reset->adapter->ops;

    void (*callback)(void) = SX126x.Reset->data;
    if (callback)
        callback();

    /* configure as input */
    gpio_ops->set_direction(SX126x.Reset, 0);
    //gpio_ops->release(SX127x.Reset);
    SX126x.Reset->data = NULL;
	ULOG_INFO("Radio SX126x Reset Done!");
}

static void radio_reset_second_stage(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX126x.Reset->adapter->ops;
    gpio_ops->digital_write(SX126x.Reset, HIGH);
}

static void radio_reset_first_stage(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX126x.Reset->adapter->ops;
    gpio_ops->digital_write(SX126x.Reset, LOW);
}


void SX126xReset( void )
{
    int i;

    runqueue_init(&q);
	q.empty_cb = q_done;
	q.max_running_tasks = 1;
    SX126x.Reset->data = callback;

    for (i = 0; i < 2; i++) {
        memset(&type[i], 0, sizeof(struct runqueue_task_type));
        memset(&task[i], 0, sizeof(struct runqueue_task));
    }

    type[0].name = "stage-1";
    type[0].run = radio_reset_first_stage;
    type[0].cancel = type_cancel;
    type[0].kill = NULL;

    type[1].name = "stage-2";
    type[1].run = radio_reset_second_stage;
    type[1].cancel = type_cancel;
    type[1].kill = NULL;

    task[0].type = &type[0];
    task[0].run_timeout = 20;
    task[0].complete = NULL;

    task[1].type = &type[1];
    task[1].run_timeout = 10;
    task[1].complete = q_complete;

    runqueue_task_add_first(&q, &task[0], false);
    runqueue_task_add(&q, &task[1], false);
}

void SX126xWaitOnBusy( void )
{
    //while( GpioRead( &SX126x.BUSY ) == 1 );
}

void SX126xWakeup( void )
{
    //CRITICAL_SECTION_BEGIN( );
    uint8_t msg[2] = { RADIO_GET_STATUS, 0x00 };
    struct spi_operations *spi_ops = SX126x.Spi->adapter->ops;

    //SX127xSetOpMode(RF_OPMODE_SLEEP);
    spi_ops->transfer(SX126x.Spi, msg, NULL, 2);

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    // Update operating mode context variable
    SX126xSetOperatingMode( MODE_STDBY_RC );

    //CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    struct spi_operations *spi_ops = SX126x.Spi->adapter->ops;

    SX126xCheckDeviceReady( );

    spi_ops->write(SX126x.Spi, (uint8_t)command, buffer, size);

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint8_t status = 0;
    struct spi_operations *spi_ops = SX126x.Spi->adapter->ops;

    SX126xCheckDeviceReady( );

    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, ( uint8_t )command );
    status = SpiInOut( &SX126x.Spi, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }

    GpioWrite( &SX126x.Spi.Nss, 1 );
    spi_ops->read(SX126x.Spi, (uint8_t)command, buffer, size);

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    GpioWrite( &SX126x.Spi.Nss, 0 );
    
    SpiInOut( &SX126x.Spi, RADIO_WRITE_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }

    GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_READ_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_WRITE_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_READ_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    if( GpioRead( &DeviceSel ) == 1 )
    {
        return SX1261;
    }
    else
    {
        return SX1262;
    }
}

void SX126xAntSwOn( void )
{
    GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX126xAntSwOff( void )
{
    GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX126xGetDio1PinState( void )
{
    return GpioRead( &SX126x.DIO1 );
}

#if defined( USE_RADIO_DEBUG )
static void SX126xDbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

static void SX126xDbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif

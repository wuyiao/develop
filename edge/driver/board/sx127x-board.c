/*!
 * \file      sx127x-board.c
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
#include <thread.h>
#include <sx127x-board.h>
#include "sx127xRegs-Fsk.h"
#include "sx127xRegs-LoRa.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX127xGetPaSelect( int8_t power );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

static struct uloop_fd gpio_irq;
static struct runqueue_task_type type[2];
static struct runqueue_task task[2];

/*!
 * TCXO power control pin
 */
Gpio_t TcxoPower;

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchRx;
Gpio_t AntSwitchTxBoost;
Gpio_t AntSwitchTxRfo;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif


void SX127xIoInit( struct edge_driver *drv )
{
    struct lora_conf *conf;
    struct gpio_operations *gpio_ops;
    struct spi_operations *spi_ops;

    conf = drv->conf;
    SX127x.Reset = edev_get_dev(conf->gpio_rst);
    SX127x.DIO0 = edev_get_dev(conf->gpio_dio);
    //SX127x.NSS = edev_get_dev(conf->gpio_nss);
    SX127x.Spi = edev_get_dev(conf->spidev);

    gpio_ops = SX127x.Reset->adapter->ops;
    spi_ops = SX127x.Spi->adapter->ops;

    gpio_ops->reserve(SX127x.Reset);
    gpio_ops->reserve(SX127x.DIO0);
    //gpio_ops->reserve(SX127x.NSS);
    gpio_ops->open(SX127x.DIO0);
    //gpio_ops->open(SX127x.NSS);
    //gpio_ops->set_direction(SX127x.NSS, 1);
    gpio_ops->set_edge(SX127x.DIO0, "rising");
    spi_ops->open(SX127x.Spi);
}

void SX127xIoIrqInit( DioIrqHandler **irqHandlers )
{
    char buf[32] = {0};
    memset(&gpio_irq, 0, sizeof(gpio_irq));

    gpio_irq.fd = SX127x.DIO0->fd[0];
    gpio_irq.cb = irqHandlers[0];

    /* consume any prior interrupt */
    lseek(gpio_irq.fd, 0, SEEK_SET);
    read(gpio_irq.fd, buf, sizeof buf);

    uloop_fd_add(&gpio_irq, ULOOP_READPRI | ULOOP_ERROR_CB);
}

void SX127xIoDeInit( struct edge_driver *drv )
{
    struct lora_conf *conf;
    struct gpio_operations *gpio_ops;
    struct spi_operations *spi_ops;

    conf = drv->conf;
    SX127x.Reset = edev_get_dev(conf->gpio_rst);
    SX127x.DIO0 = edev_get_dev(conf->gpio_dio);
    SX127x.Spi = edev_get_dev(conf->spidev);

    gpio_ops = SX127x.Reset->adapter->ops;
    spi_ops = SX127x.Spi->adapter->ops;

    spi_ops->close(SX127x.Spi);
    gpio_ops->close(SX127x.DIO0);
    gpio_ops->release(SX127x.DIO0);
    gpio_ops->release(SX127x.Reset);
}

int SX127xProbe(void)
{
    uint8_t version = 0;
    struct spi_operations *spi_ops = SX127x.Spi->adapter->ops;

    //SX127xSetOpMode(RF_OPMODE_SLEEP);
    spi_ops->readb(SX127x.Spi, REG_VERSION, &version);

    if (version == 0x12) {
        //ULOG_INFO("%s: SX1278 detected, starting.", SX127x.Spi->name);
        return true;
    }
    else {
        //ULOG_ERR("%s: SX1278 Not Detected. ver(%d)", SX127x.Spi->name, version);
        return false;
    }
}

void SX127xIoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
#endif
}

void SX127xIoTcxoInit( void )
{
}

void SX127xSetBoardTcxo( uint8_t state )
{
#if 0
    if( state == true )
    {
        if( GpioRead( &TcxoPower ) == 0 )
        { // TCXO OFF power it up.
            // Power ON the TCXO
            GpioWrite( &TcxoPower, 1 );
            DelayMs( BOARD_TCXO_WAKEUP_TIME );
        }
    }
    else
    {
        // Power OFF the TCXO
        GpioWrite( &TcxoPower, 0 );
    }
#endif
}

uint32_t SX127xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

// static void type_cancel(struct runqueue *q, struct runqueue_task *t, int type)
// {
    // runqueue_task_complete(t);
// }
//
// static void q_done(struct runqueue *q)
// {
// }

static void SX127xResetComplete(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX127x.Reset->adapter->ops;

    void (*callback)(void) = SX127x.Reset->data;
    if (callback)
        callback();

    /* configure as input */
    gpio_ops->set_direction(SX127x.Reset, 0);
    //gpio_ops->release(SX127x.Reset);
    SX127x.Reset->data = NULL;
}

static void SX127xResetSecondStage(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX127x.Reset->adapter->ops;
    gpio_ops->digital_write(SX127x.Reset, HIGH);
}

static void SX127xResetFirstStage(struct runqueue *q, struct runqueue_task *t)
{
    struct gpio_operations *gpio_ops = SX127x.Reset->adapter->ops;
    gpio_ops->digital_write(SX127x.Reset, LOW);
}

void SX127xReset(void (*callback)(void))
{
    int i;

    SX127x.Reset->data = callback;

    for (i = 0; i < 2; i++) {
        memset(&type[i], 0, sizeof(struct runqueue_task_type));
        memset(&task[i], 0, sizeof(struct runqueue_task));
    }

    /* dont use SX127x's Type and Task */
    type[0].name = "stage-1";
    type[0].run = SX127xResetFirstStage;
    type[0].cancel = thread_type_cancel;
    type[0].kill = NULL;

    type[1].name = "stage-2";
    type[1].run = SX127xResetSecondStage;
    type[1].cancel = thread_type_cancel;
    type[1].kill = NULL;

    task[0].type = &type[0];
    task[0].run_timeout = 1;
    task[0].complete = NULL;

    task[1].type = &type[1];
    task[1].run_timeout = 6;
    task[1].complete = SX127xResetComplete;

    runqueue_task_add(&SX127x.S.q, &task[0], false);
    runqueue_task_add(&SX127x.S.q, &task[1], false);
}

void SX127xSetRfTxPower(int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX127xRead(REG_PACONFIG);
    paDac = SX127xRead(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | SX127xGetPaSelect(power);

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else {
        if (power > 0) {
            if (power > 15) {
                power = 15;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (7 << 4) | (power);
        }
        else {
            if (power < -4) {
                power = -4;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (0 << 4) | (power + 4);
        }
    }
    SX127xWrite(REG_PACONFIG, paConfig);
    SX127xWrite(REG_PADAC, paDac);
}

static uint8_t SX127xGetPaSelect( int8_t power )
{
    return RF_PACONFIG_PASELECT_PABOOST;
#if 0
    if( power > 14 )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
#endif
}

void SX127xSetAntSwLowPower( bool status )
{
    if (RadioIsActive != status) {
        RadioIsActive = status;

        if (status == false) {
            SX127xAntSwInit();
        }
        else {
            SX127xAntSwDeInit();
        }
    }
}

void SX127xAntSwInit( void )
{
#if 0
    GpioInit( &AntSwitchRx, RADIO_ANT_SWITCH_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &AntSwitchTxBoost, RADIO_ANT_SWITCH_TX_BOOST, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &AntSwitchTxRfo, RADIO_ANT_SWITCH_TX_RFO, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX127xAntSwDeInit( void )
{
#if 0
    GpioInit( &AntSwitchRx, RADIO_ANT_SWITCH_RX, PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    GpioInit( &AntSwitchTxBoost, RADIO_ANT_SWITCH_TX_BOOST, PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    GpioInit( &AntSwitchTxRfo, RADIO_ANT_SWITCH_TX_RFO, PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
#endif
}

void SX127xSetAntSw( uint8_t opMode )
{
#if 0
    uint8_t paConfig =  SX127xRead( REG_PACONFIG );
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
        {
            GpioWrite( &AntSwitchTxBoost, 1 );
        }
        else
        {
            GpioWrite( &AntSwitchTxRfo, 1 );
        }
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        GpioWrite( &AntSwitchRx, 1 );
        break;
    }
#endif
}

bool SX127xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX127xGetDio1PinState( void )
{
    return 0;
//    return GpioRead( &SX127x.DIO1 );
}

#if defined( USE_RADIO_DEBUG )
void SX127xDbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

void SX127xDbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif

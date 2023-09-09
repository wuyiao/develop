/*!
 * \file      sx1276.c
 *
 * \brief     SX127x driver implementation
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
 * \author    Wael Guibene ( Semtech )
 */
#include <math.h>
#include <assert.h>
#include <string.h>
#include <thread.h>
#include <adapter.h>
#include <ubuf.h>
#include <linklist.h>
#include <timer.h>
#include <sx127x.h>
#include <sx127x-board.h>
#include "sx127xRegs-Fsk.h"
#include "sx127xRegs-LoRa.h"

/*!
 * \brief Internal frequency of the radio
 */
#define SX127x_XTAL_FREQ                            32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX127x_PLL_STEP_SHIFT_AMOUNT                ( 8 )

/*!
 * \brief PLL step - scaled with SX127x_PLL_STEP_SHIFT_AMOUNT
 */
#define SX127x_PLL_STEP_SCALED                      ( SX127x_XTAL_FREQ >> ( 19 - SX127x_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Radio buffer size
 */
#define RX_TX_BUFFER_SIZE                           256

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void );

/*!
 * \brief Sets the SX127x in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
static void SX127xSetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX127x FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
static void SX127xWriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX127x FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
static void SX127xReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX127x operating mode
 *
 * \param [IN] opMode New operating mode
 */
static void SX127xSetOpMode( uint8_t opMode );

/*!
 * \brief Get frequency in Hertz for a given number of PLL steps
 *
 * \param [in] pllSteps Number of PLL steps
 *
 * \returns Frequency in Hertz
 */
static uint32_t SX127xConvertPllStepToFreqInHz( uint32_t pllSteps );

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t SX127xConvertFreqInHzToPllStep( uint32_t freqInHz );

/*!
 * \brief Get the parameter corresponding to a FSK Rx bandwith immediately above the minimum requested one.
 *
 * \param [in] bw Minimum required bandwith in Hz
 *
 * \returns parameter
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bw );

/*!
 * \brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * \param [in] bw LoRa bandwidth parameter
 *
 * \returns Actual LoRa bandwidth in Hertz
 */
static uint32_t SX127xGetLoRaBandwidthInHz( uint32_t bw );

/*!
 * Compute the numerator for GFSK time-on-air computation.
 *
 * \remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * \param [in] preambleLen 
 * \param [in] fixLen 
 * \param [in] payloadLen 
 * \param [in] crcOn 
 *
 * \returns GFSK time-on-air numerator
 */
static uint32_t SX127xGetGfskTimeOnAirNumerator( uint16_t preambleLen, bool fixLen,
                                                 uint8_t payloadLen, bool crcOn );

/*!
 * Compute the numerator for LoRa time-on-air computation.
 *
 * \remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * \param [in] bandwidth 
 * \param [in] datarate 
 * \param [in] coderate 
 * \param [in] preambleLen 
 * \param [in] fixLen 
 * \param [in] payloadLen 
 * \param [in] crcOn 
 *
 * \returns LoRa time-on-air numerator
 */
static uint32_t SX127xGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*
 * SX127x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
static void SX127xOnDio0Irq( struct uloop_fd *fd, unsigned int events );

/*!
 * \brief DIO 1 IRQ callback
 */
static void SX127xOnDio1Irq( struct uloop_fd *fd, unsigned int events );

/*!
 * \brief DIO 2 IRQ callback
 */
static void SX127xOnDio2Irq( struct uloop_fd *fd, unsigned int events );

/*!
 * \brief DIO 3 IRQ callback
 */
static void SX127xOnDio3Irq( struct uloop_fd *fd, unsigned int events );

/*!
 * \brief DIO 4 IRQ callback
 */
static void SX127xOnDio4Irq( struct uloop_fd *fd, unsigned int events );

/*!
 * \brief Tx & Rx timeout timer callback
 */
static void SX127xOnTimeoutIrq( struct uloop_timeout *timeout );

//static void SX127xPrintRegisters(uint16_t Start, uint16_t End);
static void SX127xAsyncSendThread(struct thread *t);
static void SX127xAsyncSendThreadComplete(struct runqueue *q, struct runqueue_task *t);
/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_TX_BUFFER_SIZE];
static struct ubuf *TxUbuf;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX127x_t SX127x;

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX127xOnDio0Irq, SX127xOnDio1Irq,
                            SX127xOnDio2Irq, SX127xOnDio3Irq,
                            SX127xOnDio4Irq, NULL };

/*!
 * Tx and Rx timers
 */
static struct uloop_timeout TxTimeoutTimer;
static struct uloop_timeout RxTimeoutTimer;
static struct uloop_timeout RxTimeoutSyncWord;
static struct thread Thread;

/*
 * Radio driver functions implementation
 */

void SX127xInit( RadioEvents_t *events )
{
    uint8_t i;

    RadioEvents = events;

    TxUbuf = ubuf_new(256);

    // Initialize driver timeout timers
    memset(&TxTimeoutTimer, 0, sizeof(TxTimeoutTimer));
    memset(&RxTimeoutTimer, 0, sizeof(RxTimeoutTimer));
    memset(&RxTimeoutSyncWord, 0, sizeof(RxTimeoutSyncWord));

    TxTimeoutTimer.cb = SX127xOnTimeoutIrq;
    RxTimeoutTimer.cb = SX127xOnTimeoutIrq;
    RxTimeoutSyncWord.cb = SX127xOnTimeoutIrq;

#if 0
    /* scheduler */
    memset(&SX127x.S, 0, sizeof(SX127x.S));
    runqueue_init(&SX127x.S.q);
	SX127x.S.q.empty_cb = thread_q_done;
	SX127x.S.q.max_running_tasks = 1;
#endif

    /* async thread */
    memset(&Thread, 0, sizeof(Thread));
    Thread.task.run_timeout = 1;
    Thread.thread = SX127xAsyncSendThread;
    Thread.task.complete = SX127xAsyncSendThreadComplete;
    thread_init(&Thread);

    RxChainCalibration( );

    SX127xSetOpMode( RF_OPMODE_SLEEP );

    SX127xIoIrqInit( DioIrq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX127xSetModem( RadioRegsInit[i].Modem );
        SX127xWrite( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX127xSetModem( MODEM_FSK );

    SX127x.Settings.State = RF_IDLE;
}

RadioState_t SX127xGetStatus( void )
{
    return SX127x.Settings.State;
}

void SX127xSetChannel( uint32_t freq )
{
    uint32_t freqInPllSteps = SX127xConvertFreqInHzToPllStep( freq );

    SX127x.Settings.Channel = freq;

    SX127xWrite( REG_FRFMSB, ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF ) );
    SX127xWrite( REG_FRFMID, ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF ) );
    SX127xWrite( REG_FRFLSB, ( uint8_t )( freqInPllSteps & 0xFF ) );
}

bool SX127xIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    SX127xSetSleep( );

    SX127xSetModem( MODEM_FSK );

    SX127xSetChannel( freq );

    SX127xWrite( REG_RXBW, GetFskBandwidthRegValue( rxBandwidth ) );
    SX127xWrite( REG_AFCBW, GetFskBandwidthRegValue( rxBandwidth ) );

    SX127xSetOpMode( RF_OPMODE_RECEIVER );

    //DelayMs( 1 );
    usleep(1000);

    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = SX127xReadRssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    SX127xSetSleep( );
    return status;
}

uint32_t SX127xRandom( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX127xSetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX127xWrite( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX127xSetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        //DelayMs( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX127xRead( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX127xSetSleep( );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX127xRead( REG_PACONFIG );

    initialFreq = SX127xConvertPllStepToFreqInHz( ( ( ( uint32_t )SX127xRead( REG_FRFMSB ) << 16 ) |
                                                    ( ( uint32_t )SX127xRead( REG_FRFMID ) << 8 ) |
                                                    ( ( uint32_t )SX127xRead( REG_FRFLSB ) ) ) );

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX127xWrite( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX127xWrite( REG_IMAGECAL, ( SX127xRead( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX127xRead( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX127xSetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX127xWrite( REG_IMAGECAL, ( SX127xRead( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX127xRead( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX127xWrite( REG_PACONFIG, regPaConfigInitVal );
    SX127xSetChannel( initialFreq );
}

void SX127xSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX127xSetModem( modem );

    SX127xSetStby( );

    switch( modem )
    {
    case MODEM_FSK:
        {
            SX127x.Settings.Fsk.Bandwidth = bandwidth;
            SX127x.Settings.Fsk.Datarate = datarate;
            SX127x.Settings.Fsk.BandwidthAfc = bandwidthAfc;
            SX127x.Settings.Fsk.FixLen = fixLen;
            SX127x.Settings.Fsk.PayloadLen = payloadLen;
            SX127x.Settings.Fsk.CrcOn = crcOn;
            SX127x.Settings.Fsk.IqInverted = iqInverted;
            SX127x.Settings.Fsk.RxContinuous = rxContinuous;
            SX127x.Settings.Fsk.PreambleLen = preambleLen;
            SX127x.Settings.Fsk.RxSingleTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;

            uint32_t bitRate = ( uint32_t )( SX127x_XTAL_FREQ / datarate );
            SX127xWrite( REG_BITRATEMSB, ( uint8_t )( bitRate >> 8 ) );
            SX127xWrite( REG_BITRATELSB, ( uint8_t )( bitRate & 0xFF ) );

            SX127xWrite( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX127xWrite( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX127xWrite( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX127xWrite( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX127xWrite( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX127xWrite( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

            SX127xWrite( REG_PACKETCONFIG1,
                         ( SX127xRead( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX127xWrite( REG_PACKETCONFIG2, ( SX127xRead( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                ULOG_ERR("Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported");
                //while( 1 );
            }
            bandwidth += 7;
            SX127x.Settings.LoRa.Bandwidth = bandwidth;
            SX127x.Settings.LoRa.Datarate = datarate;
            SX127x.Settings.LoRa.Coderate = coderate;
            SX127x.Settings.LoRa.PreambleLen = preambleLen;
            SX127x.Settings.LoRa.FixLen = fixLen;
            SX127x.Settings.LoRa.PayloadLen = payloadLen;
            SX127x.Settings.LoRa.CrcOn = crcOn;
            SX127x.Settings.LoRa.FreqHopOn = freqHopOn;
            SX127x.Settings.LoRa.HopPeriod = hopPeriod;
            SX127x.Settings.LoRa.IqInverted = iqInverted;
            SX127x.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX127x.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX127x.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX127xWrite( REG_LR_MODEMCONFIG1,
                         ( SX127xRead( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX127xWrite( REG_LR_MODEMCONFIG2,
                         ( SX127xRead( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX127xWrite( REG_LR_MODEMCONFIG3,
                         ( SX127xRead( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX127x.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX127xWrite( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX127xWrite( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX127xWrite( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX127xWrite( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( SX127x.Settings.LoRa.FreqHopOn == true )
            {
                SX127xWrite( REG_LR_PLLHOP, ( SX127xRead( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX127xWrite( REG_LR_HOPPERIOD, SX127x.Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( SX127x.Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX127xWrite( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                SX127xWrite( REG_LR_HIGHBWOPTIMIZE2, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX127xWrite( REG_LR_HIGHBWOPTIMIZE1, 0x02 );
                SX127xWrite( REG_LR_HIGHBWOPTIMIZE2, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX127xWrite( REG_LR_HIGHBWOPTIMIZE1, 0x03 );
            }

            if( datarate == 6 )
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE,
                             ( SX127xRead( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX127xWrite( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE,
                             ( SX127xRead( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX127xWrite( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void SX127xSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX127xSetModem( modem );

    SX127xSetStby( );

    SX127xSetRfTxPower( power );

    switch( modem )
    {
    case MODEM_FSK:
        {
            SX127x.Settings.Fsk.Power = power;
            SX127x.Settings.Fsk.Fdev = fdev;
            SX127x.Settings.Fsk.Bandwidth = bandwidth;
            SX127x.Settings.Fsk.Datarate = datarate;
            SX127x.Settings.Fsk.PreambleLen = preambleLen;
            SX127x.Settings.Fsk.FixLen = fixLen;
            SX127x.Settings.Fsk.CrcOn = crcOn;
            SX127x.Settings.Fsk.IqInverted = iqInverted;
            SX127x.Settings.Fsk.TxTimeout = timeout;

            uint32_t fdevInPllSteps = SX127xConvertFreqInHzToPllStep( fdev );
            SX127xWrite( REG_FDEVMSB, ( uint8_t )( fdevInPllSteps >> 8 ) );
            SX127xWrite( REG_FDEVLSB, ( uint8_t )( fdevInPllSteps & 0xFF ) );

            uint32_t bitRate = ( uint32_t )( SX127x_XTAL_FREQ / datarate );
            SX127xWrite( REG_BITRATEMSB, ( uint8_t )( bitRate >> 8 ) );
            SX127xWrite( REG_BITRATELSB, ( uint8_t )( bitRate & 0xFF ) );

            SX127xWrite( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX127xWrite( REG_PREAMBLELSB, preambleLen & 0xFF );

            SX127xWrite( REG_PACKETCONFIG1,
                         ( SX127xRead( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX127xWrite( REG_PACKETCONFIG2, ( SX127xRead( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            SX127x.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                ULOG_ERR("Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported");
                //while( 1 );
            }
            bandwidth += 7;
            SX127x.Settings.LoRa.Bandwidth = bandwidth;
            SX127x.Settings.LoRa.Datarate = datarate;
            SX127x.Settings.LoRa.Coderate = coderate;
            SX127x.Settings.LoRa.PreambleLen = preambleLen;
            SX127x.Settings.LoRa.FixLen = fixLen;
            SX127x.Settings.LoRa.FreqHopOn = freqHopOn;
            SX127x.Settings.LoRa.HopPeriod = hopPeriod;
            SX127x.Settings.LoRa.CrcOn = crcOn;
            SX127x.Settings.LoRa.IqInverted = iqInverted;
            SX127x.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX127x.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX127x.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX127x.Settings.LoRa.FreqHopOn == true )
            {
                SX127xWrite( REG_LR_PLLHOP, ( SX127xRead( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX127xWrite( REG_LR_HOPPERIOD, SX127x.Settings.LoRa.HopPeriod );
            }

            SX127xWrite( REG_LR_MODEMCONFIG1,
                         ( SX127xRead( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX127xWrite( REG_LR_MODEMCONFIG2,
                         ( SX127xRead( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX127xWrite( REG_LR_MODEMCONFIG3,
                         ( SX127xRead( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX127x.Settings.LoRa.LowDatarateOptimize << 3 ) );

            SX127xWrite( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX127xWrite( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE,
                             ( SX127xRead( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX127xWrite( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE,
                             ( SX127xRead( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX127xWrite( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX127xGetTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * SX127xGetGfskTimeOnAirNumerator( preambleLen, fixLen, payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * SX127xGetLoRaTimeOnAirNumerator( bandwidth, datarate, coderate, preambleLen, fixLen,
                                                                   payloadLen, crcOn );
            denominator = SX127xGetLoRaBandwidthInHz( bandwidth );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void SX127xAsyncSendThreadComplete(struct runqueue *q, struct runqueue_task *t)
{
    // Write payload buffer
    SX127xWriteFifo(ubuf_data_pointer(TxUbuf), ubuf_used_len(TxUbuf));

    ubuf_reset(TxUbuf);

    SX127xSetTx(SX127x.Settings.LoRa.TxTimeout);
}

void SX127xAsyncSendThread(struct thread *t)
{
    //uint8_t size = ubuf_used_len(listnode_head(&Pending));
    uint8_t size = ubuf_used_len(TxUbuf);

    if( SX127x.Settings.LoRa.IqInverted == true )
    {
        SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
        SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    SX127x.Settings.LoRaPacketHandler.Size = size;

    // Initializes the payload size
    SX127xWrite( REG_LR_PAYLOADLENGTH, size );

    // Full buffer used for Tx
    SX127xWrite( REG_LR_FIFOTXBASEADDR, 0 );
    SX127xWrite( REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( SX127xRead( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
    {
        SX127xSetStby( );
    }
}

void SX127xAsyncSend( uint8_t *buffer, uint8_t size )
{
    switch( SX127x.Settings.Modem ) {
        case MODEM_FSK:
            {
                SX127xSend(buffer, size);
            }
        case MODEM_LORA:
            {
                //printf("ubuf: %p\n", TxUbuf);
                //printf("used: %d\n", ubuf_used_len(TxUbuf));
                //printf("capacity: %d\n", ubuf_capacity(TxUbuf));
                memcpy(ubuf_put(TxUbuf, size), buffer, size);
                thread_schedule(&SX127x.S, &Thread);
            }
    }
}

void SX127xSend( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        {
            SX127x.Settings.FskPacketHandler.NbBytes = 0;
            SX127x.Settings.FskPacketHandler.Size = size;

            if( SX127x.Settings.Fsk.FixLen == false )
            {
                SX127xWriteFifo( ( uint8_t* )&size, 1 );
            }
            else
            {
                SX127xWrite( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                SX127x.Settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy( RxTxBuffer, buffer, size );
                SX127x.Settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX127xWriteFifo( buffer, SX127x.Settings.FskPacketHandler.ChunkSize );
            SX127x.Settings.FskPacketHandler.NbBytes += SX127x.Settings.FskPacketHandler.ChunkSize;
            txTimeout = SX127x.Settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( SX127x.Settings.LoRa.IqInverted == true )
            {
                SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX127x.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX127xWrite( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX127xWrite( REG_LR_FIFOTXBASEADDR, 0 );
            SX127xWrite( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX127xRead( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX127xSetStby( );
                usleep(1000);
                //DelayMs( 1 );
            }
            // Write payload buffer
            SX127xWriteFifo( buffer, size );
            txTimeout = SX127x.Settings.LoRa.TxTimeout;
        }
        break;
    }

    SX127xSetTx( txTimeout );
}

void SX127xSetSleep( void )
{
    uloop_timeout_cancel( &RxTimeoutTimer );
    uloop_timeout_cancel( &TxTimeoutTimer );
    uloop_timeout_cancel( &RxTimeoutSyncWord );

    SX127xSetOpMode( RF_OPMODE_SLEEP );

    // Disable TCXO radio is in SLEEP mode
    SX127xSetBoardTcxo( false );

    SX127x.Settings.State = RF_IDLE;
}

void SX127xSetStby( void )
{
    uloop_timeout_cancel( &RxTimeoutTimer );
    uloop_timeout_cancel( &TxTimeoutTimer );
    uloop_timeout_cancel( &RxTimeoutSyncWord );

    SX127xSetOpMode( RF_OPMODE_STANDBY );
    SX127x.Settings.State = RF_IDLE;
}

void SX127xSetRx( uint32_t timeout )
{
    bool rxContinuous = false;
    uloop_timeout_cancel( &TxTimeoutTimer );

    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = SX127x.Settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

            SX127xWrite( REG_DIOMAPPING2, ( SX127xRead( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            SX127x.Settings.FskPacketHandler.FifoThresh = SX127xRead( REG_FIFOTHRESH ) & 0x3F;

            SX127xWrite( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            SX127x.Settings.FskPacketHandler.PreambleDetected = false;
            SX127x.Settings.FskPacketHandler.SyncWordDetected = false;
            SX127x.Settings.FskPacketHandler.NbBytes = 0;
            SX127x.Settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( SX127x.Settings.LoRa.IqInverted == true )
            {
                SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX127xWrite( REG_LR_INVERTIQ, ( ( SX127xRead( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX127xWrite( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX127x.Settings.LoRa.Bandwidth < 9 )
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE, SX127xRead( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX127xWrite( REG_LR_IFFREQ2, 0x00 );
                switch( SX127x.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x48 );
                    SX127xSetChannel(SX127x.Settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x44 );
                    SX127xSetChannel(SX127x.Settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x44 );
                    SX127xSetChannel(SX127x.Settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x44 );
                    SX127xSetChannel(SX127x.Settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x44 );
                    SX127xSetChannel(SX127x.Settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x44 );
                    SX127xSetChannel(SX127x.Settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX127xWrite( REG_LR_IFFREQ1, 0x40 );
                    break;
                }
            }
            else
            {
                SX127xWrite( REG_LR_DETECTOPTIMIZE, SX127xRead( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX127x.Settings.LoRa.RxContinuous;

            if( SX127x.Settings.LoRa.FreqHopOn == true )
            {
                SX127xWrite( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX127xWrite( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX127xWrite( REG_LR_FIFORXBASEADDR, 0 );
            SX127xWrite( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_TX_BUFFER_SIZE );

    SX127x.Settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        uloop_timeout_set( &RxTimeoutTimer, timeout );
        //TimerStart( &RxTimeoutTimer );
    }

    if( SX127x.Settings.Modem == MODEM_FSK )
    {
        SX127xSetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            uloop_timeout_set( &RxTimeoutSyncWord, SX127x.Settings.Fsk.RxSingleTimeout );
            //TimerStart( &RxTimeoutSyncWord );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX127xSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX127xSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

static void SX127xSetTx( uint32_t timeout )
{
    uloop_timeout_cancel( &RxTimeoutTimer );

    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoLevel
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) );

            SX127xWrite( REG_DIOMAPPING2, ( SX127xRead( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            SX127x.Settings.FskPacketHandler.FifoThresh = SX127xRead( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( SX127x.Settings.LoRa.FreqHopOn == true )
            {
                SX127xWrite( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX127xWrite( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    SX127x.Settings.State = RF_TX_RUNNING;
    //TimerStart( &TxTimeoutTimer );
    uloop_timeout_set( &TxTimeoutTimer, timeout );
    SX127xSetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX127xStartCad( void )
{
    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX127xWrite( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX127xWrite( REG_DIOMAPPING1, ( SX127xRead( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX127x.Settings.State = RF_CAD;
            SX127xSetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX127xSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )time * 1000;

    SX127xSetChannel( freq );

    SX127xSetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX127xWrite( REG_PACKETCONFIG2, ( SX127xRead( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    SX127xWrite( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX127xWrite( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );


    SX127x.Settings.State = RF_TX_RUNNING;
    uloop_timeout_set( &TxTimeoutTimer, timeout );
    //TimerStart( &TxTimeoutTimer );
    SX127xSetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX127xReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX127xRead( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( SX127x.Settings.Channel > RF_MID_BAND_THRESH )
        {
            rssi = RSSI_OFFSET_HF + SX127xRead( REG_LR_RSSIVALUE );
        }
        else
        {
            rssi = RSSI_OFFSET_LF + SX127xRead( REG_LR_RSSIVALUE );
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

static void SX127xSetOpMode( uint8_t opMode )
{
#if defined( USE_RADIO_DEBUG )
    switch( opMode )
    {
        case RF_OPMODE_TRANSMITTER:
            SX127xDbgPinTxWrite( 1 );
            SX127xDbgPinRxWrite( 0 );
            break;
        case RF_OPMODE_RECEIVER:
        case RFLR_OPMODE_RECEIVER_SINGLE:
            SX127xDbgPinTxWrite( 0 );
            SX127xDbgPinRxWrite( 1 );
            break;
        default:
            SX127xDbgPinTxWrite( 0 );
            SX127xDbgPinRxWrite( 0 );
            break;
    }
#endif
    if( opMode == RF_OPMODE_SLEEP )
    {
        SX127xSetAntSwLowPower( true );
    }
    else
    {
        // Enable TCXO if operating mode different from SLEEP.
        SX127xSetBoardTcxo( true );
        SX127xSetAntSwLowPower( false );
        SX127xSetAntSw( opMode );
    }
    SX127xWrite( REG_OPMODE, ( SX127xRead( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX127xSetModem( RadioModems_t modem )
{
    if( ( SX127xRead( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        SX127x.Settings.Modem = MODEM_LORA;
    }
    else
    {
        SX127x.Settings.Modem = MODEM_FSK;
    }

    if( SX127x.Settings.Modem == modem )
    {
        return;
    }

    SX127x.Settings.Modem = modem;
    switch( SX127x.Settings.Modem )
    {
    default:
    case MODEM_FSK:
        SX127xSetOpMode( RF_OPMODE_SLEEP );
        SX127xWrite( REG_OPMODE, ( SX127xRead( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX127xWrite( REG_DIOMAPPING1, 0x00 );
        SX127xWrite( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX127xSetOpMode( RF_OPMODE_SLEEP );
        SX127xWrite( REG_OPMODE, ( SX127xRead( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX127xWrite( REG_DIOMAPPING1, 0x00 );
        SX127xWrite( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX127xWrite( uint32_t addr, uint8_t data )
{
    SX127xWriteBuffer( addr, &data, 1 );
}

uint8_t SX127xRead( uint32_t addr )
{
    uint8_t data;
    SX127xReadBuffer( addr, &data, 1 );
    return data;
}

void SX127xWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    struct spi_operations *spi_ops = SX127x.Spi->adapter->ops;
#if 0
    int i;
    for (i = 0; i < size; i++) {
        spi_ops->writeb(SX127x.Spi, addr, buffer[i]);
        //printf("%02x", buffer[i]);
    }
#else
    spi_ops->write(SX127x.Spi, addr, buffer, size);
#endif
}

void SX127xReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    struct spi_operations *spi_ops = SX127x.Spi->adapter->ops;
#if 0
    int i;
    for (i = 0; i < size; i++) {
        spi_ops->readb(SX127x.Spi, addr, &buffer[i]);
        //if (!addr)
        //ULOG_DEBUG("%02x", buffer[i]);
    }
#else
    spi_ops->read(SX127x.Spi, addr, buffer, size);
#endif
}

static void SX127xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX127xWriteBuffer( 0, buffer, size );
}

static void SX127xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX127xReadBuffer( 0, buffer, size );
}

void SX127xSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX127xSetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( SX127x.Settings.Fsk.FixLen == false )
        {
            SX127xWrite( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX127xWrite( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX127xSetPublicNetwork( bool enable )
{
    SX127xSetModem( MODEM_LORA );
    SX127x.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX127xWrite( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX127xWrite( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

uint32_t SX127xGetWakeupTime( void )
{
    return SX127xGetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

static uint32_t SX127xConvertPllStepToFreqInHz( uint32_t pllSteps )
{
    uint32_t freqInHzInt;
    uint32_t freqInHzFrac;
    
    // freqInHz = pllSteps * ( SX127x_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    freqInHzInt = pllSteps >> SX127x_PLL_STEP_SHIFT_AMOUNT;
    freqInHzFrac = pllSteps - ( freqInHzInt << SX127x_PLL_STEP_SHIFT_AMOUNT );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return freqInHzInt * SX127x_PLL_STEP_SCALED + 
           ( ( freqInHzFrac * SX127x_PLL_STEP_SCALED + ( 128 ) ) >> SX127x_PLL_STEP_SHIFT_AMOUNT );
}

static uint32_t SX127xConvertFreqInHzToPllStep( uint32_t freqInHz )
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX127x_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX127x_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * SX127x_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << SX127x_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX127x_PLL_STEP_SHIFT_AMOUNT ) + ( SX127x_PLL_STEP_SCALED >> 1 ) ) /
             SX127x_PLL_STEP_SCALED );
}

static uint8_t GetFskBandwidthRegValue( uint32_t bw )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bw >= FskBandwidths[i].bandwidth ) && ( bw < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

static uint32_t SX127xGetLoRaBandwidthInHz( uint32_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case 0: // 125 kHz
        bandwidthInHz = 125000UL;
        break;
    case 1: // 250 kHz
        bandwidthInHz = 250000UL;
        break;
    case 2: // 500 kHz
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t SX127xGetGfskTimeOnAirNumerator( uint16_t preambleLen, bool fixLen,
                                                 uint8_t payloadLen, bool crcOn )
{
    const uint8_t syncWordLength = 3;

    return ( preambleLen << 3 ) +
           ( ( fixLen == false ) ? 8 : 0 ) +
             ( syncWordLength << 3 ) +
             ( ( payloadLen +
               ( 0 ) + // Address filter size
               ( ( crcOn == true ) ? 2 : 0 ) 
               ) << 3 
             );
}

static uint32_t SX127xGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

static void SX127xOnTimeoutIrq( struct uloop_timeout *timeout )
{
    switch( SX127x.Settings.State )
    {
    case RF_RX_RUNNING:
        if( SX127x.Settings.Modem == MODEM_FSK )
        {
            SX127x.Settings.FskPacketHandler.PreambleDetected = false;
            SX127x.Settings.FskPacketHandler.SyncWordDetected = false;
            SX127x.Settings.FskPacketHandler.NbBytes = 0;
            SX127x.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX127xWrite( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX127xWrite( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( SX127x.Settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX127xWrite( REG_RXCONFIG, SX127xRead( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            }
            else
            {
                SX127x.Settings.State = RF_IDLE;
                uloop_timeout_cancel( &RxTimeoutSyncWord );
            }
        }
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // Reported issue of SPI data corruption resulting in TX TIMEOUT 
        // is NOT related to a bug in radio transceiver.
        // It is mainly caused by improper PCB routing of SPI lines and/or
        // violation of SPI specifications.
        // To mitigate redesign, Semtech offers a workaround which resets
        // the radio transceiver and putting it into a known state.

        // BEGIN WORKAROUND

        // Reset the radio
        SX127xReset( NULL );
        usleep(1000);

        // Calibrate Rx chain
        RxChainCalibration( );

        // Initialize radio default values
        SX127xSetOpMode( RF_OPMODE_SLEEP );

        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX127xSetModem( RadioRegsInit[i].Modem );
            SX127xWrite( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        SX127xSetModem( MODEM_FSK );

        // Restore previous network type setting.
        SX127xSetPublicNetwork( SX127x.Settings.LoRa.PublicNetwork );
        // END WORKAROUND

        SX127x.Settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

static void SX127xOnDio0Irq( struct uloop_fd *fd, unsigned int events )
{
    char buf[32] = {0};
    volatile uint8_t irqFlags = 0;

    /* in fact, it should not touch the inside variable of uloop fd,
     * unless you known what exactly you are doning */
    if (fd->error) {
        fd->error = false;
    }
    /* wait for interrupt */
    if (events & ULOOP_READPRI) {
        lseek(fd->fd, 0, SEEK_SET);
        read(fd->fd, buf, sizeof(buf));
    }
    //ULOG_INFO("Dio0Irq: %d", atoi(buf));

    switch( SX127x.Settings.State )
    {
        case RF_RX_RUNNING:
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( SX127x.Settings.Modem )
            {
            case MODEM_FSK:
                if( SX127x.Settings.Fsk.CrcOn == true )
                {
                    irqFlags = SX127xRead( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        SX127xWrite( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        SX127xWrite( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        uloop_timeout_cancel( &RxTimeoutTimer );

                        if( SX127x.Settings.Fsk.RxContinuous == false )
                        {
                            uloop_timeout_cancel( &RxTimeoutSyncWord );
                            SX127x.Settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX127xWrite( REG_RXCONFIG, SX127xRead( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                        }

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        SX127x.Settings.FskPacketHandler.PreambleDetected = false;
                        SX127x.Settings.FskPacketHandler.SyncWordDetected = false;
                        SX127x.Settings.FskPacketHandler.NbBytes = 0;
                        SX127x.Settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // Read received packet size
                if( ( SX127x.Settings.FskPacketHandler.Size == 0 ) && ( SX127x.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX127x.Settings.Fsk.FixLen == false )
                    {
                        SX127xReadFifo( ( uint8_t* )&SX127x.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX127x.Settings.FskPacketHandler.Size = SX127xRead( REG_PAYLOADLENGTH );
                    }
                    SX127xReadFifo( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes, SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                    SX127x.Settings.FskPacketHandler.NbBytes += ( SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    SX127xReadFifo( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes, SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                    SX127x.Settings.FskPacketHandler.NbBytes += ( SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                }

                uloop_timeout_cancel( &RxTimeoutTimer );

                if( SX127x.Settings.Fsk.RxContinuous == false )
                {
                    SX127x.Settings.State = RF_IDLE;
                    uloop_timeout_cancel( &RxTimeoutSyncWord );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX127xWrite( REG_RXCONFIG, SX127xRead( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                }

                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, SX127x.Settings.FskPacketHandler.Size, SX127x.Settings.FskPacketHandler.RssiValue, 0 );
                }
                SX127x.Settings.FskPacketHandler.PreambleDetected = false;
                SX127x.Settings.FskPacketHandler.SyncWordDetected = false;
                SX127x.Settings.FskPacketHandler.NbBytes = 0;
                SX127x.Settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    // Clear Irq
                    SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX127xRead( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX127x.Settings.LoRa.RxContinuous == false )
                        {
                            SX127x.Settings.State = RF_IDLE;
                        }
                        uloop_timeout_cancel( &RxTimeoutTimer );

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        break;
                    }

                    // Returns SNR value [dB] rounded to the nearest integer value
                    SX127x.Settings.LoRaPacketHandler.SnrValue = ( ( ( int8_t )SX127xRead( REG_LR_PKTSNRVALUE ) ) + 2 ) >> 2;

                    int16_t rssi = SX127xRead( REG_LR_PKTRSSIVALUE );
                    if( SX127x.Settings.LoRaPacketHandler.SnrValue < 0 )
                    {
                        if( SX127x.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX127x.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          SX127x.Settings.LoRaPacketHandler.SnrValue;
                        }
                        else
                        {
                            SX127x.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          SX127x.Settings.LoRaPacketHandler.SnrValue;
                        }
                    }
                    else
                    {
                        if( SX127x.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX127x.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX127x.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    SX127x.Settings.LoRaPacketHandler.Size = SX127xRead( REG_LR_RXNBBYTES );
                    SX127xWrite( REG_LR_FIFOADDRPTR, SX127xRead( REG_LR_FIFORXCURRENTADDR ) );
                    SX127xReadFifo( RxTxBuffer, SX127x.Settings.LoRaPacketHandler.Size );

                    if( SX127x.Settings.LoRa.RxContinuous == false )
                    {
                        SX127x.Settings.State = RF_IDLE;
                    }
                    uloop_timeout_cancel( &RxTimeoutTimer );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, SX127x.Settings.LoRaPacketHandler.Size, SX127x.Settings.LoRaPacketHandler.RssiValue, SX127x.Settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            uloop_timeout_cancel( &TxTimeoutTimer );
            // TxDone interrupt
            switch( SX127x.Settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX127x.Settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

static void SX127xOnDio1Irq( struct uloop_fd *fd, unsigned int events )
{
    switch( SX127x.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX127x.Settings.Modem )
            {
            case MODEM_FSK:
                // Check FIFO level DIO1 pin state
                //
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in FSK reception mode a rising edge must be detected in order to handle the
                // IRQ.
                if( SX127xGetDio1PinState( ) == 0 )
                {
                    break;
                }
                // Stop timer
                uloop_timeout_cancel( &RxTimeoutSyncWord );

                // FifoLevel interrupt
                // Read received packet size
                if( ( SX127x.Settings.FskPacketHandler.Size == 0 ) && ( SX127x.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX127x.Settings.Fsk.FixLen == false )
                    {
                        SX127xReadFifo( ( uint8_t* )&SX127x.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX127x.Settings.FskPacketHandler.Size = SX127xRead( REG_PAYLOADLENGTH );
                    }
                }

                // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
                //
                //              When FifoLevel interrupt is used to offload the
                //              FIFO, the microcontroller should  monitor  both
                //              PayloadReady  and FifoLevel interrupts, and
                //              read only (FifoThreshold-1) bytes off the FIFO
                //              when FifoLevel fires
                if( ( SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes ) >= SX127x.Settings.FskPacketHandler.FifoThresh )
                {
                    SX127xReadFifo( ( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes ), SX127x.Settings.FskPacketHandler.FifoThresh - 1 );
                    SX127x.Settings.FskPacketHandler.NbBytes += SX127x.Settings.FskPacketHandler.FifoThresh - 1;
                }
                else
                {
                    SX127xReadFifo( ( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes ), SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                    SX127x.Settings.FskPacketHandler.NbBytes += ( SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Check RxTimeout DIO1 pin state
                //
                // DIO1 irq is setup to be triggered on rsing and falling edges
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in LoRa reception mode a rising edge must be detected in order to handle the
                // IRQ.
                if( SX127xGetDio1PinState( ) == 0 )
                {
                    break;
                }
                // Sync time out
                uloop_timeout_cancel( &RxTimeoutTimer );
                // Clear Irq
                SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX127x.Settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX127x.Settings.Modem )
            {
            case MODEM_FSK:
                // Check FIFO level DIO1 pin state
                //
                // As DIO1 interrupt is triggered when a rising or a falling edge is detected the IRQ handler must
                // verify DIO1 pin state in order to decide if something has to be done.
                // When radio is operating in FSK transmission mode a falling edge must be detected in order to handle
                // the IRQ.
                if( SX127xGetDio1PinState( ) == 1 )
                {
                    break;
                }

                // FifoLevel interrupt
                if( ( SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes ) > SX127x.Settings.FskPacketHandler.ChunkSize )
                {
                    SX127xWriteFifo( ( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes ), SX127x.Settings.FskPacketHandler.ChunkSize );
                    SX127x.Settings.FskPacketHandler.NbBytes += SX127x.Settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    SX127xWriteFifo( RxTxBuffer + SX127x.Settings.FskPacketHandler.NbBytes, SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes );
                    SX127x.Settings.FskPacketHandler.NbBytes += SX127x.Settings.FskPacketHandler.Size - SX127x.Settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

static void SX127xOnDio2Irq( struct uloop_fd *fd, unsigned int events )
{
    switch( SX127x.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX127x.Settings.Modem )
            {
            case MODEM_FSK:
                // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
                if( SX127x.DIO4 == NULL )
                {
                    SX127x.Settings.FskPacketHandler.PreambleDetected = true;
                }

                if( ( SX127x.Settings.FskPacketHandler.PreambleDetected != 0 ) && ( SX127x.Settings.FskPacketHandler.SyncWordDetected == 0 ) )
                {
                    uloop_timeout_cancel( &RxTimeoutSyncWord );

                    SX127x.Settings.FskPacketHandler.SyncWordDetected = true;

                    SX127x.Settings.FskPacketHandler.RssiValue = -( SX127xRead( REG_RSSIVALUE ) >> 1 );

                    SX127x.Settings.FskPacketHandler.AfcValue = ( int32_t )SX127xConvertPllStepToFreqInHz( ( ( uint16_t )SX127xRead( REG_AFCMSB ) << 8 ) |
                                                                                                           ( uint16_t )SX127xRead( REG_AFCLSB ) );
                    SX127x.Settings.FskPacketHandler.RxGain = ( SX127xRead( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( SX127x.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX127xRead( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX127x.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( SX127x.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX127xRead( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

static void SX127xOnDio3Irq( struct uloop_fd *fd, unsigned int events )
{
    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX127xRead( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            SX127xWrite( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

static void SX127xOnDio4Irq( struct uloop_fd *fd, unsigned int events )
{
    switch( SX127x.Settings.Modem )
    {
    case MODEM_FSK:
        {
            if( SX127x.Settings.FskPacketHandler.PreambleDetected == false )
            {
                SX127x.Settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

#if 0
void SX127xPrintRegisters(uint16_t Start, uint16_t End)
{
    uint16_t Loopv1, Loopv2, RegData;

    printf("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    //32 lines
    for (Loopv1 = Start; Loopv1 <= End;) {
        printf("0x");
        if (Loopv1 < 0x10) {
            printf("0");
        }
        printf("%X",Loopv1);                 //print the register number
        printf("  ");
        for (Loopv2 = 0; Loopv2 <= 15; Loopv2++) {
            RegData = SX127xRead(Loopv1);
            if (RegData < 0x10) {
                printf("0");
            }
            printf("%X",RegData);                //print the register number
            printf(" ");
            Loopv1++;
        }
        printf("\n");
    }
}
#endif

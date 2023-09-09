#ifndef _LORA_H_
#define _LORA_H_
#include <thread.h>

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE          false
#define LORA_IQ_INVERSION_ON_DISABLE                false


#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps

#define LORA_RADIO_DRIVER_USING_LORA_CHIP_SX127X

#if defined( LORA_RADIO_DRIVER_USING_LORA_CHIP_SX127X )

#define FSK_BANDWIDTH                               50000     // Hz >> SSB in sx127x
#define FSK_AFC_BANDWIDTH                           83333     // Hz

#elif defined( LORA_RADIO_DRIVER_USING_LORA_CHIP_SX126X) || defined( LORA_RADIO_DRIVER_USING_LORA_CHIP_LLCC68 )

#define FSK_BANDWIDTH                               100000    // Hz >> DSB in sx126x 
#define FSK_AFC_BANDWIDTH                           166666    // Hz >> Unused in sx126x

#elif defined( LORA_RADIO_DRIVER_USING_LORA_SOC_STM32WL )

#define FSK_BANDWIDTH                               100000    // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           166666    // Hz >> Unused in sx126x
#elif defined( LORA_RADIO_DRIVER_USING_LORA_CHIP_SX128X )

#define FSK_BANDWIDTH                               100000    // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           166666    // Hz >> Unused in sx126x
#else
    #error "Please define a lora-shield in the compiler options."
#endif

#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#define TX_TIMEOUT_VALUE                            1000
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 256 // Define the payload size here

#define LORA_MASTER_DEVADDR 0x11223344
#define LORA_SLAVER_DEVADDR 0x01020304
#define MAC_HEADER_OVERHEAD 13

// Ping pong event
#define EV_RADIO_INIT            0x0001
#define EV_RADIO_TX_START        0x0002
#define EV_RADIO_TX_DONE         0x0004
#define EV_RADIO_TX_TIMEOUT      0x0008
#define EV_RADIO_RX_DONE         0x0010
#define EV_RADIO_RX_TIMEOUT      0x0020
#define EV_RADIO_RX_ERROR        0x0040
#define EV_RADIO_ALL             (EV_RADIO_INIT | EV_RADIO_TX_START | EV_RADIO_TX_DONE | EV_RADIO_TX_TIMEOUT | EV_RADIO_RX_DONE | EV_RADIO_RX_TIMEOUT | EV_RADIO_RX_ERROR)

#define BUCKETSIZE              (jhash_size(8))
#define BUCKETMASK              (jhash_mask(8))

struct lora_context {
    int event; 
    void *data;
    struct thread thread;
};

int lora_service(void);
#endif

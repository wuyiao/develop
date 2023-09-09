#ifndef _EDGE_ADAPTER_H_
#define _EDGE_ADAPTER_H_

#include <stdint.h>
#include <edge.h>

#define ADAPTER_DECLARE(fn) INIT(fn##_adapter, 2)


#define LOW 		0
#define HIGH 		1
#define GPIO_IN         0
#define GPIO_OUT        1

struct edge_device;

struct gpio_operations {
    int (*reserve)(struct edge_device *dev);
    int (*release)(struct edge_device *dev);
    int (*open)(struct edge_device *dev);
    int (*close)(struct edge_device *dev);
    int (*set_direction)(struct edge_device *dev, int direction);
    int (*set_state)(struct edge_device *dev, int state);
    int (*get_state)(struct edge_device *dev);
    int (*set_edge)(struct edge_device *dev, char *edge);
    int (*digital_read)(struct edge_device *dev);
    int (*digital_write)(struct edge_device *dev, int state);
};

struct serial_operations {
    int (*open)(struct edge_device *dev);
    int (*close)();
    int (*write)(unsigned char *send_buf, int data_len);
    int (*read)(unsigned char *rcv_buf, int data_len);
};

struct spi_operations {
    int (*open)(struct edge_device *dev);
    int (*close)(struct edge_device *dev);
    int (*writeb)(struct edge_device *dev, uint8_t address, uint8_t data);
    int (*readb)(struct edge_device *dev, uint8_t address, uint8_t *data);
    int (*write)(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size);
    int (*read)(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size);
    int (*transfer)(struct edge_device *dev, uint8_t const *tx, uint8_t const *rx, size_t size);
};

#if 0
struct ch34x_operations {
    int (*open)(uint8_t index);
    void (*close)();
    int (*get_drv_ver)( char *drv_version );
    int (*get_vendor_id)( uint32_t *vendor_id );
    int (*set_para_mode)( uint32_t mode );
    int (*init_parallel)( uint32_t mode );

    int (*epp_read)( uint8_t *oBuffer, uint32_t ioLength, uint32_t PipeMode );
    // PipiMode->0 : read data
    // PipeMode->1 : read Addr    

    int (*epp_write)( uint8_t *iBuffer, uint32_t ioLength, uint32_t PipeMode );
    // PipeMode->0 : write data
    // PipeMode->1 : write data		

    int (*epp_read_data)( uint8_t *oBuffer, uint32_t ioLength ); //wanted length of read     

    int (*epp_write_data)( uint8_t *iBuffer, uint32_t ioLength );

    int (*epp_write_addr)( uint8_t *iBuffer, uint32_t ioLength );

    int (*epp_read_addr)( uint8_t *oBuffer, uint32_t ioLength );

    int (*epp_set_addr)( uint32_t iAddr );	//EPP: WR#=0,DS#=1,AS#=0,D0~D7 output
	
    int (*init_mem)(void);

    int (*mem_read_data)( uint8_t *oBuffer, uint32_t ioLength, uint32_t PipeMode );

    int (*mem_write_data)( uint8_t *iBuffer, uint32_t ioLength, uint32_t PipeMode );

    int (*set_stream)( uint32_t mode );		

    int (*set_delay_ms)( uint32_t iDelay );

    int (*read_data)( void *oBuffer, uint32_t *ioLength );

    int (*write_data)( void *iBuffer, uint32_t *ioLength );

    int (*write_read)( uint32_t iWriteLength, 
                        void *iWriteBuffer,
                     /*   ULONG iReadStep,
                        ULONG iReadTimes,*/
                        uint32_t *oReadLength,
                        void *oReadBuffer );

    int (*set_output)( uint32_t iEnable, uint32_t iSetDirOut, uint32_t iSetDataOut );

    int (*set_d5_d0)( uint8_t iSetDirOut, uint8_t iSetDataOut);

    int (*stream_i2c)( uint32_t iWriteLength, void *iWriteBuffer,
			uint32_t iReadLength, void *oReadBuffer);

    int (*read_eeprom)( EEPROM_TYPE iEepromID,  
                  uint32_t iAddr,  
	    	      uint32_t iLength,  
                  uint8_t *oBuffer );  

    int (*write_eeprom)( EEPROM_TYPE iEepromID,  
			uint32_t iAddr,  
			uint32_t iLength,  
			uint8_t *iBuffer );  

    int (*stream_spix)( uint32_t iChipSelect,
			uint32_t iLength,
			void *ioBuffer,
			void *ioBuffer2 );

    int (*stream_spi4)( uint32_t iChipSelect, 
			uint32_t iLength,
			void *ioBuffer );
};
#endif

struct adapter_object {
    /* adapter type */
    int type;
};

struct edge_adapter {
    /* object */
    struct adapter_object obj;

    /* adapter operations */
    void *ops;
};

extern struct list adapter_list;

struct edge_adapter *adapter_request(int type);
void *adapter_request_opi(struct edge_adapter *adapter);
int adapter_register(struct edge_adapter *ada);
int adapter_unregister(struct edge_adapter *ada);

#endif

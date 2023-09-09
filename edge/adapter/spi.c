/**
 * Author: xuzhou
 * Date: 28/07/2021
 */

#include <edge.h>
#include <edev.h>
#include <config.h>
#include <adapter.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <debug.h>

static struct spi_operations spi_ops;
static struct edge_adapter adapter;

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80
//#define SPI_SPEED       8000000
#define SPI_SPEED       1000000
#define SPI_BURST_CHUNK 1024

void wait_ms(unsigned long a) {
    struct timespec dly;
    struct timespec rem;

    dly.tv_sec = a / 1000;
    dly.tv_nsec = ((long)a % 1000) * 1000000;

    //MSG("NOTE dly: %ld sec %ld ns\n", dly.tv_sec, dly.tv_nsec);

    if((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
        //MSG("NOTE remain: %ld sec %ld ns\n", rem.tv_sec, rem.tv_nsec);
    }
    return;
}

void wait_us(unsigned long a) {
    struct timespec dly;
    struct timespec rem;

    dly.tv_sec = a / 1000000;
    dly.tv_nsec = ((long)a % 1000000) * 1000;

    //MSG("NOTE dly: %ld sec %ld ns\n", dly.tv_sec, dly.tv_nsec);

    if((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
        //MSG("NOTE remain: %ld sec %ld ns\n", rem.tv_sec, rem.tv_nsec);
    }
    return;
}

/* SPI initialization and configuration */
static int spi_open(struct edge_device *dev)
{
    int a = 0, b = 0;
    int fd = 0;
    struct spi_conf *conf = dev->conf;

    char spidev[32] = {0};

    snprintf(spidev, sizeof(spidev), "/dev/%s", conf->dev);

    /* open SPI device */
    fd = open(spidev, O_RDWR);
    if (fd < 0) {
        ULOG_ERR("failed to open SPI device %s, %s\n", spidev, strerror(errno));
        return -1;
    }

    /* setting SPI mode to 'mode 0' */
    //conf->mode = SPI_MODE_0;
    a = ioctl(fd, SPI_IOC_WR_MODE, &conf->mode);
    b = ioctl(fd, SPI_IOC_RD_MODE, &conf->mode);
    if ((a < 0) || (b < 0)) {
        ULOG_ERR("ERROR~(%s): SPI PORT FAIL TO SET IN MODE 0\n", conf->dev);
        close(fd);
        return -1;
    }

    /* setting SPI max clk (in Hz) */
    //conf->speed = SPI_SPEED;
    a = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &conf->speed);
    b = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &conf->speed);
    if ((a < 0) || (b < 0)) {
        ULOG_ERR("ERROR~(%s): SPI PORT FAIL TO SET MAX SPEED\n", conf->dev);
        close(fd);
        return -1;
    }

    /* setting SPI to MSB first */
    //conf->msb = 0;
    a = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &conf->msb);
    b = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &conf->msb);
    if ((a < 0) || (b < 0)) {
        ULOG_ERR("ERROR~(%s): SPI PORT FAIL TO SET MSB FIRST\n", conf->dev);
        close(fd);
        return -1;
    }

    /* setting SPI to 8 bits per word */
    //conf->bits_per_word = 0;
    a = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &conf->bits_per_word);
    b = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &conf->bits_per_word);
    if ((a < 0) || (b < 0)) {
        ULOG_ERR("ERROR~(%s): SPI PORT FAIL TO SET 8 BITS-PER-WORD\n", conf->dev);
        close(fd);
        return -1;
    }

    dev->fd[0] = fd;
    ULOG_DEBUG("spidev (%s) opened: %d\n", conf->dev, dev->fd[0]);

    return fd;
}

static int spi_close(struct edge_device *dev)
{
    if (dev->fd[0] > 0)
        return close(dev->fd[0]);

    return 0;
}

static int spi_raw_transfer(struct edge_device *dev, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	struct spi_ioc_transfer tr;

    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.cs_change = 0;
    //tr.delay_usecs = delay,
    //tr.speed_hz = speed,
    //tr.bits_per_word = bits,

	ret = ioctl(dev->fd[0], SPI_IOC_MESSAGE(1), &tr);
    /* determine return code */
    if (ret != (int)tr.len) {
        ULOG_ERR("ERROR~ ERROR~ SPI WRITE FAILURE\n");
        return -1;
    } else {
        //ULOG_DEBUG("Note: SPI write success");
        return 0;
    }
}

/* Simple write */
static int spi_write_byte(struct edge_device *dev, uint8_t address, uint8_t data)
{
    uint8_t out_buf[3];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    if ((address & 0x80) != 0) {
        ULOG_WARN("ERROR~ WARNING: SPI address > 127\n");
    }

    /* prepare frame to be sent */
    out_buf[0] = WRITE_ACCESS | (address & 0x7F);
    out_buf[1] = data;
    command_size = 2;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.len = command_size;
    //k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    //k.bits_per_word = 8;
    a = ioctl(dev->fd[0], SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        ULOG_ERR("ERROR~ ERROR~ SPI WRITE FAILURE\n");
        return -1;
    } else {
        //ULOG_DEBUG("Note: SPI write success");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
static int spi_read_byte(struct edge_device *dev, uint8_t address, uint8_t *data)
{
    uint8_t out_buf[3];
    uint8_t command_size;
    uint8_t in_buf[10];
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    if ((address & 0x80) != 0) {
        fprintf(stderr, "ERROR~ WARNING: SPI address > 127\n");
    }

    /* prepare frame to be sent */
    out_buf[0] = READ_ACCESS | (address & 0x7F);
    out_buf[1] = 0x00;
    command_size = 2;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.rx_buf = (unsigned long) in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(dev->fd[0], SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        fprintf(stderr, "ERROR~ SPI READ FAILURE\n");
        return -1;
    } else {
        *data = in_buf[command_size - 1];
        //ULOG_DEBUG("Note: SPI read success");
        return 0;
    }
}

static int spi_write(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t command[2];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int byte_transfered = 0;

    /* check input parameters */
    if ((address & 0x80) != 0) {
        ULOG_WARN("WARNING: SPI address > 127\n");
    }
    if (!data) {
        return -1;
    }
    if (size == 0) {
        ULOG_ERR("ERROR: BURST OF NULL LENGTH\n");
        return -1;
    }

    command[0] = WRITE_ACCESS | (address & 0x7F);
    command_size = 1;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].tx_buf = (unsigned long)(data);
    k[1].len = size;
    k[1].cs_change = 0;
    byte_transfered = ioctl(dev->fd[0], SPI_IOC_MESSAGE(2), &k);

    /* determine return code */
    if (byte_transfered != size + command_size) {
        ULOG_ERR("ERROR: SPI BURST WRITE FAILURE\n");
        return -1;
    } else {
        //ULOG_INFO("Note: SPI burst write success");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
static int spi_read(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t command[2];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int byte_transfered = 0;

    /* check input parameters */
    if ((address & 0x80) != 0) {
        ULOG_WARN("WARNING: SPI address > 127\n");
    }
    SANITY_CHECK(data);
    if (size == 0) {
        ULOG_ERR("ERROR: BURST OF NULL LENGTH\n");
        return -1;
    }

    command[0] = READ_ACCESS | (address & 0x7F);
    command_size = 1;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].rx_buf = (unsigned long)(data);
    k[1].len = size;
    k[1].cs_change = 0;
    byte_transfered = ioctl(dev->fd[0], SPI_IOC_MESSAGE(2), &k);

    /* determine return code */
    if (byte_transfered != size + command_size) {
        ULOG_ERR("ERROR: SPI BURST READ FAILURE\n");
        return -1;
    } else {
        //ULOG_INFO("Note: SPI burst read success");
        return 0;
    }
}

#if 0
/* Burst (multiple-byte) write */
static int spi_write(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t command[2];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int size_to_do, chunk_size, offset;
    int byte_transfered = 0;
    int i;

    /* check input parameters */
    if ((address & 0x80) != 0) {
        ULOG_WARN("WARNING: SPI address > 127");
    }
    if (!data) {
        return -1;
    }
    if (size == 0) {
        ULOG_ERR("ERROR: BURST OF NULL LENGTH");
        return -1;
    }

    command[0] = WRITE_ACCESS | (address & 0x7F);
    command_size = 1;
    size_to_do = size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].cs_change = 0;
    for (i = 0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < SPI_BURST_CHUNK) ? size_to_do : SPI_BURST_CHUNK;
        offset = i * SPI_BURST_CHUNK;
        k[1].tx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        byte_transfered += (ioctl(dev->fd[0], SPI_IOC_MESSAGE(2), &k) - k[0].len );
        //ULOG_INFO("BURST WRITE: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
        size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
    }

    /* determine return code */
    if (byte_transfered != size) {
        ULOG_ERR("ERROR: SPI BURST WRITE FAILURE");
        return -1;
    } else {
        //ULOG_INFO("Note: SPI burst write success");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
static int spi_read(struct edge_device *dev, uint8_t address, uint8_t *data, uint16_t size)
{
    uint8_t command[2];
    uint8_t command_size;
    struct spi_ioc_transfer k[2];
    int size_to_do, chunk_size, offset;
    int byte_transfered = 0;
    int i;

    /* check input parameters */
    if ((address & 0x80) != 0) {
        ULOG_WARN("WARNING: SPI address > 127");
    }
    SANITY_CHECK(data);
    if (size == 0) {
        ULOG_ERR("ERROR: BURST OF NULL LENGTH");
        return -1;
    }

    command[0] = READ_ACCESS | (address & 0x7F);
    command_size = 1;
    size_to_do = size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k[0].tx_buf = (unsigned long) &command[0];
    k[0].len = command_size;
    k[0].cs_change = 0;
    k[1].cs_change = 0;
    for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < SPI_BURST_CHUNK) ? size_to_do : SPI_BURST_CHUNK;
        offset = i * SPI_BURST_CHUNK;
        k[1].rx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        byte_transfered += (ioctl(dev->fd[0], SPI_IOC_MESSAGE(2), &k) - k[0].len );
        //ULOG_INFO("BURST READ: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
        size_to_do -= chunk_size;  /* subtract the quantity of data already transferred */
    }

    /* determine return code */
    if (byte_transfered != size) {
        ULOG_ERR("ERROR: SPI BURST READ FAILURE");
        return -1;
    } else {
        //ULOG_INFO("Note: SPI burst read success");
        return 0;
    }
}
#endif

ADAPTER_DECLARE(spi)
{
    memset(&spi_ops, 0, sizeof(spi_ops));
    memset(&adapter, 0, sizeof(adapter));

    spi_ops.open = spi_open;
    spi_ops.close = spi_close;
    spi_ops.readb = spi_read_byte;
    spi_ops.writeb = spi_write_byte;
    spi_ops.read = spi_read;
    spi_ops.write = spi_write;
    spi_ops.transfer = spi_raw_transfer;

    adapter.obj.type = SPI;
    adapter.ops = &spi_ops;

    adapter_register(&adapter);

    if (DEBUG(init, INIT))
        ULOG_DEBUG("(2) initialize spi\n");
}

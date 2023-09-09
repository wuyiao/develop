#include <edge.h>
#include <edev.h>
#include <config.h>
#include <adapter.h>
#include <debug.h>

static struct gpio_operations gpio_ops;
static struct edge_adapter adapter;
/*
 * Reserve a GPIO for this program's use.
 * @gpio the GPIO pin to reserve.
 * @return true if the reservation was successful.
 */
static int gpio_reserve(struct edge_device *dev) {
    /* File descriptor for GPIO controller class */
    int fd;
    /* Write buffer */
    char buf[8];
    struct gpio_conf *conf = dev->conf;

    /* Try to open GPIO controller class */
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        /* The file could not be opened */
        ULOG_ERR("ERROR~ gpio_reserve: could not open /sys/class/gpio/export");
        return false;
    }

    /* Prepare buffer */
    snprintf(buf, sizeof(buf), "%d", conf->number);

    /* Try to reserve GPIO */
    if (write(fd, buf, strlen(buf)) < 0) {
        close(fd);
        ULOG_ERR("ERROR~ gpio_reserve: could not write '%s' to /sys/class/gpio/export", buf);
        return false;
    }

    /* Close the GPIO controller class */
    if (close(fd) < 0) {
        ULOG_ERR("ERROR~ gpio_reserve: could not close /sys/class/gpio/export\r\n");
        return false;
    }

    /* Success */
    return true;
}

/*
 * Release a GPIO after use.
 * @gpio the GPIO pin to release.
 * @return true if the release was successful.
 */
static int gpio_release(struct edge_device *dev)
{
    /* File descriptor for GPIO controller class */
    int fd;
    /* Write buffer */
    char buf[8];
    struct gpio_conf *conf = dev->conf;

    /* Try to open GPIO controller class */
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        /* The file could not be opened */
       ULOG_ERR("ERROR~ gpio_release: could not open /sys/class/gpio/unexport\r\n");
       return false;
    }

    /* Prepare buffer */
    snprintf(buf, sizeof(buf), "%d", conf->number);

    /* Try to release GPIO */
    if (write(fd, buf, strlen(buf)) < 0) {
        ULOG_ERR("ERROR~ gpio_release: could not write /sys/class/gpio/unexport\r\n");
        return false;
    }

    /* Close the GPIO controller class */
    if (close(fd) < 0) {
        ULOG_ERR("ERROR~ gpio_release: could not close /sys/class/gpio/unexport\r\n");
        return false;
    }

    /* Success */
    return true;
}

#if 1
/*
 * Set the direction of the GPIO port.
 * @gpio the GPIO pin to release.
 * @direction the direction of the GPIO port.
 * @return true if the direction could be successfully set.
 */
static int gpio_set_direction(struct edge_device *dev, int direction)
{
    int fd; /* File descriptor for GPIO port */
    char buf[64]; /* Write buffer */
    struct gpio_conf *conf = dev->conf;

    /* Make the GPIO port path */
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", conf->number);

    /* Try to open GPIO port for writing only */
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        /* The file could not be opened */
        return false;
    }

    /* Set the port direction */
    if (direction == GPIO_OUT) {
        if (write(fd, "out", 3) < 0) {
            return false;
        }
    } else {
        if (write(fd, "in", 2) < 0) {
            return false;
        }
    }

    /* Close the GPIO port */
    if (close(fd) < 0) {
        return false;
    }

    /* Success */
    return true;
}

/*
 * Set the state of the GPIO port.
 * @gpio the GPIO pin to set the state for.
 * @state 1 or 0
 * @return true if the state change was successful.
 */
static int gpio_set_state(struct edge_device *dev, int state)
{
    int fd; /* File descriptor for GPIO port */
    char buf[32]; /* Write buffer */
    struct gpio_conf *conf = dev->conf;

    /* Make the GPIO port path */
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", conf->number);

    /* Try to open GPIO port */
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        /* The file could not be opened */
        return false;
    }

    /* Set the port state */
    if (write(fd, (state == 1 ? "1" : "0"), 1) < 0) {
        return false;
    }

    /* Close the GPIO port */
    if (close(fd) < 0) {
        return false;
    }

    /* Success */
    return true;
}

/*
 * Get the state of the GPIO port.
 * @gpio the gpio pin to get the state from.
 * @return GPIO_HIGH if the pin is HIGH, GPIO_LOW if the pin is low. GPIO_ERR
 * when an error occured. 
 */
static int gpio_get_state(struct edge_device *dev)
{
    int fd;             /* File descriptor for GPIO port */
    char buf[32];       /* Write buffer */
    char port_state; /* Character indicating the port state */
    int state; /* API integer indicating the port state */
    struct gpio_conf *conf = dev->conf;

    /* Make the GPIO port path */
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", conf->number);

    /* Try to open GPIO port */
    fd = open(buf, O_RDONLY);
    if (fd < 0) {
        /* The file could not be opened */
        fprintf(stderr, "ERROR~ gpio_get_state: could not open /sys/class/gpio/gpio%d/value\r\n", conf->number);
        return LOW;
    }

    /* Read the port state */
    if (read(fd, &port_state, 1) < 0) {
        close(fd);
        fprintf(stderr, "ERROR~ gpio_get_state: could not read /sys/class/gpio/gpio%d/value\r\n", conf->number);
        return LOW;
    }

    /* Translate the port state into API state */
    state = port_state == '1' ? HIGH : LOW;

    /* Close the GPIO port */
    if (close(fd) < 0) {
        fprintf(stderr, "ERROR~ gpio_get_state: could not close /sys/class/gpio/gpio%d/value\r\n", conf->number);
        return LOW;
    }

    /* Return the state */
    return state;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int gpio_digital_write(struct edge_device *dev, int state)
{
    gpio_set_direction(dev, GPIO_OUT);
    gpio_set_state(dev, state);

    return 0;
} 

/*
 * Read the state of the port. The port can be input
 * or output.
 * @gpio the GPIO pin to read from.
 * @return GPIO_HIGH if the pin is HIGH, GPIO_LOW if the pin is low. Output
 * is -1 when an error occurred.
 */
static int gpio_digital_read(struct edge_device *dev)
{
    int state; /* The port state */

    /* Read the port */
    state = gpio_get_state(dev);

    /* Return the port state */
    return state;
}

static int gpio_open(struct edge_device *dev)
{
    int fd;             /* File descriptor for GPIO port */
    char buf[32] = {0};       /* Write buffer */
    struct gpio_conf *conf = dev->conf;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", conf->number);
    /* Try to open GPIO port */
    fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        ULOG_ERR("gpio open %s failed, %s", buf, strerror(errno));
        /* The file could not be opened */
        return false;
    }

    dev->fd[0] = fd;

    return fd;
}

static int gpio_close(struct edge_device *dev)
{
    int i;

    for (i = 0; i < sizeof(dev->fd) && dev->fd[i] > 0; i++)
        if (dev->fd[i] > 0) {
            if (close(dev->fd[i]) < 0)
                return -1;
        }

    return 0;
}

#else
static int gpio_open(struct edge_device *dev)
{
    int fd;             /* File descriptor for GPIO port */
    char buf[32];       /* Write buffer */
    struct gpio_conf *conf = dev->conf;

    memset(buf, sizeof(buf), 0);

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", conf->number);
    ULOG_DEBUG("%s", buf);
    /* Try to open GPIO port */
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        ULOG_ERR("gpio open %s failed, %s", buf, strerror(errno));
        /* The file could not be opened */
        return false;
    }

    dev->fd[0] = fd;

    return fd;
}

static int gpio_close(struct edge_device *dev)
{
    int i;

    for (i = 0; i < sizeof(dev->fd) && dev->fd[i] > 0; i++)
        if (dev->fd[i] > 0) {
            if (close(dev->fd[i]) < 0)
                return -1;
        }

    return 0;
}

/*
 * Set the direction of the GPIO port.
 * @gpio the GPIO pin to release.
 * @direction the direction of the GPIO port.
 * @return true if the direction could be successfully set.
 */
static int gpio_set_direction(struct edge_device *dev, int direction)
{
    /* File descriptor for GPIO port */
    int fd;
    /* Write buffer */
    char buf[64] = {0};
    struct gpio_conf *conf = dev->conf;

    /* Make the GPIO port path */
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", conf->number);
    ULOG_DEBUG("%s", buf);
    /* Try to open GPIO port */
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        ULOG_ERR("gpio open %s failed, %s", buf, strerror(errno));
        /* The file could not be opened */
        return false;
    }

    /* Set the port direction */
    if (direction == GPIO_OUT) {
        if (write(fd, "out", 3) < 0) {
            ULOG_ERR("gpio set out failed, %s", strerror(errno));
            return false;
        }
    }
    else {
        if (write(fd, "in", 2) < 0) {
            ULOG_ERR("gpio set in failed, %s", strerror(errno));
            return false;
        }
    }

    /* Close the GPIO port */
    if (close(fd) < 0) {
        return false;
    }
    /* Success */
    return true;
}

/*
 * Set the state of the GPIO port.
 * @gpio the GPIO pin to set the state for.
 * @state 1 or 0
 * @return true if the state change was successful.
 */
static int gpio_set_state(struct edge_device *dev, int state)
{
    /* Set the port state */
    if (write(dev->fd[0], (state == 1 ? "1" : "0"), 1) < 0) {
        ULOG_ERR("gpio set state failed, %s", strerror(errno));
        return false;
    }

    /* Success */
    return true;
}

/*
 * Get the state of the GPIO port.
 * @gpio the gpio pin to get the state from.
 * @return GPIO_HIGH if the pin is HIGH, GPIO_LOW if the pin is low. GPIO_ERR
 * when an error occured. 
 */
static int gpio_get_state(struct edge_device *dev)
{
    char port_state; /* Character indicating the port state */
    int state; /* API integer indicating the port state */

    /* Read the port state */
    if (read(dev->fd[0], &port_state, 1) < 0) {
        ULOG_ERR("ERROR~ gpio_get_state: could not read fd %d, %s", dev->fd[0], strerror(errno));
        //close(dev->fd[1]);
        return LOW;
    }

    /* Translate the port state into API state */
    state = port_state == '1' ? HIGH : LOW;

    /* Return the state */
    return state;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int gpio_digital_write(struct edge_device *dev, int state)
{
    int ret = 0;

    //gpio_open(dev);
    ret = gpio_set_direction(dev, GPIO_OUT);
    ret += gpio_set_state(dev, state);
    //gpio_close(dev);

    return ret;
} 

/*
 * Read the state of the port. The port can be input
 * or output.
 * @gpio the GPIO pin to read from.
 * @return GPIO_HIGH if the pin is HIGH, GPIO_LOW if the pin is low. Output
 * is -1 when an error occurred.
 */
int gpio_digital_read(struct edge_device *dev)
{
    int state; /* The port state */

    /* Read the port */
    state = gpio_get_state(dev);

    /* Return the port state */
    return state;
}
#endif

static int gpio_set_edge(struct edge_device *dev, char *edge)
{
    int fd; /* File descriptor for GPIO port */
    char buf[32] = {0}; /* Write buffer */
    struct gpio_conf *conf = dev->conf;

    /* Make the GPIO port path */
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/edge", conf->number);

    /* Try to open GPIO port */
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        /* The file could not be opened */
        return false;
    }

    /* Set the port state */
    //if (write(fd, "rising", sizeof("rising")) < 0) {
    if (write(fd, edge, strlen(edge) + 1) < 0) {
    //if (write(fd, "both", sizeof("both")) < 0) {
        ULOG_ERR("write: %s", strerror(errno));
        return false;
    }

    /* Close the GPIO port */
    if (close(fd) < 0) {
        return false;
    }

    /* Success */
    return true;
}



/* for now, we only open one gpio for an edge device instance, later will be more */
ADAPTER_DECLARE(gpio)
{
    memset(&gpio_ops, 0, sizeof(gpio_ops));
    memset(&adapter, 0, sizeof(adapter));

    gpio_ops.reserve = gpio_reserve;
    gpio_ops.release = gpio_release;
    gpio_ops.open = gpio_open;
    gpio_ops.close = gpio_close;
    gpio_ops.set_direction = gpio_set_direction;
    gpio_ops.set_state = gpio_set_state;
    gpio_ops.get_state = gpio_get_state;
    gpio_ops.set_edge = gpio_set_edge;
    gpio_ops.digital_read = gpio_digital_read;
    gpio_ops.digital_write = gpio_digital_write;

    adapter.obj.type = GPIO;
    adapter.ops = &gpio_ops;

    adapter_register(&adapter);

    if (DEBUG(init, INIT))
        ULOG_DEBUG("(2) initialize gpio");
}

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <syslog.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <net/if.h>
#include <netinet/in.h>
#include <net/if_arp.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if_link.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <hlist.h>
#include <jhash.h>
#include <unistd.h>
#include <pthread.h>
#include <syslog.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
//#include <list.h>
//#include <openssl/ssl.h>
//#include <openssl/err.h>
//#include <openssl/opensslv.h>
//#include <openssl/crypto.h>
#include "ubus.h"
#include "mqtt.h"
#include "config.h"
#include "upload.h"
#include "net.h"
#include "ipc.h"
#include "utils.h"

//#include <json-c/json_object.h>
//#include <json-c/json_tokener.h>
//#include <json-c/printbuf.h>
//#include <json-c/json_object_private.h>


static void
controller_handle_signal(int signo)
{
    uloop_end();
}

static void
controller_setup_signals(void)
{
    struct sigaction s;

    memset(&s, 0, sizeof(s));
    s.sa_handler = controller_handle_signal;
    s.sa_flags = 0;
    sigaction(SIGINT, &s, NULL);
    sigaction(SIGTERM, &s, NULL);
    sigaction(SIGUSR1, &s, NULL);
    sigaction(SIGUSR2, &s, NULL);

    s.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &s, NULL);
}

static int
usage(const char *program_name)
{
    fprintf( stdout, "Usage: %s  [OPTIONS]\n", program_name );
    fprintf( stdout, "\t -l log debug information into system log\n");
    return 1;
}


int main(int argc, char *argv[])
{
    int i;
    int ch;
    int opts = 0;
    int use_syslog = 1;
    const char *socket = NULL;

    while ((ch = getopt(argc, argv, "c:i:v::d:f:r" )) != -1) {
        switch(ch) {
            case 'r':
                use_syslog = 1;
                opts++;
                break;

            default:
                return usage(argv[0]);
        }
    }

    ulog_open(ULOG_SYSLOG, LOG_USER, "controller");
    ulog_threshold(LOG_DEBUG);

    controller_setup_signals();

    if (controller_ubus_init(socket) < 0) {
        ULOG_ERR("Failed to connect to ubus\n");
        return 1;
    }

    config_init();
    
    //connector_init();

    ipc_init();
    mqtt_init();
    utils_monitor_cpustat();

    uloop_run();

    controller_ubus_done();

    ulog_close();

    return 0;
}

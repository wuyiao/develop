#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
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
//#include <list.h>
#include <hlist.h>
#include <jhash.h>
#include <pthread.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubox/utils.h>
//#include <openssl/ssl.h>
//#include <openssl/err.h>
//#include <openssl/opensslv.h>
//#include <openssl/crypto.h>
#include <termios.h>
#include "config.h"
//#include <libssh/libssh.h>

//#include <json-c/json_object.h>
//#include <json-c/json_tokener.h>
//#include <json-c/printbuf.h>
//#include <json-c/json_object_private.h>

#define BUCKETSIZ               (jhash_size(8))
#define BUCKETMASK              (jhash_mask(8))

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

#define RED(_text)  "\033[0;31m"_text"\033[0m"
#define GRN(_text)  "\033[1;32m"_text"\033[0m"
#define YLW(_text)  "\033[1;33m"_text"\033[0m"
#define BLUE(_text) "\033[1;36m"_text"\033[0m"
#define UNDERLINE(_text) "\033[4m"_text"\033[0m"


#define NONE    "\033[0m"

#define CLEAR()              printf("\033[2J")
#define CLEAR_ROW()          printf("\033[K")
#define MOVEUP(x)            printf("\033[%dA", (x))
#define MOVEDOWN(x)          printf("\033[%dB", (x))
#define MOVELEFT(y)          printf("\033[%dD", (y))
#define MOVERIGHT(y)         printf("\033[%dC",(y))
#define MOVETO(x,y)          printf("\033[%d;%dH", (x), (y))
#define RESET_CURSOR()       printf("\033[H")
#define HIDE_CURSOR()        printf("\033[?25l")
#define SHOW_CURSOR()        printf("\033[?25h")
#define SAVE_CURSOR()        printf("\033[s")
#define RESTORE_CURSOR()     printf("\033[u")
#define HIGHT_LIGHT()        printf("\033[7m")
#define UN_HIGHT_LIGHT()     printf("\033[27m")
#define RESET                printf(NONE)

static struct hlist_head devices[BUCKETSIZ];

static struct list_head timeouts;
static struct list_head produce_list, consume_list;
static struct uloop_timeout connector_timeout;

char *ifname;

static int sock;
//=============================================================================
int 
get_ifname(
    char *ifname
)
//=============================================================================
{
    struct ifaddrs *ifaddr, *ifa;
    int family, s, n;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    /* Walk through linked list, maintaining head pointer so we
     * can free list later 
     */

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        /* For an AF_INET* interface address, display the address */

        if (family == AF_INET) {
            /* Display interface name and family (including symbolic
             * form of the latter for the common families) 
             */

            printf("%-8s %s (%d)\n",
                    ifa->ifa_name,
                    (family == AF_PACKET) ? "AF_PACKET" :
                    (family == AF_INET) ? "AF_INET" :
                    (family == AF_INET6) ? "AF_INET6" : "???",
                    family);


            s = getnameinfo(ifa->ifa_addr,
                    (family == AF_INET) ? sizeof(struct sockaddr_in) :
                    sizeof(struct sockaddr_in6),
                    host, NI_MAXHOST,
                    NULL, 0, NI_NUMERICHOST);
            if (s != 0) {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
            }

            printf("\t\taddress: <%s>\n", host);
            if (strcasecmp(ifa->ifa_name, "lo") == 0 ||
                    strncasecmp(ifa->ifa_name, "docker", 6) == 0) {
            }
            else if (ifa->ifa_flags & IFF_RUNNING) {
                strncpy(ifname, ifa->ifa_name, IF_NAMESIZE);
                printf(BLUE("\t\tSelected interface %s with ip <%s>\n"), ifa->ifa_name, host);
            }

        }
    }

    freeifaddrs(ifaddr);
    return 0;
}



//=============================================================================
int 
get_ifip(
    char *ifname, 
    char *addr_str
)
//=============================================================================
{
    int fd;
    struct ifreq ifr;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
        /* Is the interface up? */
        if (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) {
            if (!(ifr.ifr_flags & IFF_RUNNING)) {
                fprintf( stderr, "Interface %s not up\n", ifname );
                close(fd);
                return -1;
            }
        }

        /* Get interface IP */
        if (ioctl(fd, SIOCGIFADDR, &ifr) == 0) {
            strncpy(addr_str, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr), 16);
            close(fd);
            return 0;
        }

        close(fd);
    }

    return -1;
}

//=============================================================================
int
get_ifmac( 
        char *ifname, 
        char *addr_str 
)
//=============================================================================
{
    struct ifreq ifr;
    int fd;
    unsigned char *mac;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
        if ( ioctl(fd, SIOCGIFHWADDR, &ifr ) == 0) {
            mac = (unsigned char *)ifr.ifr_hwaddr.sa_data;
            snprintf(addr_str, 13, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            close(fd);
            return 0;
        }

        close(fd);
    }

    return -1;
}

//==============================================================================
static int
get_peer_mac(
    struct in_addr *in,
    unsigned char *mac
)
//==============================================================================
{
    int fd, retry = 2;
    char addr[INET_ADDRSTRLEN+1];
    char command[256];
//    char buf[18];
//    unsigned char *ptr;
    struct sockaddr_in *sin;
    struct sockaddr_storage ss;
    struct arpreq arpreq;;

    memset(addr, 0, INET_ADDRSTRLEN+1);
    memset(&ss, 0, sizeof(ss));

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if ( fd == -1 ) {
        return -1;
    }

    sin = (struct sockaddr_in *)&ss;
    sin->sin_family = AF_INET;
    sin->sin_addr.s_addr = in->s_addr;

    while (retry) {
        retry--;
        memset(&arpreq, 0, sizeof(struct arpreq));
        memcpy(&arpreq.arp_pa, &ss, sizeof(struct sockaddr));
        sprintf(&arpreq.arp_dev[0], "%s", ifname);
        arpreq.arp_ha.sa_family = AF_UNSPEC;

        if (ioctl(fd, SIOCGARP, &arpreq) < 0) {
            fprintf(stderr, "error get mac addr at %s\n", inet_ntoa(*in));
            snprintf(command, sizeof(command), "ping -c 1 -q -w 3 -I %s %s", ifname, inet_ntoa(*in));
            system(command);
            close(fd);
        }
        else {
//            ptr = (unsigned char *)&arpreq.arp_ha.sa_data[0];
//            sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", *ptr, *(ptr+1), *(ptr+2), *(ptr+3), *(ptr+4), *(ptr+5));
//            fprintf(stderr, "Got mac "MACSTR"\n", MAC2STR(ptr));
            memcpy(mac, arpreq.arp_ha.sa_data, 6);
//            fprintf(stderr, "Got mac "MACSTR"\n", MAC2STR((unsigned char *)mac));
            close(fd);
            return 0;
        }
    }
    return -1;
}

void setnonblocking(int fd) {

  if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
    if (errno == ENODEV) {
      /* Some devices (like /dev/null redirected in)
       *       * can't be set to non-blocking */
    }
    else {
      exit(-1);
    }
  }
}

int net_connect()
{
    char ip_str[64];
    int port = 0;

    sock = socket(AF_INET, SOCK_STREAM, 0);

    if (sock < 0) {
        ULOG_ERR("socket error\n");
        return -1;
    }

    config_get_access_point(ip_str, &port);

    ULOG_INFO("access point: %s %d\n", ip_str, port);

    struct sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    //addr.sin_port = htons(9090);
    addr.sin_port = htons(port);
    //inet_pton(AF_INET, "36.152.118.245", &addr.sin_addr);
    inet_pton(AF_INET, ip_str, &addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ULOG_ERR("connect: %s", strerror(errno));
        close(sock);
        return -1;
    }

    ULOG_INFO("connected \n");
    return 0;
}

static void try_connect(struct uloop_timeout *timeout)
{
    if (net_connect() < 0) {
        uloop_timeout_set(&connector_timeout, 1000);
    }
}

void connector_init()
{
    memset(&connector_timeout, 0, sizeof(connector_timeout));
    connector_timeout.cb = try_connect;

    if (net_connect() < 0) {
        uloop_timeout_set(&connector_timeout, 10 * 1000);
    }
}

int send_data_to_remote_platform(void *data, int len)
{
    int rc = send(sock, data, len, 0);
    if (rc < 0) {
        if (uloop_timeout_remaining(&connector_timeout) < 0)
            uloop_timeout_set(&connector_timeout, 10 * 1000);
    }
    ULOG_INFO("send %d bytes: %s\n", rc, data);

    return rc;
}

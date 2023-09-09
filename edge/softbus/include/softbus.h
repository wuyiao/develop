#ifndef _EDGE_SOFTBUS_H_
#define _EDGE_SOFTBUS_H_

#include <ubuf.h>
#include <queue.h>

#define SB_DEBUG
struct list;

struct softbus_source {
    int priority;
    char addr[32];
};

struct softbus_peer {
    char name[32];
    struct list *source_list;
    struct list *sink_list;
};

struct softbus_sink {
    int priority;
    char addr[32];
    struct softbus_peer *peer;
    int (*sink)(struct softbus_peer *peer, struct softbus_source *source, struct ubuf *ub);
};

struct softbus_chain {
    struct softbus_peer *peer;
    struct softbus_source *source;
    struct list *chain;
};

struct softbus_data {
    char addr[32];
    struct ubuf *ub;
    TAILQ_ENTRY(softbus_data) entry;
};


int softbus_init(void);
struct softbus_peer *softbus_peer_new(char *name);
void softbus_peer_init(struct softbus_peer *peer);
void softbus_peer_add_source(struct softbus_peer *peer, char *addr, int priority);
void softbus_peer_add_sink(struct softbus_peer *peer, char *addr, int priority, int (*func)(struct softbus_peer *peer, struct softbus_source *source, struct ubuf *ub));
void softbus_add_map_chain(struct softbus_chain *chain, struct softbus_sink *sink);
void softbus_create_source_chain(struct softbus_peer *peer, struct softbus_source *source);
int softbus_peer_register(struct softbus_peer *peer);
int softbus_peer_unregister(struct softbus_peer *peer);
int softbus_push_data(char *addr, void *buf, int len);
void softbus_dump_all_chain(void);
void softbus_dump_remain_sink(void);
#endif

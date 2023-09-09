/* design by xuzhou, copyright Revised.
 * author    xuzhou ( ei.link )
 */
#include <edge.h>
#include <assert.h>
#include <map.h>
#include <queue.h>
#include <linklist.h>
#include <thread.h>
#include <softbus.h>
#include <debug.h>

static map_void_t map_chain;
static struct list unmaped_sink;
static TAILQ_HEAD(tailq_head, softbus_data) head;

static struct thread softbus_thread;

static void softbus_thread_schedule()
{
    thread_schedule(&edge_ctx.s, &softbus_thread);
}

static void softbus_thread_instance(struct thread *t)
{
    int processed = 0;
    struct listnode *node;
    struct softbus_sink *sink;
    struct softbus_chain **chain;
    struct softbus_data *data, *temp;

    TAILQ_FOREACH_SAFE(data, &head, entry, temp) {
        /* more sanity check */
        if (!data) {
            ULOG_ERR("data is nil");
            continue;
        }

        ULOG_DEBUG("chain '%s'\n", data->addr);

        chain = (struct softbus_chain **)map_get(&map_chain, data->addr);
        if (chain) {
            for (ALL_LIST_ELEMENTS_RO((*chain)->chain, node, sink)) {
                if (sink) {
                    ULOG_DEBUG(" -> %s\n", sink->addr);
                    sink->sink((*chain)->peer, (*chain)->source, data->ub);
                }
            }

            ubuf_free(data->ub);
        }

        TAILQ_REMOVE(&head, data, entry);
        free(data);

        processed++;

        /* so much time we already take, dont block here */
        if (processed > 100) {
            softbus_thread_schedule();
            break;
        }
    }

    thread_complete(t);
}

int softbus_init(void)
{
    TAILQ_INIT(&head);
    map_init(&map_chain);

    memset(&softbus_thread, 0, sizeof(softbus_thread));
    softbus_thread.thread = softbus_thread_instance;
    thread_init(&softbus_thread);

    return 0;
}

void softbus_peer_init(struct softbus_peer *peer)
{
    //memset(peer, 0, sizeof(*peer));
    if (peer) {
        peer->source_list = list_new();
        peer->sink_list = list_new();
    }
}

struct softbus_peer *softbus_peer_new(char *name)
{
    struct softbus_peer *peer = calloc(sizeof(struct softbus_peer), 1);

    if (peer && name)
        /* not safe */
        strcpy(peer->name, name);

    softbus_peer_init(peer);

    return peer;
}

void softbus_peer_add_source(struct softbus_peer *peer, char *addr, int priority)
{
    struct softbus_source *source = calloc(sizeof(struct softbus_source), 1);
    if (!source) {
        ULOG_ERR("out of memory\n");
        return;
    }

    /* not safe */
    strcpy(source->addr, addr);

    listnode_add(peer->source_list, source);
    ULOG_DEBUG("add source '%s' for peer\n", addr);
}

void softbus_peer_add_sink(struct softbus_peer *peer, char *addr, int priority, int (*func)(struct softbus_peer *peer, struct softbus_source *source, struct ubuf *ub))
{
    struct softbus_sink *sink = calloc(sizeof(struct softbus_sink), 1);
    if (!sink) {
        ULOG_ERR("out of memory\n");
        return;
    }

    /* not safe */
    strcpy(sink->addr, addr);

    sink->peer = peer;
    sink->sink = func;

    listnode_add(peer->sink_list, sink);
    ULOG_DEBUG("add sink '%s' for peer\n", addr);
}


void softbus_add_map_chain(struct softbus_chain *chain, struct softbus_sink *sink)
{
    listnode_add(chain->chain, sink);
    ULOG_DEBUG("insert '%s(%s)' into chain '%s'\n", sink->peer->name, sink->addr, chain->source->addr);
}

void softbus_create_source_chain(struct softbus_peer *peer, struct softbus_source *source)
{
    struct softbus_chain *chain = calloc(sizeof(struct softbus_chain), 1);
    /* memory leak ? */
    if (!chain) {
        ULOG_ERR("out of memory\n");
        return;
    }

    chain->chain = list_new();

    if (!chain->chain) {
        ULOG_ERR("out of memory\n");
        goto free_chain;
    }

    chain->peer = peer;
    chain->source = source;
    map_set(&map_chain, source->addr, chain);
    ULOG_DEBUG("create chain for source '%s'\n", source->addr);

    return;

free_chain:
    free(chain);
//free_source:
    /* we dont have the source ownship, leave peer will process it*/
    //free(source);
}

int softbus_peer_register(struct softbus_peer *peer)
{
    const char *key;
    struct list *head = &unmaped_sink;
    struct listnode *node, *temp;
    struct softbus_source *source;
    struct softbus_sink *sink, *maped_sink;
    struct softbus_chain **chain;
    map_iter_t iter = map_iter(&map_chain);

    /* add more sanity check later */
    for (ALL_LIST_ELEMENTS_RO(peer->source_list, node, source)) {
        chain = (struct softbus_chain **)map_get(&map_chain, source->addr);
        if (!chain)
            softbus_create_source_chain(peer, source);
        else {
            ULOG_ERR("source chain duplicated\n");
        }
    }

    for (ALL_LIST_ELEMENTS_RO(peer->sink_list, node, sink)) {
        chain = (struct softbus_chain **)map_get(&map_chain, sink->addr);
        if (chain) {
            //ULOG_DEBUG("insert sink '%s' into chain", sink->addr);
            softbus_add_map_chain(*chain, sink);
        }
        else {
            ULOG_DEBUG("unmaped sink '%s'\n", sink->addr);
            listnode_add(head, sink);
        }
    }

    /* the left sinks still not map into chain */
    for (ALL_LIST_ELEMENTS(head, node, temp, sink)) {
        chain = (struct softbus_chain **)map_get(&map_chain, sink->addr);
        /* dont loopback */
        if (chain) {
            softbus_add_map_chain(*chain, sink);
            listnode_delete(head, sink);
        }
        else {
            /* add to all chain except peer self */
            if (strcmp(sink->addr, "all") == 0) {

                while ((key = map_next(&map_chain, &iter))) {
                    chain = (struct softbus_chain **)map_get(&map_chain, key);
                    //ULOG_DEBUG("> chain '%s'", key);
                    if (list_isempty((*chain)->chain)) {
                        if ((*chain)->peer != sink->peer)
                            softbus_add_map_chain(*chain, sink);
                    } else if ((*chain)->peer != sink->peer) {
                        for (ALL_LIST_ELEMENTS((*chain)->chain, node, temp, maped_sink)) {
                            //ULOG_DEBUG(" %s -> %s -> %s", (*chain)->source->addr, maped_sink->addr, sink->addr);
                            if (maped_sink == sink)
                                continue;
                            softbus_add_map_chain(*chain, sink);
                        }
                    }
                }
            }
        }
    }

    return 0;
}

int softbus_peer_unregister(struct softbus_peer *peer)
{
    return 0;
}

void softbus_dump_remain_sink(void)
{
    int len = 0;
    char buf[1024] = {0};
    struct listnode *node;
    struct list *head = &unmaped_sink;
    struct softbus_sink *sink;

    for (ALL_LIST_ELEMENTS_RO(head, node, sink)) {
        if (!len)
            len += sprintf(buf + len, "%s(%s)", sink->peer->name, sink->addr);
        else
            len += sprintf(buf + len, ", %s(%s)", sink->peer->name, sink->addr);
    }

    ULOG_DEBUG("unmaped sink: %s\n", buf);
}

void softbus_dump_all_chain(void)
{
    int len;
    int chain_num = 0;
    int sink_num = 0;
    char buf[256] = {0};
    const char *key;
    struct listnode *node;
    struct softbus_sink *sink;
    struct softbus_chain **chain;
    map_iter_t iter = map_iter(&map_chain);

    ULOG_DEBUG("softbus routing chain\n");
    ULOG_DEBUG("--------------------------------------\n");
    while ((key = map_next(&map_chain, &iter))) {
        len = 0;
        chain_num++;
        ULOG_DEBUG("> chain '%s'\n", key);
        chain = (struct softbus_chain **)map_get(&map_chain, key);
        for (ALL_LIST_ELEMENTS_RO((*chain)->chain, node, sink)) {
            sink_num++;
            if (!len)
                len += sprintf(buf + len, "%s(%s)", sink->peer->name, sink->addr);
            else
                len += sprintf(buf + len, ", %s(%s)", sink->peer->name, sink->addr);
        }
        ULOG_DEBUG("  ^~~> %s\n", buf);
    }
    ULOG_DEBUG("--------------------------------------\n");

    ULOG_DEBUG("softbus chain total: %d\n", chain_num);
    ULOG_DEBUG("softbus sink total: %d\n", sink_num);
}

int softbus_push_data(char *addr, void *buf, int len)
{
    struct ubuf *ub;
    struct softbus_data *data;

    ub = ubuf_new(256);
    data = calloc(sizeof(struct softbus_data), 1);

    if (!ub || !data) {
        ULOG_ERR("out of memory\n");
        return -1;
    }

    memcpy(ubuf_put(ub, len), buf, len);
    /* dont change order above */
    data->ub = ub;
    /* not safe */
    strcpy(data->addr, addr);

    TAILQ_INSERT_TAIL(&head, data, entry);

    /* start thread schedule */
    softbus_thread_schedule();

    return 0;
}

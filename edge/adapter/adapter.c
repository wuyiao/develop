/* peripheral device adapter
 * writen by xuzhou
 */
#include <stdlib.h>
#include <assert.h>
#include <adapter.h>
#include <vector.h>
#include <edge.h>
#include <edev.h>
#include <linklist.h>

struct list adapter_list;

int adapter_register(struct edge_adapter *ada)
{
    listnode_add(&adapter_list, ada);

    return 0;
}

int adapter_unregister(struct edge_adapter *ada)
{
    listnode_delete(&adapter_list, ada);

    return 0;
}

struct edge_adapter *adapter_request(int type)
{
    struct list *head = &adapter_list;
    struct listnode *node;
    struct edge_adapter *ada;

    /* add more sanity check later */
    for (ALL_LIST_ELEMENTS_RO(head, node, ada))
        if (ada->obj.type == type)
            return ada;

    return NULL;
}

void *adapter_request_opi(struct edge_adapter *adapter)
{
    void *opi = NULL;

    switch (adapter->obj.type) {
        case GPIO:
            break;
        case SERIAL:
            break;
        case SPI:
            break;
        case UNKNOWN:
        default:
            break;
    }

    return opi;
}

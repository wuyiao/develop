#include <edge.h>
#include <lora.h>

int service_init(struct edge_context *edge)
{
    lora_service();
    //lorawan_service();


    return 0;
}

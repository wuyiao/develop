#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "context.h"

context_t* init_context()
{
    context_t *ctx = (context_t *)malloc(sizeof(context_t));

    if (ctx == NULL)
    {
        XERROR("malloc context failed\n");
        return NULL;
    }
    XDEBUG("context init success.\n");
    return ctx;
}

int uninit_context(context_t* ctx)
{
    if (ctx == NULL)
    {
        return 0;
    }
    
    free(ctx);
    ctx = NULL;

    return 0;
}
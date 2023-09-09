#include "component.h"
#include "usb.h"

int component_load(void)
{
    get_usb_devices();

    return 0;
}

#ifndef EDCD_USB_H
#define EDCD_USB_H
#include <libusb-1.0/libusb.h>

#define VENDOR_ID   1a86
#define PRODUCT_ID  7523
int get_usb_devices(void);
#endif

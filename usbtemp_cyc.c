#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define vendor_id 0x16c0
#define product_id 0x05dc

#define request_type 0xc0
#define request_kurz_status 0x01
#define request_lang_status 0x03
#define request_rescan 0x02
#define request_reset 0x04
#define value 0x00
#define index 0x00

/* table of devices that work with this driver */
static struct usb_device_id usbtemp_table[] = {
        { USB_DEVICE(vendor_id, product_id) },
        { }                      /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, usbtemp_table);




/* Structure to hold all of our device specific stuff */
struct usbtemp {
	struct usb_device	*udev;	/* the usb device for this device */
    struct usb_interface	*interface;		/* the interface for this device */
	unsigned char *ctrl_in_buffer;	/* the buffer to receive data */
    int temp1;
    int temp2;
    int supported_probes;
	__u8 int_in_endpointAddr;
};


static ssize_t usbtemp_kurz_status_show(struct device* dev,
                                    struct device_attribute * attribute,
                                    char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    int rc;

    usbtemp_dev->ctrl_in_buffer =  kzalloc(0x08, GFP_KERNEL);
    rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_kurz_status, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x08, 10000);
    if(rc < 0){
        pr_err("temp:send request-message failed\n");
    }
    else{
        usbtemp_dev->supported_probes = usbtemp_dev->ctrl_in_buffer[6] & 0xff;

    }
    kfree(usbtemp_dev->ctrl_in_buffer);

    return  sprintf(buf, "supported probes: %d\n", usbtemp_dev->supported_probes);
}
static SENSOR_DEVICE_ATTR(kurz_status, 0444, usbtemp_kurz_status_show, NULL, 0);

static ssize_t usbtemp_lang_status_show(struct device* dev,
                                    struct device_attribute * attribute,
                                    char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    int rc;

    usbtemp_dev->ctrl_in_buffer =  kzalloc(0x20, GFP_KERNEL);
    rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_lang_status, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x20, 10000);
    if(rc < 0){
        pr_err("temp:send request-message failed\n");
    }
    else{
        if(usbtemp_dev->ctrl_in_buffer[7] & 0x01){
            usbtemp_dev->temp1 = (usbtemp_dev->ctrl_in_buffer[8] & 0xff) + ((usbtemp_dev->ctrl_in_buffer[9] & 0xff) << 8);
        }
        else pr_err("temp:Sensor 1 nicht vorhanden\n");
        if(usbtemp_dev->ctrl_in_buffer[23] & 0x01){
            usbtemp_dev->temp2 = (usbtemp_dev->ctrl_in_buffer[24] & 0xff) + ((usbtemp_dev->ctrl_in_buffer[25] & 0xff) << 8);
        }
        else pr_err("temp:Sensor 2 nicht vorhanden\n");
    }
    kfree(usbtemp_dev->ctrl_in_buffer);

    return sprintf(buf, "temp1: %d \n temp2: %d\n", usbtemp_dev->temp1, usbtemp_dev->temp2);
}
static SENSOR_DEVICE_ATTR(lang_status, 0444, usbtemp_lang_status_show, NULL, 0);

static ssize_t usbtemp_temp1_show(struct device* dev,
                                    struct device_attribute * attribute,
                                    char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    int rc;

    usbtemp_dev->ctrl_in_buffer =  kzalloc(0x20, GFP_KERNEL);
    rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_lang_status, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x20, 10000);
    if(rc < 0){
        pr_err("temp:send request-message failed\n");
    }
    else{
        if(usbtemp_dev->ctrl_in_buffer[7] & 0x01){
            usbtemp_dev->temp1 = (usbtemp_dev->ctrl_in_buffer[8] & 0xff) + ((usbtemp_dev->ctrl_in_buffer[9] & 0xff) << 8);
        }
        else pr_err("temp:Sensor 1 nicht vorhanden\n");
    }

    kfree(usbtemp_dev->ctrl_in_buffer);

    return sprintf(buf, "temp1: %d \n ", usbtemp_dev->temp1);
}
static SENSOR_DEVICE_ATTR(temp1_input, 0444, usbtemp_temp1_show, NULL, 0);

static ssize_t usbtemp_temp2_show(struct device* dev,
                                    struct device_attribute * attribute,
                                    char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    int rc;

    usbtemp_dev->ctrl_in_buffer =  kzalloc(0x20, GFP_KERNEL);
    rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_lang_status, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x20, 10000);
    if(rc < 0){
        pr_err("temp:send request-message failed\n");
    }
    else{
        if(usbtemp_dev->ctrl_in_buffer[23] & 0x01){
            usbtemp_dev->temp2 = (usbtemp_dev->ctrl_in_buffer[24] & 0xff) + ((usbtemp_dev->ctrl_in_buffer[25] & 0xff) << 8);
        }
        else pr_err("temp:Sensor 2 nicht vorhanden\n");
    }

    kfree(usbtemp_dev->ctrl_in_buffer);

    return   sprintf(buf, "temp2: %d \n ", usbtemp_dev->temp2);
}
static SENSOR_DEVICE_ATTR(temp2_input, 0444, usbtemp_temp2_show, NULL, 0);

static ssize_t usbtemp_rescan_store(struct device* dev,
                                    struct device_attribute * attribute,
                                    const char *buf, size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
	struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    unsigned long * val;
    int ret;
    int rc;
    // if 1 is passed, rescan temperature sensors
    // and write all values to s_data->tempX
    // up to the number of available temperature sensors
    ret = kstrtoul(buf, 10, val);
    if(ret) return ret;
    if(*val == 1){
         usbtemp_dev->ctrl_in_buffer =  kzalloc(0x08, GFP_KERNEL);
         rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_rescan, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x08, 10000);
         if(rc < 0){
             pr_err("temp:send rescan-message failed\n");
         }
         else{
             if( *usbtemp_dev->ctrl_in_buffer == 23) pr_info("temp:reset successed\n");
             if( *usbtemp_dev->ctrl_in_buffer == 42) pr_info("temp:reset failed\n");
             if( *usbtemp_dev->ctrl_in_buffer != 23 &&  *usbtemp_dev->ctrl_in_buffer != 42 ) pr_err("temp:rescan-message answer is wrong\n");
         }
         kfree(usbtemp_dev->ctrl_in_buffer);
    }
    return count;
}
static SENSOR_DEVICE_ATTR(rescan, 0200, NULL, usbtemp_rescan_store, 0);

static ssize_t usbtemp_reset_store(struct device* dev,
                                    struct device_attribute * attribute,
                                    const char *buf, size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
	struct usbtemp *usbtemp_dev = usb_get_intfdata(intf);
    unsigned long * val;
    int ret;
    int rc;

    // if 1 is passed, rescan temperature sensors
    // and write all values to s_data->tempX
    // up to the number of available temperature sensors
    ret = kstrtoul(buf, 10, val);
    if(ret) return ret;
    if(*val == 1){
         usbtemp_dev->ctrl_in_buffer =  kzalloc(0x08, GFP_KERNEL);
         rc =  usb_control_msg(usbtemp_dev->udev, usb_rcvctrlpipe(usbtemp_dev->udev,0), request_reset, request_type, value, index, usbtemp_dev->ctrl_in_buffer, 0x08, 10000);
         if(rc < 0){
             pr_err("temp:reset failed\n");
         }
         else{
             pr_info("temp:reset successed\n");
         }
         kfree(usbtemp_dev->ctrl_in_buffer);
    }
    return count;
}
static SENSOR_DEVICE_ATTR(reset, 0200, NULL, usbtemp_reset_store, 0);


static struct attribute *usb_temp_attrs[] = {
    &sensor_dev_attr_kurz_status.dev_attr.attr,
	&sensor_dev_attr_lang_status.dev_attr.attr,
    &sensor_dev_attr_temp1_input.dev_attr.attr,
    &sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_rescan.dev_attr.attr,
	&sensor_dev_attr_reset.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(usb_temp);


static int usbtemp_probe(struct usb_interface *interface,
		      const struct usb_device_id *id){
    struct usbtemp *dev;
    struct device *hwmon_dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint = NULL;
    int i;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = usb_get_intf(interface);

    /* find the one control endpoint of this device */
	iface_desc = interface->cur_altsetting;
        /* vielleicht hier ++i und Endpoint 0 einmal testen?
         * weil allerdings müssen alle Geräte mindestens den Endpunkt 0 bereitstellen, der für Control Transfers
         * benötigt wird (u.a. für die Konfiguration)
         * mache ich das spaeter
         * */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_is_int_in(endpoint)) {
			dev->int_in_endpointAddr = endpoint->bEndpointAddress;
			break;
		}
	}
	if (!dev->int_in_endpointAddr) {
		dev_err(&interface->dev, "Could not find int-in endpoint");
        usb_put_intf(interface);
        kfree(dev);
        return ENODEV;
	}

    /* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);
    //Registrieren eines USB-Gerätetreibers im System
	hwmon_dev = hwmon_device_register_with_groups(&interface->dev,
							dev->udev->product,
							dev, usb_temp_groups);
	return 0;
}

static void usbtemp_disconnect(struct usb_interface *interface)
{
	struct usbtemp *dev;

	dev = usb_get_intfdata(interface);

	hwmon_device_unregister(&interface->dev);

	usb_set_intfdata(interface, NULL);

	usb_put_dev(dev->udev);

	kfree(dev);

    usb_put_intf(interface);

	dev_info(&interface->dev, "USB Temp now disconnected\n");

}


static struct usb_driver usbtemp_driver = {
        .name        = "usbtemp",
        .probe       = usbtemp_probe,
        .disconnect  = usbtemp_disconnect,
        .id_table    = usbtemp_table,
};

//Anschließen von USB-Geräten
static int __init usbtemp_init(void)
{
        int result;

        /* register this driver with the USB subsystem */
        result = usb_register(&usbtemp_driver);
        if (result < 0) {
                pr_err("usb_register failed for the %s driver. Error number %d\n",
                       usbtemp_driver.name, result);
                return -1;
        }

        return 0;
}

//Entfernen von USB-Geräten
static void __exit usbtemp_exit(void)
{
        /* deregister this driver with the USB subsystem */
        usb_deregister(&usbtemp_driver);
}

module_init(usbtemp_init);
module_exit(usbtemp_exit);


MODULE_AUTHOR("gruppe2-SRA-2022");
MODULE_DESCRIPTION("ds1820tousb driver");
MODULE_LICENSE("GPL");
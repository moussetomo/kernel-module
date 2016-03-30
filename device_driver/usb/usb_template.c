/*
 * 
 * Implement USB portion of rtl8139 Network Card driver
 * Use this as a template. Add code in areas matching string "CODE HERE".  
 * In this phase of the project we will be writing USB routines. 
 * Compile the driver as module. Use the makefile provided earlier
 * Make sure to unload the production module: 
 * # lsmod|grep rtl8150 
 * # rmmod rtl8150  
 * # lsmod 
 * Load the driver and run "ifconfig -a", You should see the MAC 
 * Address read from the device memory. 
 *
 * Guidelines are provided to assist you with writing a usb portion of the 
 * driver. Do not limit yourself to it. You are encouraged to review source 
 * code of production driver. 
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <asm/uaccess.h>

#include "rtl8150.h"

#define VENDOR_ID_REALTEK		0x0bda
#define PRODUCT_ID_RTL8150		0x8150

#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth

// Table of devices that work with this driver 
static struct usb_device_id rtl8150_table[] = {
	{USB_DEVICE(VENDOR_ID_REALTEK, PRODUCT_ID_RTL8150)},
	{}
};

/** 
  * This marks the usb_device_id table in the module image. This information 
  * loads the module on demand when the USBcard is inserted into  USB slot. 
  * It is part of Module auotload mechanism supported in Linux
  */

MODULE_DEVICE_TABLE(usb, rtl8150_table);

static struct usb_driver rtl8150_driver = {
	.name =		DRV_NAME,
	.id_table =	rtl8150_table,
	.probe =	rtl8150_probe,
	.disconnect =	rtl8150_disconnect
};

static struct net_device_ops rtl8150_netdev_ops = {
        .ndo_open               = rtl8150_open,
        .ndo_stop               = rtl8150_close,
        .ndo_get_stats          = rtl8150_get_stats,
        .ndo_start_xmit         = rtl8150_start_xmit
};

/********* USB ROUTINES*************/

static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id)
{
        struct net_device *netdev;
        rtl8150_t *priv;
		struct usb_device *udev;

	/* extract usb_device from the usb_interface structure */
	udev = interface_to_usbdev(intf);

	/**
          * Linux Network Stack works with network device not the USB device. 
          * We need to allocate an ethernet network device. alloc_etherdev() 
          * allocates net device structure with memory allocated for 
          * rtl8150 private structure. Return ENOMEM if failed
	  */
	netdev = alloc_etherdev(sizeof(*priv));
	if(!netdev) return -ENOMEM;

	/** 
    * Set the device name to DRV_NAME instead of eth via memcpy 
	*
    * What routine provides access to device's private structure from the 
    * net_device instance 
    */
	priv = netdev_priv(netdev);
	memcpy(netdev->name, DRV_NAME, sizeof(DRV_NAME));

    /* sysfs stuff. Sets up device link in /sys/class/net/interface_name */
    SET_NETDEV_DEV(netdev, &intf->dev);

    netdev->netdev_ops = &rtl8150_netdev_ops;

	/* Initialize Device private structure and initialize the spinlock*/
	spin_lock_init(&priv->lock);
	priv->udev = udev;
	priv->netdev = netdev;


	/* Register netdevice. If fail goto out  */
	if(register_netdev(netdev) != 0) {
		goto out;
	}

        /**
          * You can stuff device private structure into USB Interface 
	  * structure using usb_set_intfdata. That can be retrieved later 
	  * using usb__get_drvdata, for example, in disconnect or other driver 
	  * functions
          */
	usb_set_intfdata(intf, priv);

	/**
          * Update net device struct with MAC and Broadcast address
          * RealTek8150 datasheet - page 17 states that IDR registers, IDR0-5, 
          * found at device memory location 0x0120 contains MAC Address of the
	  * the NIC device
	  * 
	  * Fill net device netdev->dev_addr[6] and netdev->broadcast[6] 
	  * arrays.  For broadcast address fill all octets with 0xff. 
	  * 
	  * In USB, you don't map device memory, instead you submit
	  * control urb to USB core when needs to read/write device register memory. 
	  * 
	  * The structure that sends the control request to a USB device has to 
	  * conform to USB specification (chapter 9) and is defined in 
	  *	include/linux/usb/ch8.h
	  *
	  * struct usb_ctrlrequest {
	  *	__u8 bRequestType;
	  *	__u8 bRequest;
	  *	__le16 mValue;
	  *	__le16 wIndex;
	  *	__le16 wLength;
	  * } __attribute__ ((packed));
	  * 
	  * Read IDR memory location in device memory by submitting 
	  * usb_control_msg() to USB core
	  * int usb_control_msg(struct usb_device *dev, unsigned int pipe,
	  * 		__u8 request, __u8 requesttype, __u16 value, 
          *		__u16 index, void *data, __u16 size, int timeout);
	  * where: 
	  *   - struct usb_device *dev: pointer to the target USB device
	  *   - unsigned int pipe: endpoint of the target USB device where
          *	this message will be sent. This value is created  by calling
	  *	usb_snd|usb_rcvctrlpipe
	  *   - __u8 request: Requets value for the control message 
	  *		- vendor specific (RTL8150_REQ_GET_REGS)
	  *   - __u8 requesttype: Request type 
		        - vendor specific (RTL8150_REQT_READ)
	  *   - __u16 value: the USB message value for the control message
	  *	   driver uses it to specify the memory location in 
		   device memory such as IDR register location.
	  *   - __u16 indx: Desired offset.  driver sets it to 0 
	  *	            -  means start of the requested memory location 
	  *   - void *data: A pointer to the data to send to the device if this
	  *	 is an OUT endpoint. If this is an IN endpoint, this is a
	  *	 pointer to where the data should be placed after being read
	  *	 from the device. 
	  *   - __u16 size: size of the buffer that is pointed to by the data
	  *	 paramter above. It is a number of bytes to transfer
	  *   - int timeout: The amount of time, in jiffies, that should be
	  *	waited before timing out. If this value is 0, the function
	  *	will wait forever for the message to complete. For USB compliant
	  *	host, device requests with a data stage must start to return data
	  *	in 500ms after the request.
	  *
	  *   Return Value: on success, it returns the number of bytes that
	  *   are transferred to or from the device, othervise negative number
	  *		
	  *  Some parameters for usb_control_msg: request, requesttype, value, 
	  * indx,size map directly to the USB specification for how a USB 
	  * control message is defined.
	  *   	  
	  * usb_control_msg or usb_bulk_msg cannot be called from the 
	  * interrupt context. Also, this function cannot be cancelled by
	  * any other function, so be careful when using it; make sure that
	  * your driver disconnect function knows enough to wait for the call
	  * to complete before allowing itself to be unloaded from memory
	  * 
	  * Read six bytes into net_device structure member dev_addr from 
	  * device memory location IDR 
	  */ 
		usb_control_msg(priv->udev,
						usb_rcvctrlpipe(priv->udev, 0),
						RTL8150_REQ_GET_REGS,
						RTL8150_REQT_READ,
						IDR,
						0,
						priv->netdev->dev_addr,
						sizeof(priv->netdev->dev_addr),
						500
					   );

        /* Length of Ethernet frame. It is a "hardware header length", number 
         * of octets that lead the transmitted packet before IP header, or 
         * other protocol information.  Value is 14 for Ethernet interfaces.
         */

        netdev->hard_header_len = 14;

	return 0;

out:
        usb_set_intfdata(intf, NULL);
        free_netdev(netdev);
        return -EIO;
}

static void read_bulk_callback(struct urb *urb)
{
	rtl8150_t *priv;
	struct net_device *netdev;
	int res, pkt_len;
	// Get access to priv struct and status of urb
	int status = urb->status;
	priv = urb->context;
	if(!priv) return;
	netdev = priv->netdev;

	switch(status) {
		case 0:  break;
		case -ENOENT: return; /* urb is unlinked */
		case -ETIME:
			printk("\n Reschedule it. Could be a problem with device\n");
			goto reschedule;
		default:
			printk("\nRx status %d\n", status);
			goto reschedule;
	}

	// we come here after receiving success status of urb
	res = urb->actual_length; // amount of actual data received in the urb. Size of the packet
	pkt_len = res - 4;	// first 4 bytes contains Rx header and CRC..

	/**
	 * 1- Use skb_put to set skb->len and skb->tail to actual payload
	 * 2- Set protocol field in skb
	 * 3- Hand over the packet to protocol layer
	 * 4- Increment rx_packet and rx_bytes
	 */
	skb_put(priv->rx_skb, pkt_len); // update skb->len and skb->tail to point to actual payload

	priv->rx_skb->protocol = eth_type_trans(priv->rx_skb, netdev);
	netif_rx(priv->rx_skb); // hand over the sk_buff to the protocol layer
							// make sure don't touch the skb or free it
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += pkt_len;

	reschedule:	// submit another urb for next packet receive from the device

	/**
	 * Allocate sk_buff again and point priv->rx_skb to it
	 * populate bulk URB, Don't need to allocate urb, reuse it.
	 * submit urb.
	 * CAUTION: code is running in atomic or /interrupt context
	 */
	priv->rx_skb = dev_alloc_skb(RTL8150_MTU + 2);
	skb_reserve(priv->rx_skb, 2);
	usb_fill_bulk_urb(priv->rx_urb, priv->udev, usb_rcvbulkpipe(priv->udev, 1), priv->rx_skb->data, RTL8150_MTU, read_bulk_callback, priv);
	res = usb_submit_urb(priv->rx_urb, GFP_ATOMIC);
	return;
}

static void write_bulk_callback(struct urb *urb)
{
	rtl8150_t *priv;
	int status, res, pkt_len;

	printk(KERN_INFO "Entering %s\n", __FUNCTION__);
	// Get access to priv struct and status of urb
	priv = urb->context;
	if(!priv) return;

	status = urb->status;

	switch(status) {
		case 0: break;
		case -ENOENT: return; /* urb is in unlink state */
		case -ETIME:
			printk("\n Could be a problem with device\n");
			goto reenable;
		default:
			printk("\n%s: Tx status %d\n", priv->netdev->name, status);
			goto reenable;
	}

	res = urb->actual_length; // amount of actual data received in the urb. Size of the packet
	pkt_len = res - 4;	// first 4 bytes contains Rx header and CRC..

	// urb was transmitted successfully
	// increment tx_packets and tx_bytes
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += pkt_len;

	printk(KERN_INFO "\n%s: Queued Tx packet at %p size %u\n", priv->netdev->name, priv->tx_skb->data, priv->tx_skb->len);
	dev_kfree_skb_irq(priv->tx_skb); // free skb, atomic context

	/**
	 * Protocol layer should take case of error recovery.
	 * We should just enable the queue so that protocol layer
	 * continue to send skb_buff to us.
	 */
	reenable: // enable the queue

	netif_wake_queue(priv->netdev);
	return;
}

static void intr_callback(struct urb *urb)
{
	rtl8150_t *priv;
	int status;
	int res;
	__u8 *d;	// used for pointing to transfer_buffer

	// Get access to priv struct and status of urb
	priv = urb->context;
	if(!priv) return;
	status = urb->status;
	switch(status) {
		case 0: break;
		case -ECONNRESET: // urb is unlinked
		case -ENOENT:
		case -ESHUTDOWN:
			return;
		default:
			dev_info(&urb->dev->dev, "%s intr status %d\n", priv->netdev->name, status);
			goto resubmit;
	}

	// we get here when status is set to success
	d = urb->transfer_buffer;
	// Test for transmit errors
	if(d[0] & TSR_ERRORS) {
		priv->netdev->stats.tx_errors++;
		if(d[INT_TSR] & (TSR_ECOL | TSR_JBR))
			priv->netdev->stats.tx_aborted_errors++;
		if(d[INT_TSR] & TSR_LCOL)
			priv->netdev->stats.tx_window_errors++;
		if(d[INT_TSR] & TSR_LOSS_CRS)
			priv->netdev->stats.tx_carrier_errors++;
	}

	/* report link status changes to the network stack */
	if((d[INT_MSR] & MSR_LINK) == 0) {
		if(netif_carrier_ok(priv->netdev)) {
			netif_carrier_off(priv->netdev);
			printk("%s: LINK LOST\n", __FUNCTION__);
		}
	} else {
		if(!netif_carrier_ok(priv->netdev)) {
			netif_carrier_on(priv->netdev);
			printk("%s: LINK CAME BACK\n", __FUNCTION__);
		}
	}

	resubmit:
		res = usb_submit_urb(urb, GFP_ATOMIC);

		return;
}

static int rtl8150_open(struct net_device *netdev)
{
	rtl8150_t *priv;
	int res;

	printk(KERN_INFO "Entering %s\n", __FUNCTION__);
	/* Get the address of private structure from net_device */
	priv = netdev_priv(netdev);

	// set registers
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
					RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, IDR, 0,
					netdev->dev_addr, sizeof(netdev->dev_addr), 500);
	
	/* 
	 * Driver sets up pipes to bulk IN (EP1) endpoint for
	 * receiving packets, bulk OUT (EP2) for transmitting packets
	 * and to interrupt IN (EP3) endpoint for receiving device errors
	 * and link status from the device:
	 *
	 * 1- Allocate memory for bulk IN, OUT, and interrupt urb
	 * 2- Allocate memory for sk_buff for bulk IN urb -receive
	 * 3- Populate bulk IN urb and register call back function
	 * NOTE Make sure to allocate memory for intr_buff
	 * 4- Populate interrupt urb and register call back funciton
	 * 5- Submit bulk IN and interrupt urb
	 *
	 * NOTE: Driver submits urb to bulk OUT EP from rtl8150_start_xmit
	 * when the driver receives sk_buff from the protocol layer
	 */
	priv->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	priv->intr_urb = usb_alloc_urb(0, GFP_KERNEL);

	priv->rx_skb = dev_alloc_skb(RTL8150_MTU + 2);
	skb_reserve(priv->rx_skb, 2);

	usb_fill_bulk_urb(priv->rx_urb, priv->udev, usb_rcvbulkpipe(priv->udev, 1),
					priv->rx_skb->data, RTL8150_MTU, read_bulk_callback, priv);
	if((res = usb_submit_urb(priv->rx_urb, GFP_KERNEL))) {
		if(res == -ENODEV)
			netif_device_detach(priv->netdev);
		return res;
	}
	
	priv->intr_buff = kmalloc(INTBUFSIZE, GFP_KERNEL);
	priv->intr_interval = 100;
	usb_fill_int_urb(priv->intr_urb, priv->udev, usb_rcvintpipe(priv->udev, 3),
					priv->intr_buff, INTBUFSIZE, intr_callback,
					priv, priv->intr_interval);
	if((res = usb_submit_urb(priv->intr_urb, GFP_KERNEL))) {
		if(res == -ENODEV)
			netif_device_detach(priv->netdev);
			usb_kill_urb(priv->rx_urb);
	}

	printk(KERN_INFO "\n Submit Rx and Intr urbs\n");

	// Initialize the hardware to make sure it is ready
	rtl8150_hardware_start(netdev);

	/* Nofify the protocol layer so that it can start sending packet */
	netif_start_queue(netdev); /* transmission queue start */

	printk(KERN_INFO "Exiting %s\n", __FUNCTION__);
    return 0;
}

static int rtl8150_close(struct net_device *dev)
{
		u8 cr;
		rtl8150_t *priv;
        printk("rtl8150_close\n");
		// get address of private structure and ioaddr
		priv = netdev_priv(dev);
		// Notify protocol layer not to send any more packet to this interface
        netif_stop_queue(dev); /* transmission queue stop */
		printk(KERN_INFO "\n rtl8150 close: shutting down the interface");

		// get the value of CR register into cr using usb_control_msg
		usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_GET_REGS, RTL8150_REQT_READ, CR, 0, &cr, sizeof(cr), 500);
		cr &= 0xf3;

		// set the CR register to what is in cr using usb_control_msg
		usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &cr, sizeof(cr), 500);

		// unlink all urbs
		usb_kill_urb(priv->rx_urb);
		usb_kill_urb(priv->tx_urb);
		usb_kill_urb(priv->intr_urb);
        return 0;

}

static void rtl8150_hardware_start(struct net_device *netdev)
{
	u8 data = 0x10;
	u8 cr = 0x0c;
	u8 tcr = 0xd8;
	u8 rcr = 0x9e;
	short tmp;
	int i = HZ;
	rtl8150_t *priv;

	// get address of device private staructure
	priv = netdev_priv(netdev);

	printk("Entering %s \n", __FUNCTION__);
	
	// reset the chip. Make sure to wait for chip to reset
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &data, 1, 500);

	// confirm it that device has been reset successfully
	do {
		usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &data, 1, 500);
	} while((data & 0x10) && --i);

	printk(KERN_INFO "\n DEVICE IS RESET SUCCESSFULLY\n");

	// Set RCR, TCR and CR registers to values in rcr, tcr and cr
	// using usb_control_msg()
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, RCR, 0, &rcr, 1, 500);
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, TCR, 0, &tcr, 1, 500);
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &cr, 1, 500);

	// Read CS Configuration Register(CSCR) register value in tmp,
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0), RTL8150_REQ_GET_REGS, RTL8150_REQT_READ, CSCR, 0, &tmp, 2, 500);

	// Check Link status
	if(tmp & CSCR_LINK_STATUS)
		netif_carrier_on(netdev);
	else
		netif_carrier_off(netdev);

	printk(KERN_INFO "\n DEVICE CARRIER SET SUCCESSFULLY\n");
}

static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int res;
	rtl8150_t *priv;
	printk("rtl8150_start_xmit\n");
	priv = netdev_priv(dev);
	// allocate memory for buld urb
    priv->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	
	// point tx_skb to the sk_buff received from the protocol engine
	priv->tx_skb = skb;

	// populate bulk urb and register call back function
	usb_fill_bulk_urb(priv->tx_urb, priv->udev, usb_sndbulkpipe(priv->udev, 2), skb->data, skb->len, write_bulk_callback, priv);

	if((res = usb_submit_urb(priv->tx_urb, GFP_ATOMIC))) {
		if(res == -ENODEV)
			netif_device_detach(priv->netdev);
		else {
			priv->stats.tx_errors++;
			netif_start_queue(dev);
		}
	} else {
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += skb->len;
	}
	//dev_kfree_skb(skb); /* Just free it for now */

    return NETDEV_TX_OK;
	
}

static struct net_device_stats* rtl8150_get_stats(struct net_device *dev)
{
	rtl8150_t *priv = netdev_priv( dev );

	printk("dev_get_stats: Add code later\n");

	/**
	* You cannot return NULL, make sure to return the address 
	* of net_dev_stat that is in device private structure
	*/
	return &(priv->stats);
}

/* USB disconnect routine - required else can't rmmod */
static void rtl8150_disconnect(struct usb_interface *intf)
{
	/* Get address of device private structure */
	rtl8150_t *priv = usb_get_intfdata(intf);

	if (priv) {
		/**
	  	* Unregister and free memory of net_device structure
	  	* Call usb_set_intfdata(intf, NULL) to free memory	
	  	*/
		unregister_netdev(priv->netdev);
		usb_set_intfdata(intf, NULL);
	}
}

/************* USB init and exit routines ***************/
static int __init usb_rtl8150_init(void)
{
	return usb_register(&rtl8150_driver);
}

static void __exit usb_rtl8150_exit(void)
{
	usb_deregister(&rtl8150_driver);
}

module_init(usb_rtl8150_init);
module_exit(usb_rtl8150_exit);

MODULE_AUTHOR("Masatomo Takai");
MODULE_DESCRIPTION("USB Driver for Realtek rtl8150 USB Ethernet Wired Card");
MODULE_LICENSE("Dual BSD/GPL");

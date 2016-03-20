/*
**	Pegasus: USB 10/100Mbps/HomePNA (1Mbps) Controller
**
**	Copyright (c) 1999,2000 Petko Manolov - Petkan (petkan@dce.bg)
**	
**
**	ChangeLog:
**		....	Most of the time spend reading sources & docs.
**		v0.2.x	First official release for the Linux kernel.
**		v0.3.0	Beutified and structured, some bugs fixed.
**		v0.3.x	URBifying bulk requests and bugfixing. First relatively
**			stable release. Still can touch device's registers only
**			from top-halves.
**		v0.4.0	Control messages remained unurbified are now URBs.
**			Now we can touch the HW at any time.
**		v0.4.9	Control urbs again use process context to wait. Argh...
**			Some long standing bugs (enable_net_traffic) fixed.
**			Also nasty trick about resubmiting control urb from
**			interrupt context used. Please let me know how it
**			behaves. Pegasus II support added since this version.
**			TODO: suppressing HCD warnings spewage on disconnect.
**		v0.4.13	Ethernet address is now set at probe(), not at open()
**			time as this seems to break dhcpd. 
*/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

 /*
 **
 **	ChangeLog of Realtek:
 **		1.0.0	First release for RTL8150 USB Fast Ethernet NIC
 **
 */


#include <linux/module.h>
#include <linux/sched.h>
#include <linux/malloc.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>


static const char *version = __FILE__ ": v1.0.0 2001/01/31 (C) 2001 Realtek Semiconductor Corp.(shwei@realtek.com.tw)";


/// define this to use interrupt endpoint 3
#define	RTL8150_USE_INTR	


#define	RTL8150_MTU			1500
#define	RTL8150_MAX_MTU			1536
#define RTL8150_MIN_PACKET_LENGTH	60


/// phy registers index
#define MII_PHY_ID1	0x02
#define MII_PHY_ID2	0x03


/// manufacture ID of HPNA PHY 
#define AMD_PHY_ID	0x6b90
#define NS_PHY_ID	0x5c20


/// rtl8150->features
#define	FEATURE_DEFAULT		0x00
#define	FEATURE_HAS_HOME_PNA	0x01


/// rtl8150 states
#define	RTL8150_PRESENT		0x00000001
#define	RTL8150_RUNNING		0x00000002
#define	RTL8150_TX_BUSY		0x00000004
#define	RTL8150_RX_BUSY		0x00000008
#define	RTL8150_UNPLUG		0x00000040


/// ctrl urb
#define	CTRL_URB_RUNNING	0x00000010
#define	CTRL_URB_SLEEP		0x00000020

//#define	ETH_REGS_CHANGE		0x40000000
#define	ETH_REGS_CHANGED	0x80000000

#define	REG_TIMEOUT		(HZ)
#define	RTL8150_TX_TIMEOUT	(HZ*10)


/// vendor memory read & write
#define	RTL8150_REQT_READ	0xc0
#define	RTL8150_REQT_WRITE	0x40
#define	RTL8150_REQ_GET_REG	0x05
#define	RTL8150_REQ_SET_REG	0x05


#define	NUM_CTRL_URBS		0x10
#define	ALIGN(x)		x __attribute__((aligned(L1_CACHE_BYTES)))


#define SIZE32_BIT0		0x00000001
#define SIZE32_BIT1		0x00000002
#define SIZE32_BIT2		0x00000004
#define SIZE32_BIT3		0x00000008
#define SIZE32_BIT4		0x00000010
#define SIZE32_BIT5		0x00000020
#define SIZE32_BIT6		0x00000040
#define SIZE32_BIT7		0x00000080
#define SIZE32_BIT8		0x00000100
#define SIZE32_BIT9		0x00000200
#define SIZE32_BIT10		0x00000400
#define SIZE32_BIT11		0x00000800
#define SIZE32_BIT12		0x00001000
#define SIZE32_BIT13		0x00002000
#define SIZE32_BIT14		0x00004000
#define SIZE32_BIT15		0x00008000
#define SIZE32_BIT16		0x00010000
#define SIZE32_BIT17		0x00020000
#define SIZE32_BIT18		0x00040000
#define SIZE32_BIT19		0x00080000
#define SIZE32_BIT20		0x00100000
#define SIZE32_BIT21		0x00200000
#define SIZE32_BIT22		0x00400000
#define SIZE32_BIT23		0x00800000
#define SIZE32_BIT24		0x01000000
#define SIZE32_BIT25		0x02000000
#define SIZE32_BIT26		0x04000000
#define SIZE32_BIT27		0x08000000
#define SIZE32_BIT28		0x10000000
#define SIZE32_BIT29		0x20000000
#define SIZE32_BIT30		0x40000000
#define SIZE32_BIT31		0x80000000


#define SIZE16_BIT0		0x0001
#define SIZE16_BIT1		0x0002
#define SIZE16_BIT2		0x0004
#define SIZE16_BIT3		0x0008
#define SIZE16_BIT4		0x0010
#define SIZE16_BIT5		0x0020
#define SIZE16_BIT6		0x0040
#define SIZE16_BIT7		0x0080
#define SIZE16_BIT8		0x0100
#define SIZE16_BIT9		0x0200
#define SIZE16_BIT10		0x0400
#define SIZE16_BIT11		0x0800
#define SIZE16_BIT12		0x1000
#define SIZE16_BIT13		0x2000
#define SIZE16_BIT14		0x4000
#define SIZE16_BIT15		0x8000


#define SIZE8_BIT0		0x01
#define SIZE8_BIT1		0x02
#define SIZE8_BIT2		0x04
#define SIZE8_BIT3		0x08
#define SIZE8_BIT4		0x10
#define SIZE8_BIT5		0x20
#define SIZE8_BIT6		0x40
#define SIZE8_BIT7		0x80


/// registers index
#define IDR0			0x0120
#define Command			0x012e
#define TxConfig		0x012f
#define RxConfig		0x0130
#define TxStst			0x0132
#define RxStst			0x0133
#define Config			0x0135
#define Config1			0x0136
#define MediaStst		0x0137
#define MiiPhyAddr		0x0138
#define MiiPhyData		0x0139
#define MiiPhyAccess		0x013b
#define GPPCtrl			0x013d
#define WakeUpEventCtrl		0x013e
#define BasicModeCtrl		0x0140
#define BasicModeStst		0x0142
#define AutoNegoAdver		0x0144
#define AutoNegoLinkPartner	0x0146
#define AutoNegoExpan		0x0148
#define NwayTst			0x014a
#define CSConfig		0x014c


/// which HPNA PHY
enum hpna_phy {
	UNKNOWN_PHY = 0,
	AMD_PHY = 1,
	NS_PHY = 2,
};


/// loopback mode
enum loopback_mode{
	NORMAL_OPERATION = 0,
	DIGITAL_LOOPBACK = 1,
	ANALOG_LOOPBACK = 2,
};


/// EEPROM index
#define		EPROM_IDR0	 	0x1202
#define 	EPROM_Config0   	0x1208
#define	  	EPROM_MSR		0x1209
#define 	EPROM_GPCP		0x120a
#define		EPROM_UDP		0x120b
#define		EPROM_ATTR		0x120c
#define		EPROM_PHY2_PARM		0x120d
#define		EPROM_PHY1_PARM		0x120e
#define		EPROM_TW1_PARM		0x1212
#define		EPROM_MAXPOR		0x1216
#define		EPROM_Interv		0x1217
#define		EPROM_LanguageID	0x1218
#define		EPROM_ManufactureID	0x121a
#define 	EPROM_ProductID		0x121c
#define		EPROM_SerialNumber	0x121e
#define		EPROM_ManufactureString	0x1228
#define		EPROM_ProductString	0x1250


/// Command Register
#define 	WEPROM			SIZE8_BIT5
#define 	SOFT_RST		SIZE8_BIT4
#define		RE			SIZE8_BIT3
#define		TE			SIZE8_BIT2
#define 	EP3CLEAR		SIZE8_BIT1
#define 	AUTOLOAD      		SIZE8_BIT0


/// Transmit Configuration Register
#define		TXRR1			SIZE8_BIT7
#define		TXRR0			SIZE8_BIT6
#define 	IFG1			SIZE8_BIT4
#define		IFG0			SIZE8_BIT3
#define 	LBK1			SIZE8_BIT2
#define 	LBK0			SIZE8_BIT1
#define		NOCRC			SIZE8_BIT0


/// Receive Configuration Register
#define 	EARRX			SIZE16_BIT10
#define		TAIL			SIZE16_BIT7
#define		AER		   	SIZE16_BIT6
#define		AR		    	SIZE16_BIT5
#define		AM		   	SIZE16_BIT4
#define		AB		    	SIZE16_BIT3
#define		AD		    	SIZE16_BIT2
#define		AAM		 	SIZE16_BIT1
#define		AAP		   	SIZE16_BIT0

/// Transmit Status Register
#define 	ECOL			SIZE8_BIT5
#define		LCOL			SIZE8_BIT4
#define 	LOSS_CRS		SIZE8_BIT3
#define		JBR			SIZE8_BIT2
#define 	TX_BUF_EMPTY		SIZE8_BIT1
#define		TX_BUF_FULL		SIZE8_BIT0

/// Receive Status Register
#define		WEVENT			SIZE8_BIT7
#define		RX_BUF_FULL		SIZE8_BIT6
#define		LKCHG			SIZE8_BIT5
#define 	RUNT			SIZE8_BIT4
#define		LONG			SIZE8_BIT3
#define 	CRC			SIZE8_BIT2
#define		FAE			SIZE8_BIT1
#define		ROK			SIZE8_BIT0

/// Configuration Register0
#define		SUSLED			SIZE8_BIT7
#define		PARM_EN			SIZE8_BIT6
#define		LDPS			SIZE8_BIT3
#define		MSEL			SIZE8_BIT2
#define		LEDS1			SIZE8_BIT1
#define 	LEDS0			SIZE8_BIT0

/// Configuration Register1
#define 	BWF			SIZE8_BIT6
#define		MWF			SIZE8_BIT5
#define		UWF			SIZE8_BIT4
#define		LONGWF1			SIZE8_BIT1
#define		LONGWF0			SIZE8_BIT0

/// Media Status Register
#define		TXFCE			SIZE8_BIT7
#define		RXFCE			SIZE8_BIT6
#define		DUPLEX			SIZE8_BIT4
#define		SPEED_100		SIZE8_BIT3
#define 	LINK			SIZE8_BIT2
#define		TXPF			SIZE8_BIT1
#define		RXPF			SIZE8_BIT0


typedef struct rtl8150 {
	struct usb_device	*usb; // store "dev" passed in rtl8150_probe( struct usb_device *dev...)
	struct net_device	*net; // store "net" returned by init_etherdev()
	struct net_device_stats	stats;
	unsigned		flags;
	unsigned		features;
	int			intr_interval;
	struct urb		ctrl_urb, rx_urb, tx_urb, intr_urb;
	devrequest		dr;
	wait_queue_head_t	ctrl_wait;
	struct semaphore	ctrl_sem;
	unsigned char		ALIGN(rx_buff[RTL8150_MAX_MTU]);
	unsigned char		ALIGN(tx_buff[RTL8150_MAX_MTU]);
	unsigned char		ALIGN(intr_buff[8]);
	__u8			eth_regs[4];
	__u8			phy;
	__u8			hpna_phy;
	__u8			gpio_res;
	
	/// added by Owen on 01/11/2000
	__u8			get_registers_flag;
	__u8			set_registers_flag;
} rtl8150_t;


struct usb_eth_dev {
	char	*name;
	__u16	vendor;
	__u16	device;
	__u32	private; /* LSB is gpio reset value */
};


static int loopback = 0;
static int mii_mode = 0;
static int multicast_filter_limit = 32;


MODULE_AUTHOR("shwei@realtek.com.tw");
MODULE_DESCRIPTION("Realtek RTL8150 USB 10/100 Fast Ethernet Driver");
MODULE_PARM(loopback, "i");
MODULE_PARM(mii_mode, "i");
MODULE_PARM_DESC(loopback, "Enable MAC loopback mode (bit 0)");
MODULE_PARM_DESC(mii_mode, "Enable HomePNA mode (bit 0) - default = MII mode = 0");


static struct usb_eth_dev usb_dev_id[] = {
	{"Realtek RTL8150 USB 10/100 Fast Ethernet Adatper", 0x0bda, 0x8150, FEATURE_DEFAULT},
	{"Realtek RTL8150 USB 1Mbps HomePNA", 0x0bda, 0x8151, FEATURE_HAS_HOME_PNA | FEATURE_DEFAULT},
	{NULL, 0, 0, 0}
};

static void set_registers_callback( urb_t *urb )
{
	rtl8150_t	*rtl8150 = urb->context;

	//info( "+set_registers_callback()");

	if ( !rtl8150 )
		return;

	switch ( urb->status ) {
		case USB_ST_NOERROR:
			break;
		case USB_ST_URB_PENDING:
			//info( " set_registers_callback: case USB_ST_URB_PENDING" );
			return;
		case USB_ST_URB_KILLED:
			break;
		default:
			//info( " set_registers_callback: default: status = 0x%x", urb->status );
			warn( __FUNCTION__ " status %d", urb->status);
	}
	rtl8150->flags &= ~ETH_REGS_CHANGED;
	if ( rtl8150->flags & CTRL_URB_SLEEP )
	{
		rtl8150->flags &= ~CTRL_URB_SLEEP;
		/// marked and added by Owen on 01/11/2001
		///wake_up_interruptible( &rtl8150->ctrl_wait );
		rtl8150->set_registers_flag = 0;
	}
	
	//info("-set_registers_callback()");
}


static void get_registers_callback( urb_t *urb )
{
	rtl8150_t	*rtl8150 = urb->context;

	///info( "+get_registers_callback()");

	if ( !rtl8150 )
		return;

	switch ( urb->status ) {
		case USB_ST_NOERROR:
			break;
		case USB_ST_URB_PENDING:
			//info( " get_registers_callback: case USB_ST_URB_PENDING" );
			return;
		case USB_ST_URB_KILLED:
			break;
		default:
			//info( " get_registers_callback: default: status = 0x%x", urb->status );
			warn( __FUNCTION__ " status %d", urb->status);
	}
	rtl8150->flags &= ~ETH_REGS_CHANGED;
	if ( rtl8150->flags & CTRL_URB_SLEEP ) 
	{
		rtl8150->flags &= ~CTRL_URB_SLEEP;
		/// marked and added by Owen on 01/11/2001
		///wake_up_interruptible( &rtl8150->ctrl_wait );
		rtl8150->get_registers_flag = 0;
	}
	
	///info("-get_registers_callback()");
}


static int get_registers(rtl8150_t *rtl8150, __u16 indx, __u16 size, void *data)
{
	int	ret;

	///info( "+get_registers()=====>" );

	rtl8150->dr.requesttype = RTL8150_REQT_READ;
	rtl8150->dr.request = RTL8150_REQ_GET_REG;
	rtl8150->dr.value = cpu_to_le16p(&indx);
	rtl8150->dr.index = 0;
	rtl8150->dr.length = 
	rtl8150->ctrl_urb.transfer_buffer_length = cpu_to_le16p(&size);

	FILL_CONTROL_URB( &rtl8150->ctrl_urb, rtl8150->usb,
			  usb_rcvctrlpipe(rtl8150->usb,0),
			  (char *)&rtl8150->dr,
			  data, size, get_registers_callback, rtl8150 );

	if ( (ret = usb_submit_urb( &rtl8150->ctrl_urb )) ) {
		info( " get_registers: usb_submit_urb failed" );	
		err( __FUNCTION__ " BAD CTRLs %d", ret);
		goto out;
	}
	rtl8150->flags |= CTRL_URB_SLEEP;
	/// marked by Owen on 01/11/2001
	//interruptible_sleep_on( &rtl8150->ctrl_wait );
out:
	///info( "-get_registers()<=====" );
	return	ret;
}


static int set_registers(rtl8150_t *rtl8150, __u16 indx, __u16 size, void *data)
{
	int	ret;

	///info( "+set_registers() =====>" );

	/// marked by Owen on 01/11/2001
	//if ( rtl8150->flags & ETH_REGS_CHANGED ) {
	//	rtl8150->flags |= CTRL_URB_SLEEP ;
	//	interruptible_sleep_on( &rtl8150->ctrl_wait );
	//}
	rtl8150->dr.requesttype = RTL8150_REQT_WRITE;
	rtl8150->dr.request = RTL8150_REQ_SET_REG;
	rtl8150->dr.value = cpu_to_le16p( &indx );
	rtl8150->dr.index = 0;
	rtl8150->dr.length = rtl8150->ctrl_urb.transfer_buffer_length = cpu_to_le16p( &size );

	FILL_CONTROL_URB( &rtl8150->ctrl_urb, rtl8150->usb,
			  usb_sndctrlpipe(rtl8150->usb,0),
			  (char *)&rtl8150->dr,
			  data, size, set_registers_callback, rtl8150 );

	if ( (ret = usb_submit_urb( &rtl8150->ctrl_urb )) ) {
		info( " set_registers: usb_submit_urb failed" );
		err( __FUNCTION__ " BAD CTRL %d", ret);
		return	ret;
	}
	rtl8150->flags |= CTRL_URB_SLEEP;
	/// marked by Owen on 01/11/2001
	//interruptible_sleep_on( &rtl8150->ctrl_wait );
	
	///info( "-set_registers()<=====" );

	return	ret;
}



static int read_phy_word( rtl8150_t *rtl8150, __u8 phy, __u8 indx, __u16 *regd )
{
	int	i;
	__u8	MiiPhyAccessContent;

	/*
	set_register( rtl8150, MiiPhyAccess, 0 );
	set_register( rtl8150, MiiPhyAddr, phy );
	set_register( rtl8150, MiiPhyAccess, (0x40 | indx) );
	for (i = 0; i < REG_TIMEOUT; i++) 
	{
		get_registers( rtl8150, MiiPhyAccess, 1, &MiiPhyAccessContent );
		if ( (MiiPhyAccessContent & 0x40) == 0)
			break;
	}
	if ( i < REG_TIMEOUT ) 
	{
		get_registers( rtl8150, MiiPhyData, 2, regd );
		return	0;
	}
	warn( __FUNCTION__ " failed" );
	*/
	
	return 1;
}



static int write_phy_word( rtl8150_t *rtl8150, __u8 phy, __u8 indx, __u16 regd )
{
	int	i;
	__u8	data[2] = { 0, 0 };
	__u8	MiiPhyAccessContent;
	
	//info( "write_phy_word() =====>" );

	/*
	*data = cpu_to_le16p( &regd );
	set_register( rtl8150, MiiPhyAccess, 0 );
	set_register( rtl8150, MiiPhyAddr, phy );
	set_registers( rtl8150, MiiPhyData, 2, data );
	set_register( rtl8150, MiiPhyAccess, (0x60 | indx) );
	for (i = 0; i < REG_TIMEOUT; i++) 
	{
		get_registers( rtl8150, MiiPhyAccess, 1, &MiiPhyAccessContent );
		if ( (MiiPhyAccessContent & 0x40) == 0)
			break;
	}

	if ( i < REG_TIMEOUT )
		return	0;
	
	warn( __FUNCTION__ " failed" );
	*/

	return 1;
}



static int read_eprom_word( rtl8150_t *rtl8150, __u8 index, __u16 *retdata )
{
	int	i, tmp;

	//info( "+read_eprom_word() =====>" );

	while( rtl8150->get_registers_flag == 1 )
		;
	rtl8150->get_registers_flag = 1;
	get_registers( rtl8150, index, 2, retdata );
	while( rtl8150->get_registers_flag == 1 )
		;
	
	//info( "-read_eprom_word() <=====" );
	return	0;
}



static int read_eprom_byte( rtl8150_t *rtl8150, __u16 index, __u8 *retdata )
{
	int	i, tmp;

	//info( "+read_eprom_byte() =====>" );

	while( rtl8150->get_registers_flag == 1 )
		;
	rtl8150->get_registers_flag = 1;
	get_registers( rtl8150,  index, 1, retdata );
	while( rtl8150->get_registers_flag == 1 )
		;
	
	//info( "-read_eprom_byte() <=====");

	return 0;
}

#ifdef	RTL8150_WRITE_EEPROM
static inline void enable_eprom_write( rtl8150_t *rtl8150 )
{
	__u8	tmp;

	//info( "+enable_eprom_write() =====>" );

	//info( "-enable_eprom_write() <=====" );
}


static inline void disable_eprom_write( rtl8150_t *rtl8150 )
{
	__u8 	tmp;

	//info( "+disable_eprom_write() =====>" );
	
	//info( "-disable_eprom_write() <=====" );
}


static int write_eprom_word( rtl8150_t *rtl8150, __u8 index, __u16 data )
{
	int	i, tmp;
	__u8	d[4] = {0x3f, 0, 0, EPROM_WRITE};

	//info( "+write_eprom_word() =====>" );

	/*
	set_registers( rtl8150, EpromOffset, 4, d );
	enable_eprom_write( rtl8150 );
	set_register( rtl8150, EpromOffset, index );
	set_registers( rtl8150, EpromData, 2, &data );
	set_register( rtl8150, EpromCtrl, EPROM_WRITE );

	for ( i=0; i < REG_TIMEOUT; i++ ) {
		get_registers( rtl8150, EpromCtrl, 1, &tmp );
		if ( tmp & EPROM_DONE )
			break;
	}
	disable_eprom_write( rtl8150 );
	if ( i < REG_TIMEOUT )
		return	0;
	warn( __FUNCTION__ " failed" );
	*/
	//info( "-write_eprom_word <=====");
	return	-1;
}
#endif	/* RTL8150_WRITE_EEPROM */


static inline void get_node_id( rtl8150_t *rtl8150, __u8 *id )
{
	int	i;

	info( "+get_node_id() =====>" );
	for (i=0; i < 6; i++)
	{
		while( rtl8150->get_registers_flag == 1 )
			;
		rtl8150->get_registers_flag = 1;
		get_registers( rtl8150, IDR0+i, 1, &id[i] );
		while( rtl8150->get_registers_flag == 1 )
			;
	}
	info( "-get_node_id() <=====" );
}


static void set_ethernet_addr( rtl8150_t *rtl8150 )
{
	__u8	node_id[6];

	info( "+set_ethernet_addr() =====>" );
	get_node_id(rtl8150, node_id);
	memcpy( rtl8150->net->dev_addr, node_id, sizeof(node_id) );
	info( " set_ethernet_addr: node id = %x %x %x %x %x %x",
		node_id[0], node_id[1], node_id[2], node_id[3], node_id[4], node_id[5] );
	info( "-set_ethernet_addr() <======" );
}


static inline int reset_mac( rtl8150_t *rtl8150 )
{
	__u8	data = 0;
	int	i;

	info( "+reset_mac() =====>" );
	/// software reset
	data = SOFT_RST;

	///info( " reset_mac: before set_register(rtl8150, Command, data)" );
	while( rtl8150->set_registers_flag == 1 )
		;
	rtl8150->set_registers_flag = 1;
	set_registers(rtl8150, Command, 1, &data);
	while( rtl8150->set_registers_flag == 1 )
		;
	
	///info( " reset_mac: after set_register( rtl8150, Command, data)" );
	for (i = 0; i < REG_TIMEOUT; i++)
	{
		while( rtl8150->get_registers_flag == 1 )
			;
		rtl8150->get_registers_flag = 1;
		get_registers(rtl8150, Command, 1, &data);
		while( rtl8150->get_registers_flag == 1 )
			;
		if ( ~(data & SOFT_RST) )
		{
			break;
		}
	}
	if ( i == REG_TIMEOUT )
	{
		info( " reset_mac: software reset failed" );
		return 1;
	}
	
	info( "-reset_mac() <=====" );
	return	0;
}


static int enable_net_traffic( struct net_device *dev, struct usb_device *usb )
{
	__u16	linkpart, bmsr;
	__u8	data[4];
	__u8	byte_tmp = 0;
	rtl8150_t *rtl8150 = dev->priv;
	__u16  	rx_config_content;

	info("+enable_net_traffic() =====>");
	
	/// check if link okay!
	while( rtl8150->get_registers_flag == 1 )
		;
	rtl8150->get_registers_flag = 1;
	get_registers( rtl8150, MediaStst, 1, &byte_tmp );
	while( rtl8150->get_registers_flag == 1 )
		;
	info(" enable_net_traffic: MediaStst = 0x%x", byte_tmp);
	if ( byte_tmp & LINK )
		info(" enable_net_traffic: LINK ON");
	else
		info(" enable_net_traffic: LINK OFF");

	
	/// software reset
	/// byte_tmp = SOFT_RST;
	///set_register( rtl8150, Command, 1, &byte_tmp );
	
	/// set Transmit Configuration Register
	///info(" enable_net_traffic: before setting Tx Config");
	while( rtl8150->set_registers_flag == 1 )
		;
	rtl8150->set_registers_flag = 1;
	byte_tmp = TXRR1 | TXRR0 | IFG1 | IFG0;
	set_registers( rtl8150, TxConfig, 1, &byte_tmp);
	while( rtl8150->set_registers_flag == 1)
		;
	///while( rtl8150->get_registers_flag == 1 )
	///	;
	///rtl8150->get_registers_flag = 1;
	///get_registers( rtl8150, TxConfig, 1, &byte_tmp );
	///while( rtl8150->get_registers_flag == 1 )
	///	;
	///info( "enable_net_traffic: TxConfig = 0x%x", byte_tmp );
	///info(" enable_net_traffic: after setting Tx Config");

	/// set Receive Configuration Register
	///info(" enable_net_traffic: before setting Rx Config");
	while( rtl8150->set_registers_flag == 1 )
		;
	rtl8150->set_registers_flag = 1;
	rx_config_content = AM | AB | AD | AAM;
	set_registers( rtl8150, RxConfig, 2, &rx_config_content );
	while( rtl8150->set_registers_flag == 1 )
		;
	///info(" enable_net_traffic: after setting Rx Config");

	/// enable Tx and Rx
	///info(" enable_net_traffic: before enabling Tx and Rx" );
	while( rtl8150->set_registers_flag == 1 )
		;
	rtl8150->set_registers_flag = 1;
	byte_tmp = RE | TE;
	set_registers( rtl8150, Command, 1, &byte_tmp );
	while( rtl8150->set_registers_flag == 1 )
		;
	///info(" enable_net_traffic: after enabling Tx and Rx" );
	
	///while( rtl8150->get_registers_flag == 1 )
	///	;
	///rtl8150->get_registers_flag = 1;
	///get_register( rtl8150, Command, &byte_tmp);
	///while( rtl8150->get_registers_flag == 1 )
	///	;
	///info(" enable_net_traffic: Command = 0x%x", byte_tmp );

	info( "-enable_net_traffic() <=====");
	return 0;
}


static void read_bulk_callback( struct urb *urb )
{
	rtl8150_t *rtl8150 = urb->context;
	struct net_device *net;
	int count = urb->actual_length, res;
	int rx_status;
	struct sk_buff	*skb;
	__u16 pkt_len;

	//info( "+read_bulk_callback() =====>" );

	if ( !rtl8150 || !(rtl8150->flags & RTL8150_RUNNING) )
		return;

	net = rtl8150->net;
	if ( !netif_device_present(net) )
		return;

	if ( rtl8150->flags & RTL8150_RX_BUSY ) 
	{
		rtl8150->stats.rx_errors++;
		return;
	}
	rtl8150->flags |= RTL8150_RX_BUSY;

	switch ( urb->status ) {
		case USB_ST_NOERROR:
			break;
		case USB_ST_NORESPONSE:
			info( " read_bulk_callback: NO RESPONSE" );
			dbg( "reset MAC" );
			rtl8150->flags &= ~RTL8150_RX_BUSY;
			break;
		default:
			info( " read_bulk_callback: Rx status = 0x%x", urb->status);
			dbg( "%s: RX status %d", net->name, urb->status );
			goto goon;
	}
	
	pkt_len = count - 4;
	//info(" read_bulk_callback: packet length = 0x%x", pkt_len );
	//info(" read_bulk_callback: packet = %x %x %x %x %x %x %x %x %x %x %x %x", rtl8150->rx_buff[0], rtl8150->rx_buff[1],
	//	rtl8150->rx_buff[2], rtl8150->rx_buff[3], rtl8150->rx_buff[4], rtl8150->rx_buff[5],
	//	rtl8150->rx_buff[6], rtl8150->rx_buff[7], rtl8150->rx_buff[8], rtl8150->rx_buff[9],
	//	rtl8150->rx_buff[10], rtl8150->rx_buff[11] );

	if ( !(skb = dev_alloc_skb(pkt_len+2)) )   /// why "pkt_len + 2" 
		goto goon;

	skb->dev = net;
	skb_reserve(skb, 2);
	
	eth_copy_and_sum(skb, rtl8150->rx_buff, pkt_len, 0);
	skb_put(skb, pkt_len);
	/// in "Linux Device Drivers", the two statements above should be 
	///	"memcpy( skb_put( skb, pkt_len), rtl8150->rx_buff, pkt_len );"


	skb->protocol = eth_type_trans(skb, net);
	netif_rx(skb);	/// call this system call to transfer the "skb" to the upper layer
	rtl8150->stats.rx_packets++;
	rtl8150->stats.rx_bytes += pkt_len;

goon:
	FILL_BULK_URB( &rtl8150->rx_urb, rtl8150->usb,
			usb_rcvbulkpipe(rtl8150->usb, 1),
			rtl8150->rx_buff, RTL8150_MAX_MTU, 
			read_bulk_callback, rtl8150 );
	if ( (res = usb_submit_urb(&rtl8150->rx_urb)) )
	{
		info( " read_bulk_callback: usb_submit_urb failed" );
		warn( __FUNCTION__ " failed submint rx_urb %d", res);
	}
	rtl8150->flags &= ~RTL8150_RX_BUSY;
	
	//info( "-read_bulk_callback() <=====");
}


static void write_bulk_callback( struct urb *urb )
{
	rtl8150_t *rtl8150 = urb->context;

	///info( "+write_bulk_callback() =====>" );

	if ( !rtl8150 || !(rtl8150->flags & RTL8150_RUNNING) )
		return;

	if ( !netif_device_present(rtl8150->net) )
		return;
		
	if ( urb->status )
		info("%s: TX status %d", rtl8150->net->name, urb->status);

	netif_wake_queue( rtl8150->net );
	///info( "-write_bulk_callback() <=====" );
}



#ifdef	RTL8150_USE_INTR
static void intr_callback( struct urb *urb )
{
	rtl8150_t *rtl8150 = urb->context;
	struct net_device *net;
	__u8	*d;

	///info( "intr_callback() =====>" );

	if ( !rtl8150 )
		return;
	d = urb->transfer_buffer;
	net = rtl8150->net;
///	if ( d[0] & 0xfc ) {
///		rtl8150->stats.tx_errors++;
///		if ( d[0] & TX_UNDERRUN )
///			rtl8150->stats.tx_fifo_errors++;
///		if ( d[0] & (EXCESSIVE_COL | JABBER_TIMEOUT) )
///			rtl8150->stats.tx_aborted_errors++;
///		if ( d[0] & LATE_COL )
///			rtl8150->stats.tx_window_errors++;
///		if ( d[0] & (NO_CARRIER | LOSS_CARRIER) )
///			rtl8150->stats.tx_carrier_errors++;
///	}
	switch ( urb->status ) 
	{
		case USB_ST_NOERROR:
			break;
		case USB_ST_URB_KILLED:
			break;
		default:
			info("intr status %d", urb->status);
	}
}
#endif



static void rtl8150_tx_timeout( struct net_device *net )
{
	rtl8150_t *rtl8150 = net->priv;

	info( "+rtl8150_tx_timeout() =====>" );

	if ( !rtl8150 )
		return;
	
	usb_unlink_urb( &rtl8150->tx_urb );
	warn("%s: Tx timed out.", net->name);
	rtl8150->stats.tx_errors++;
	net->trans_start = jiffies;

	netif_wake_queue( net );
	info( "-rtl8150_tx_timeout() <=====" );
}



static int rtl8150_start_xmit( struct sk_buff *skb, struct net_device *net )
{
	rtl8150_t	*rtl8150 = net->priv;
	int 	count = ((skb->len) & 0x3f) ? skb->len : skb->len+1;
		/// increase 64n-length packet by one, because the host controller sometimes doesn't send
		/// 0-length usb packet to end the 64n-length packet.
	int 	res;

	//info( "+rtl8150_start_xmit() =====>" );

	netif_stop_queue( net );
		
	memcpy( rtl8150->tx_buff, skb->data, skb->len );
	
	if(count < RTL8150_MIN_PACKET_LENGTH)
		count = RTL8150_MIN_PACKET_LENGTH;
	FILL_BULK_URB( &rtl8150->tx_urb, rtl8150->usb,
			usb_sndbulkpipe(rtl8150->usb, 2),
			rtl8150->tx_buff, RTL8150_MAX_MTU, 
			write_bulk_callback, rtl8150 );
	rtl8150->tx_urb.transfer_buffer_length = count;

	//info( " rtl8150_start_xmit: packet length = 0x%x", count);	
	//info( " rtl8150_start_xmit: packet = %x %x %x %x %x %x %x %x %x %x %x %x", rtl8150->tx_buff[0], rtl8150->tx_buff[1],
	//	rtl8150->tx_buff[2], rtl8150->tx_buff[3], rtl8150->tx_buff[4], rtl8150->tx_buff[5],
	//	rtl8150->tx_buff[6], rtl8150->tx_buff[7], rtl8150->tx_buff[8], rtl8150->tx_buff[9],
	//	rtl8150->tx_buff[10], rtl8150->tx_buff[11]);
	rtl8150->tx_urb.transfer_flags |= USB_ASYNC_UNLINK;
	if ((res = usb_submit_urb(&rtl8150->tx_urb))) 
	{
		warn("failed tx_urb %d", res);
		rtl8150->stats.tx_errors++;
		netif_start_queue( net );
	}
	else 
	{
		rtl8150->stats.tx_packets++;
		rtl8150->stats.tx_bytes += skb->len;
		net->trans_start = jiffies;	/// record the time stamp
	}

	dev_kfree_skb(skb);
	
	//info( "-rtl8150_start_xmit() <=====" );	
	return 0;
}


/// this function is called when "ifconfig ethx" or "netstat"
static struct net_device_stats *rtl8150_netdev_stats( struct net_device *dev )
{
	//info( "+rtl8150_netdev_stats() =====>" );
	return &((rtl8150_t *)dev->priv)->stats;
	//info( "-rtl8150_netdev_stats() <=====" );
}



static inline void disable_net_traffic( rtl8150_t *rtl8150 )
{
	__u8	tmp = 0;

	info( "+disable_net_traffic() =====>" );
	set_registers( rtl8150, Command, 1, &tmp );
	info( "-disable_net_traffic() <=====" );
}



static inline void get_interrupt_interval( rtl8150_t *rtl8150 )
{
	__u8	data;

	info( "+get_interrupt_interval() =====>" );

	read_eprom_byte( rtl8150, EPROM_Interv, &data );
	rtl8150->intr_interval = data;
	info( "-get_interrupt_interval() <=====" );
}


/// this function is called when "ifconfig ethx xxx ....."
static int rtl8150_open(struct net_device *net)
{
	rtl8150_t *rtl8150 = (rtl8150_t *)net->priv;
	int	res;

	info( "+rtl8150_open() =====>" );

	MOD_INC_USE_COUNT;
	
	///info( " rtl8150_open: before enable_net_traffic()" );
	if ( (res = enable_net_traffic(net, rtl8150->usb)) ) 
	{
		info(" rtl8150_open: enable_net_traffic failed");
		err("can't enable_net_traffic() - %d", res);
		MOD_DEC_USE_COUNT;
		return -EIO;
	}

	/// initiate the rx operation
	info( " rtl8150_open: initiate the rx operation" );
	FILL_BULK_URB( &rtl8150->rx_urb, rtl8150->usb,
			usb_rcvbulkpipe(rtl8150->usb, 1),
			rtl8150->rx_buff, RTL8150_MAX_MTU, 
			read_bulk_callback, rtl8150 );
	if ( (res = usb_submit_urb(&rtl8150->rx_urb)) )
	{
		info(" rtl8150_open: usb_submit_urb failed" );
		warn( __FUNCTION__ " failed rx_urb %d", res );
	}

	/*
	/// initiate the interrupt operation
#ifdef	RTL8150_USE_INTR
	get_interrupt_interval( rtl8150 );
	FILL_INT_URB( &rtl8150->intr_urb, rtl8150->usb,
			usb_rcvintpipe(rtl8150->usb, 3),
			rtl8150->intr_buff, sizeof(rtl8150->intr_buff),
			intr_callback, rtl8150, rtl8150->intr_interval );
	if ( (res = usb_submit_urb(&rtl8150->intr_urb)) )
		warn( __FUNCTION__ " failed intr_urb %d", res);
#endif
	*/
	netif_start_queue( net );
	rtl8150->flags |= RTL8150_RUNNING;

	info( "-rt8150_open() <=====" );
	
	return 0;
}



/// this funciton is called when "ifconfig ethx down"
static int rtl8150_close( struct net_device *net )
{
	rtl8150_t	*rtl8150 = net->priv;

	info( "+rtl8150_close() =====>" );

	rtl8150->flags &= ~RTL8150_RUNNING;
	netif_stop_queue( net );
	if ( !(rtl8150->flags & RTL8150_UNPLUG) )
		disable_net_traffic( rtl8150 );

	usb_unlink_urb( &rtl8150->rx_urb );
	usb_unlink_urb( &rtl8150->tx_urb );
	usb_unlink_urb( &rtl8150->ctrl_urb );
	usb_unlink_urb( &rtl8150->intr_urb );

	MOD_DEC_USE_COUNT;

	info( "-rtl8150_close() <=====" );
	return 0;
}



static int rtl8150_ioctl( struct net_device *net, struct ifreq *rq, int cmd )
{
	__u16 *data = (__u16 *)&rq->ifr_data;
	rtl8150_t	*rtl8150 = net->priv;

	info( "+rtl8150_ioctl() =====> return -EOPNOTSUPP directly" );

	return -EOPNOTSUPP;

	switch(cmd) {
		case SIOCDEVPRIVATE:
			data[0] = rtl8150->phy;
		case SIOCDEVPRIVATE+1:
			read_phy_word(rtl8150, data[0], data[1]&0x1f, &data[3]);
			return 0;
		case SIOCDEVPRIVATE+2:
			if ( !capable(CAP_NET_ADMIN) )
				return -EPERM;
			write_phy_word(rtl8150, rtl8150->phy, data[1] & 0x1f, data[2]);
			return 0;
		default:
			return -EOPNOTSUPP;
	}
}


static void rtl8150_set_multicast( struct net_device *net )
{
	__u16	rcr_content;
	rtl8150_t *rtl8150 = net->priv;

	info( "+rtl8150_set_multicast() =====>" );

	netif_stop_queue(net);

	while( rtl8150->get_registers_flag == 1 )
		;
	rtl8150->get_registers_flag = 1;
	get_registers( rtl8150, RxConfig, 2, &rcr_content );
	while( rtl8150->get_registers_flag == 1 )
		;

	if (net->flags & IFF_PROMISC) 
	{
		rcr_content |= AAP;
		info(" %s: Promiscuous mode enabled", net->name);
	} 
	else if( (net->mc_count > multicast_filter_limit) || (net->flags & IFF_ALLMULTI) ) 
	{
		rcr_content |= AAM;
		rcr_content &= ~AAP;
		info(" %s set allmulti", net->name);
	} 
	else 
	{
		rcr_content &= ~AAM;
		rcr_content &= ~AAP;
		info(" %s: No Promiscuous and All Multicast", net->name);
		/// added by Owen, please refer to 14.14.2 of Linux Device Drivers
		/// if( net->mc_count==0 )
		/// 	ff_get_only_own_packets();
		/// else
		/// {
		///	struct dev_mc_list	*mcptr;
		/// 	ff_clear_mc_list();
		///	for( mc_ptr = net->mc_list; mc_ptr; mc_ptr = mc_ptr->next )
		///		ff_store_mc_address( mc_ptr->dmi_addr );
		//	ff_get_packets_in_multicast_list();
		/// }
	}

	while( rtl8150->set_registers_flag == 1 )
		;
	rtl8150->set_registers_flag = 1;
	set_registers( rtl8150, RxConfig, 2, &rcr_content );
	while( rtl8150->set_registers_flag == 1 )
		;
	
	netif_wake_queue(net);	
	info( "-rtl8150_set_multicast() <=====" );	
}


static int check_device_ids( __u16 vendor, __u16 product )
{
	int i=0;

	info( "+check_device_ids() =====>" );
	info( " check_device_ids: vendor id = 0x%x", vendor );
	info( " check_device_ids: device id = 0x%x", product );
	
	while ( usb_dev_id[i].name ) {
		if ( (usb_dev_id[i].vendor == vendor) && (usb_dev_id[i].device == product) )
		{
		 	info( " check_device_ids: BINGO!" );
		 	info( "-check_device_ids() <=====" );
			return i;
		}
		i++;
	}
	
	info( " check_device_ids: not our device");
	info( "-check_device_ids() <=====" );
	return	-1;
}


static void mii_HPNA_phy_probe( rtl8150_t *rtl8150 )
{
	int	i;
	__u16	tmp;

        /*
	for ( i=0; i < 32; i++ ) 
	{
		read_phy_word( rtl8150, i, MII_PHY_ID2, &tmp );
		tmp = tmp & 0xfff0;
		if( tmp == AMD_PHY_ID ) 
		{
			rtl8150->hpna_phy = AMD_PHY;
			rtl8150->phy = i;
			return;
		}
		else if( tmp == NS_PHY_ID )
		{
			rtl8150->hpna_phy = NS_PHY;
			rtl8150->phy = i;
			return;
		}
		else
			continue;	
	}

	rtl8150->phy = 0;
	rtl8150->hpna_phy = UNKNOWN_PHY;
	*/
}



static void * rtl8150_probe( struct usb_device *dev, unsigned int ifnum, const struct usb_device_id *id)
{
	struct net_device *net;
	rtl8150_t *rtl8150;
	int	dev_indx;

	info( "+rtl8150_probe() =====>" );

	///info( " rtl8150_probe: before check_deice_ids()" );
	if ( (dev_indx = check_device_ids(dev->descriptor.idVendor, dev->descriptor.idProduct)) == -1 ) 
	{
		info( " rtl8150_pbobe: this is not my device!" );
		return NULL;
	}
	///info( " rtl8150_probe: after check_device_ids()" );
	

	///info( " rtl8150_probe: before usb_set_configuration()" );
	/// the following is for fixing the bug of 1st-cut, and must be removed in 2nd-cut
	dev->config[0].bConfigurationValue = 1;
	////////////////////////////////////////////////
	if (usb_set_configuration(dev, dev->config[0].bConfigurationValue)) 
	{
		err("usb_set_configuration() failed");
		info( " rtl8150_probe: usb_set_configuration() failed" );
		return NULL;
	}
	///info( " rtl8150_probe: after usb_set_configuration()" );

	if(!(rtl8150 = kmalloc(sizeof(struct rtl8150), GFP_KERNEL))) 
	{
		err("out of memory allocating device structure");
		info( " rtl8150_probe: out of memory allocating device structure" );
		return NULL;
	}
	
	/// added by Owen on 01/11/2000
	rtl8150->get_registers_flag = 0;
	rtl8150->set_registers_flag = 0;

	///info( " rtl8150_probe: before usb_inc_dev_use()" );
	usb_inc_dev_use( dev );
	memset(rtl8150, 0, sizeof(struct rtl8150));
	///info( " rtl8150_probe: after usb_inc_dev_use()" );
	
	///info( " rtl8150_probe: before init_MUTEX()" );
	init_MUTEX( &rtl8150-> ctrl_sem );
	///info( " rtl8150_probe: after init_MUTEX()" );
	
	///info( " rtl8150_probe: before init_waitqueue_head()" );
	init_waitqueue_head( &rtl8150->ctrl_wait );
	///info( " rtl8150_probe: after init_waitqueue_head()" );

	///info( " rtl8150_probe: before init_etherdev()" );
	net = init_etherdev( NULL, 0 );
	if ( !net ) 
	{
		kfree( rtl8150 );
		info( " rtl8150_probe: init_etherdev() failed" );
		return	NULL;
	}
	///info( " rtl8150_probe: after init_etherdev()" );
	
	rtl8150->usb = dev;
	rtl8150->net = net;
	net->priv = rtl8150;
	net->open = rtl8150_open;
	net->stop = rtl8150_close;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,3,48)
	net->watchdog_timeo = RTL8150_TX_TIMEOUT;
	net->tx_timeout = rtl8150_tx_timeout;
#endif
	net->do_ioctl = rtl8150_ioctl;
	net->hard_start_xmit = rtl8150_start_xmit;
	net->set_multicast_list = rtl8150_set_multicast;
	net->get_stats = rtl8150_netdev_stats;
	net->mtu = RTL8150_MTU;     //  1500

	rtl8150->features = usb_dev_id[dev_indx].private;
	
	///info( " rtl8150_probe: before reset_mac()" );
	if ( reset_mac(rtl8150) ) 
	{
		err("can't reset MAC");
		info( " rtl8150_probe: reset_mac() failed" );
		unregister_netdev( rtl8150->net );
		kfree(rtl8150);
		rtl8150 = NULL;
		return NULL;
	}
	///info( " rtl8150_probe: after reset_mac()" );

	///info( " rtl8150_probe: before set_ethernet_addr()" );
	set_ethernet_addr( rtl8150 );
	///info( " rtl8150_probe: after set_ethernet_addr()" );
	
	if( rtl8150->features & FEATURE_HAS_HOME_PNA )
		mii_HPNA_phy_probe( rtl8150 );
	
	info( "%s: %s", net->name, usb_dev_id[dev_indx].name );

	info( "-rtl8150_probe() <=====" );
	return rtl8150;
}


static void rtl8150_disconnect( struct usb_device *dev, void *ptr )
{
	struct rtl8150 *rtl8150 = ptr;

	info( "+rtl8150_disconnect() =====>" );

	if ( !rtl8150 ) {
		info( " rtl8150_disconnect: unregister non-existant device" );
		warn("unregistering non-existant device");
		return;
	}

	rtl8150->flags |= RTL8150_UNPLUG;
	unregister_netdev( rtl8150->net );
	usb_dec_dev_use( dev );
	kfree( rtl8150 );
	rtl8150 = NULL;
	info( "-rtl8150_disconnect() <=====" );
}


static struct usb_driver rtl8150_driver = {
	name:		"RTL8150",
	probe:		rtl8150_probe,
	disconnect:	rtl8150_disconnect,
};


/// this function is called when "insmod rtl8150.o"
int __init rtl8150_init(void)
{
	int	registerResult;
	
	info( "+rtl8150_init() =====>" );
	info( "%s", version );
        registerResult = usb_register( &rtl8150_driver );
	info( "-rtl8150_init() <=====" );

	return registerResult;
}


/// this funciton is called when "rmmod rtl8150"
void __exit rtl8150_exit(void)
{
	info( "+rtl8150_exit() =====>" );
	usb_deregister( &rtl8150_driver );
	info( "-rtl8150_exit() <=====" );
}

module_init( rtl8150_init );

module_exit( rtl8150_exit );

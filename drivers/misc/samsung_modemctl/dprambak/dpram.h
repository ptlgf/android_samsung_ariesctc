/****************************************************************************

**

** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2010 ALL RIGHTS RESERVED

**

** AUTHOR       : Kim, Geun-Young <geunyoung.kim@samsung.com>			@LDK@

**                                                                      @LDK@

****************************************************************************/

#ifndef __DPRAM_H__
#define __DPRAM_H__


/*
 * FEATURE DEFINITIONs
 */
#define _DEBUG 
#define _DEBUG_LOG

/* 
 * DPRAM SETTINGS - 16K S5PC110 Internal dpram
 */
#define DPRAM_VBASE g_idpram_region
#define DPRAM_SIZE									0x4000 			/* 16KB Size */

#define DPRAM_START_ADDRESS 							0x0000
#define DPRAM_MAGIC_CODE_ADDRESS						(DPRAM_START_ADDRESS)
#define DPRAM_ACCESS_ENABLE_ADDRESS					(DPRAM_START_ADDRESS + 0x0002)

#define DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS		(DPRAM_START_ADDRESS + 0x0004)
#define DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS		(DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS)
#define DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS		(DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS + 0x0002)
#define DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS		(DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS + 0x0004)
#define DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE			1020

#define DPRAM_PDA2PHONE_RAW_START_ADDRESS				(DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS + (DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE+4))
#define DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS				(DPRAM_PDA2PHONE_RAW_START_ADDRESS)
#define DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS				(DPRAM_PDA2PHONE_RAW_START_ADDRESS + 2)
#define DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS				(DPRAM_PDA2PHONE_RAW_START_ADDRESS + 4)
#define DPRAM_PDA2PHONE_RAW_BUFFER_SIZE					7160

#define DPRAM_PHONE2PDA_FORMATTED_START_ADDRESS		(DPRAM_PDA2PHONE_RAW_START_ADDRESS + (DPRAM_PDA2PHONE_RAW_BUFFER_SIZE+4))
#define DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS		(DPRAM_PHONE2PDA_FORMATTED_START_ADDRESS)
#define DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS		(DPRAM_PHONE2PDA_FORMATTED_START_ADDRESS + 0x0002)
#define DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS		(DPRAM_PHONE2PDA_FORMATTED_START_ADDRESS + 0x0004)
#define DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE			1020
 
#define DPRAM_PHONE2PDA_RAW_START_ADDRESS				(DPRAM_PHONE2PDA_FORMATTED_START_ADDRESS + (DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE+4))
#define DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS				(DPRAM_PHONE2PDA_RAW_START_ADDRESS)
#define DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS				(DPRAM_PHONE2PDA_RAW_START_ADDRESS + 0x0002)
#define DPRAM_PHONE2PDA_RAW_BUFFER_ADDRESS				(DPRAM_PHONE2PDA_RAW_START_ADDRESS + 0x0004)
#define DPRAM_PHONE2PDA_RAW_BUFFER_SIZE					7160

#define DPRAM_INTERRUPT_PORT_SIZE						2
  
#define TRUE	1
#define FALSE	0


/*
 * INTERRUPT MASKs
 */
#define INT_MASK_VALID					0x0080
#define INT_MASK_COMMAND				0x0040
#define INT_MASK_REQ_ACK_F				0x0020
#define INT_MASK_REQ_ACK_R				0x0010
#define INT_MASK_RES_ACK_F				0x0008
#define INT_MASK_RES_ACK_R				0x0004
#define INT_MASK_SEND_F					0x0002
#define INT_MASK_SEND_R					0x0001

#define MBX_CMD_INIT_START			0x0001
#define MBX_CMD_INIT_END				0x0002
#define MBX_CMD_REQ_ACTIVE			0x0003
#define MBX_CMD_RES_ACTIVE			0x0004
#define MBX_CMD_REQ_TIME_SYNC		0x0005
#define MBX_CMD_PHONE_RESET			0x0007
#define MBX_CMD_PHONE_START 			0x0008
#define MBX_CMD_PHONE_COMMON_BOOT  	0x0108
#define MBX_CMD_ERR_DISPLAY			0x0009
#define MBX_CMD_PHONE_DEEP_SLEEP		0x000A

#define MBX_CMD_DPRAM_DOWN           0x000b // DPRAM_DOWN Interrupt from Modem 0xCB
#define MBX_CMD_PDA_WAKEUP_INT      0x000c // PDA Wake up Interrupt to Modem 0xCC
#define MBX_CMD_PDA_SLEEP_INT       0x000d // PDA Sleep Interrupt to Modem 0xCD
#define MBX_CMD_CP_WAKEUP_START     0x000e // CP Send ack CE to PDA

#define MBX_CMD_CDMA_DEAD			0xAB09

#define INT_COMMAND(x)					(INT_MASK_VALID | INT_MASK_COMMAND | x)
#define INT_NON_COMMAND(x)				(INT_MASK_VALID | x)

#define FORMATTED_INDEX					0
#define RAW_INDEX						1
#define MAX_INDEX						2


/* 
 * IOCTL COMMANDs
 */
#define IOC_MZ_MAGIC					('o')
#define DPRAM_PHONE_POWON				_IO(IOC_MZ_MAGIC, 0xd0)
#define DPRAM_PHONEIMG_LOAD				_IO(IOC_MZ_MAGIC, 0xd1)
#define DPRAM_NVDATA_LOAD				_IO(IOC_MZ_MAGIC, 0xd2)
#define DPRAM_PHONE_BOOTSTART			_IO(IOC_MZ_MAGIC, 0xd3)
#define DPRAM_PHONE_BOOTTYPE        _IOW(IOC_MZ_MAGIC,0xd5,unsigned int)
#define IOCTL_ONEDRAM_WAKEUP        _IOW(IOC_MZ_MAGIC,0xd7, unsigned int)
//#define DPRAM_PHONE_RESET           _IO(IOC_MZ_MAGIC,0xd8)
#define DPRAM_PHONE_RAM_DUMP        _IO(IOC_MZ_MAGIC,0xd9)
#define IOCTL_SET_FATAL_OPERATION   _IOW(IOC_MZ_MAGIC,0xda, unsigned int)

#define IOC_SEC_MAGIC            (0xf0)
#define DPRAM_PHONE_ON           _IO(IOC_SEC_MAGIC, 0xc0)
#define DPRAM_PHONE_OFF          _IO(IOC_SEC_MAGIC, 0xc1)


#define DPRAM_PHONE_GETSTATUS    _IOR(IOC_SEC_MAGIC, 0xc2, unsigned int)
#define DPRAM_PHONE_RESET        _IO(IOC_SEC_MAGIC, 0xc5)
#define DPRAM_PHONE_RAMDUMP_ON    _IO(IOC_SEC_MAGIC, 0xc6)
#define DPRAM_PHONE_RAMDUMP_OFF   _IO(IOC_SEC_MAGIC, 0xc7)
#define DPRAM_EXTRA_MEM_RW        _IOWR(IOC_SEC_MAGIC, 0xc8, unsigned long)

#define DPRAM_PHONE_UNSET_UPLOAD          _IO(IOC_SEC_MAGIC, 0xca)
#define DPRAM_PHONE_SET_AUTOTEST          _IO(IOC_SEC_MAGIC, 0xcb)
#define DPRAM_PHONE_SET_DEBUGLEVEL          _IO(IOC_SEC_MAGIC, 0xcc)
#define DPRAM_PHONE_GET_DEBUGLEVEL          _IO(IOC_SEC_MAGIC, 0xcd)


/*
 * COMMON BOOT DEFINITIONs
 */
#define CP_CHIPSET_QUALCOMM     0x100
#define CP_CHIPSET_INFINEON     0x200
#define CP_CHIPSET_BROADCOM     0x300
#define AP_PLATFORM_ANDROID     0x100
#define AP_PLATFORM_LINUX       0x300
#define CP_ONLINE_BOOT      0x0000
#define CP_AIRPLANE_BOOT    0x1000

/*
 * BOOT MAGIC KEY
 */
#define DPRAM_BOOT_NORMAL 'TBMN'


/*
 * S5PC110 DPRAM REGISTER MAP
 */

#define IDPRAM_PHYSICAL_ADDR 		0xED000000
#define IDPRAM_START_ADDR 		((volatile void __iomem *)g_idpram_region)
#define IDPRAM_SIZE 				0x4000

#define IDPRAM_AP2MSM_INT_OFFSET 0x3FFC
#define IDPRAM_MSM2AP_INT_OFFSET 0x3FFE

#define IDPRAM_SFR_PHYSICAL_ADDR		0xED008000
#define IDPRAM_SFR_START_ADDR 		((volatile void __iomem *)g_dpram_sfr_base)
#define IDPRAM_SFR_INT2AP 			(IDPRAM_SFR_START_ADDR)
#define IDPRAM_SFR_INT2MSM			(IDPRAM_SFR_START_ADDR + 0x4)
#define IDPRAM_SFR_MIFCON				(IDPRAM_SFR_START_ADDR + 0x8)
#define IDPRAM_SFR_MIFPCON			(IDPRAM_SFR_START_ADDR + 0xC)
#define IDPRAM_SFR_MSMINTCLR			(IDPRAM_SFR_START_ADDR + 0x10)
#define IDPRAM_SFR_DMA_TX_ADR		(IDPRAM_SFR_START_ADDR + 0x14)
#define IDPRAM_SFR_DMA_RX_ADR		(IDPRAM_SFR_START_ADDR + 0x18)
#define IDPRAM_SFR_SIZE 				0x1C

// It is recommended that S5PC110 write data with half-word access on the interrupt port because
// S5PC100 overwrites tha data in INT2AP if there are INT2AP and INT2MSM sharing the same word
#define IDPRAM_INT2MSM_MASK 0xff

#define IDPRAM_MIFCON_INT2APEN		(1<<2)
#define IDPRAM_MIFCON_INT2MSMEN		(1<<3)
#define IDPRAM_MIFCON_DMATXREQEN_0	(1<<16)
#define IDPRAM_MIFCON_DMATXREQEN_1	(1<<17)
#define IDPRAM_MIFCON_DMARXREQEN_0	(1<<18)
#define IDPRAM_MIFCON_DMARXREQEN_1	(1<<19)
#define IDPRAM_MIFCON_FIXBIT			(1<<20)

#define IDPRAM_MIFPCON_ADM_MODE 		(1<<6) // mux / demux mode

#define IDPRAM_DMA_ADR_MASK 0x3FFF
#define IDPRAM_DMA_TX_ADR_0		// shift 0
#define IDPRAM_DMA_TX_ADR_1		// shift 16
#define IDPRAM_DMA_RX_ADR_0		// shift 0
#define IDPRAM_DMA_RX_ADR_1		// shift 16

#define IDPRAM_INT_CLEAR() g_dpram_sfr_base->msmintclr = 0xff

typedef struct {
	unsigned int int2ap;
	unsigned int int2msm;
	unsigned int mifcon;
	unsigned int mifpcon;
	unsigned int msmintclr;
	unsigned int dma_tx_adr;
	unsigned int dma_rx_adr;
} IDPRAM_SFR;


/*
 * DEVICE DEFINITIONs
 */
#define DRIVER_NAME 		"dpram0"
#define DRIVER_PROC_ENTRY	"driver/dpram0"
#define DRIVER_MAJOR_NUM	232

/*
 * MULTI PDP DEFINITIONs
 */
#define APP_DEVNAME				"multipdp0"	/* Device node name for application interface */
#define NUM_PDP_CONTEXT			3		/* number of PDP context */
/* Device types */
#define DEV_TYPE_NET			0 /* network device for IP data */
#define DEV_TYPE_SERIAL			1 /* serial device for CSD */
#define CSD_MAJOR_NUM			231 /* Device major & minor number */
#define CSD_MINOR_NUM			0
#define MAX_PDP_CONTEXT			10	/* Maximum number of PDP context */
#define MAX_PDP_DATA_LEN		1500 /* Maximum PDP data length */
/* Device flags */
#define DEV_FLAG_STICKY			0x1 /* Sticky */
#define MAX_PDP_PACKET_LEN		(MAX_PDP_DATA_LEN + 4 + 2) /* Maximum PDP packet length including header and start/stop bytes */

typedef struct pdp_arg {
	unsigned char	id;
	char		ifname[16];
} __attribute__ ((packed)) pdp_arg_t; /* Multiple PDP */

struct pdp_hdr {
	u16	len;		/* Data length */
	u8	id;			/* Channel ID */
	u8	control;	/* Control field */
} __attribute__ ((packed)); /* PDP data packet header format */

struct pdp_info {
	u8		id;	/* PDP context ID */
	unsigned		type;	/* Device type */
	unsigned		flags;	/* Device flags */
	u8		*tx_buf;	/* Tx packet buffer */
	union {
		struct {
			struct tty_driver	tty_driver[NUM_PDP_CONTEXT];	// CSD, CDMA, TRFB, CIQ
			int			refcount;
			struct tty_struct	*tty_table[1];
			struct ktermios		*termios[1];
			struct ktermios		*termios_locked[1];
			char			tty_name[16];
			struct tty_struct	*tty;
			struct semaphore	write_lock;
		} vs_u;		/* Virtual serial interface */
	} dev_u;	/* App device interface */
#define vn_dev		dev_u.vnet_u
#define vs_dev		dev_u.vs_u
};/* PDP information type */

/*
 * DEBUG FEATUREs
 */
#ifdef _DEBUG
#define _ENABLE_ERROR_DEVICE
#endif

//#ifdef _DEBUG
//#define dprintk(s, args...) printk(KERN_ERR "[IDPRAM] %s:%d - " s, __func__, __LINE__,  ##args)
//#else
//#define dprintk(s, args...)
//#endif	/* _DEBUG */

#define DL_IPC     0x01<<10
#define DL_WARN	0x01<<15
#define DL_NOTICE	0x01<<20
#define DL_INFO  	0x01<<25
#define DL_DEBUG 	0x01<<30

#ifdef _DEBUG_LOG // debug printf
#define LOGE(s, args...) printk(KERN_ERR "IDPRAM/Err/%s| " s, __func__, ##args) // Error log
#define LOGA(s, args...) printk(KERN_ERR "IDPRAM|%s| " s, __func__, ##args)	// Alway printf
#define LOG(s, args...) printk("IDPRAM|%s| " s,__func__, ##args)
#define LOGL(mask, s, args...) printk("IDPRAM|%s| " s,__func__, ##args)//do{if(mask & __log_level__) printk("IDPRAM|%s| " s,__func__, ##args);}while(0)
#else
#define LOGE(s, args...) printk("IDPRAM/Err/%s| " s, __func__, ##args) // Error log
#define LOGA(s, args...) printk("IDPRAM|%s| " s, __func__, ##args)	// Alway printf
#define LOG(...)  //printk("IDPRAM|%s| " s,__func__, ##args)
#define LOGL(...) 
#endif


#ifdef _ENABLE_ERROR_DEVICE
#define DPRAM_ERR_MSG_LEN			65
#define DPRAM_ERR_DEVICE			"dpramerr"
#endif	/* _ENABLE_ERROR_DEVICE */

/*
 * STRUCTURE DEFINITION
 */
struct _param_em {
	unsigned int offset;
	unsigned char *addr;
	unsigned int size;
	int rw;
};

typedef struct dpram_serial {
	struct tty_struct *tty; 		/* pointer to the tty for this device */
	int open_count; 				/* number of times this port has been opened */
	struct semaphore sem;			/* locks this structure */
} dpram_serial_t;

typedef struct dpram_device {
	unsigned long in_head_addr;
	unsigned long in_tail_addr;
	unsigned long in_buff_addr;
	unsigned long in_buff_size;

	unsigned long out_head_addr;
	unsigned long out_tail_addr;
	unsigned long out_buff_addr;
	unsigned long out_buff_size;

	unsigned int in_head_saved;
	unsigned int in_tail_saved;
	unsigned int out_head_saved;
	unsigned int out_tail_saved;

	u_int16_t mask_req_ack;
	u_int16_t mask_res_ack;
	u_int16_t mask_send;

	dpram_serial_t serial;
} dpram_device_t;

typedef struct dpram_tasklet_data {
	dpram_device_t *device;
	u_int16_t non_cmd;
} dpram_tasklet_data_t;

struct _mem_param {
	unsigned short addr;
	unsigned long data;
	int dir;
};

/* TODO: add more definitions */

#define GPIO_LEVEL_LOW				0
#define GPIO_LEVEL_HIGH				1

#define IDPRAM_WPEND_LOCK		'lock'
#define IDPRAM_WPEND_UNLOCK		'unlk'
/*
 * MACRO FUNCTIONs
 */


#endif	/* __DPRAM_H__ */


/****************************************************************************
**
** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2010 ALL RIGHTS RESERVED
**
**                IDPRAM Device Driver
**
****************************************************************************/



#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/sched.h>//ctc
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif	/* CONFIG_PROC_FS */

#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include "dpram.h"
#include <../../../usb/gadget/ds_manager.h>
/***************************************************************************/
/*                              GPIO SETTING                               */
/***************************************************************************/



#define IRQ_INT_IDPRAM_AP_N		IRQ_MSM
#define IRQ_QSC_ACTIVE			IRQ_EINT8
#define IRQ_QSC_INT				IRQ_EINT14


/*
 * GLOBALS
 */
volatile void __iomem *g_idpram_region=NULL;
volatile IDPRAM_SFR __iomem *g_dpram_sfr_base = NULL;
struct wake_lock g_wakelock_dpram = {.name=NULL};
static volatile u16 *g_dpram_mbx_BA;		//send mail
static volatile u16 *g_dpram_mbx_AB;		//received mail
static atomic_t g_dpram_lock_write;
static atomic_t g_dpram_lock_read;
static int g_phone_sync = 0;
static int g_dump_on = 0;
static int g_dpram_wpend=IDPRAM_WPEND_UNLOCK;
struct completion g_complet_dpramdown;
static int g_cp_reset_cnt=0;
static int phone_power_off_sequence = 0;


//static unsigned int __log_level__=(DL_IPC|DL_WARN|DL_NOTICE|DL_INFO|DL_DEBUG);


static struct tty_driver *dpram_tty_driver;
static dpram_tasklet_data_t dpram_tasklet_data[MAX_INDEX];
static dpram_device_t dpram_table[MAX_INDEX] = {
	{
		.in_head_addr = DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE,
		.in_head_saved = 0,
		.in_tail_saved = 0,

		.out_head_addr = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE,
		.out_head_saved = 0,
		.out_tail_saved = 0,

		.mask_req_ack = INT_MASK_REQ_ACK_F,
		.mask_res_ack = INT_MASK_RES_ACK_F,
		.mask_send = INT_MASK_SEND_F,
	},
	{
		.in_head_addr = DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_RAW_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_RAW_BUFFER_SIZE,
		.in_head_saved = 0,
		.in_tail_saved = 0,

		.out_head_addr = DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_RAW_BUFFER_SIZE,
		.out_head_saved = 0,
		.out_tail_saved = 0,

		.mask_req_ack = INT_MASK_REQ_ACK_R,
		.mask_res_ack = INT_MASK_RES_ACK_R,
		.mask_send = INT_MASK_SEND_R,
	},
};

static struct tty_struct *dpram_tty[MAX_INDEX];
static struct ktermios *dpram_termios[MAX_INDEX];
static struct ktermios *dpram_termios_locked[MAX_INDEX];
static struct pdp_info *pdp_table[MAX_PDP_CONTEXT];

static void res_ack_tasklet_handler(unsigned long data);
static void fmt_rcv_tasklet_handler(unsigned long data);
static void raw_rcv_tasklet_handler(unsigned long data);

static DECLARE_TASKLET(fmt_send_tasklet, fmt_rcv_tasklet_handler, 0);
static DECLARE_TASKLET(raw_send_tasklet, raw_rcv_tasklet_handler, 0);
static DECLARE_TASKLET(fmt_res_ack_tasklet, res_ack_tasklet_handler,(unsigned long)&dpram_table[FORMATTED_INDEX]);
static DECLARE_TASKLET(raw_res_ack_tasklet, res_ack_tasklet_handler,(unsigned long)&dpram_table[RAW_INDEX]);
static DEFINE_MUTEX(pdp_lock);

/*
 * FUNCTION PROTOTYPEs
 */
static void dpram_drop_data(dpram_device_t *device);
static int dpram_phone_getstatus(void);
static void dpram_send_mbx_BA(u16 irq_mask);
static void dpram_send_mbx_BA_cmd(u16 irq_mask);
static void dpram_power_down(void);
static void dpram_powerup_start(void);

static inline struct pdp_info * pdp_get_dev(u8 id);
static inline void check_pdp_table(char*, int);
static void cmd_error_display_handler(void);


#ifdef _ENABLE_ERROR_DEVICE
static unsigned int dpram_err_len;
static char dpram_err_buf[DPRAM_ERR_MSG_LEN];
static unsigned int dpram_err_cause=0;

struct class *dpram_class;

static DECLARE_WAIT_QUEUE_HEAD(dpram_err_wait_q);
static struct fasync_struct *dpram_err_async_q;
//extern void usb_switch_mode(int);
#endif	/* _ENABLE_ERROR_DEVICE */

/* tty related functions. */
static inline void byte_align(unsigned long dest, unsigned long src)
{
	u16 *p_src;
	volatile u16 *p_dest;

	if (!(dest % 2) && !(src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0xFF00) | (*p_src & 0x00FF);
	}

	else if ((dest % 2) && (src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0x00FF) | (*p_src & 0xFF00);
	}

	else if (!(dest % 2) && (src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0xFF00) | ((*p_src >> 8) & 0x00FF);
	}

	else if ((dest % 2) && !(src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0x00FF) | ((*p_src << 8) & 0xFF00);
	}

	else {
		LOGE("oops.~\n");
	}
}

static inline void _memcpy(void *p_dest, const void *p_src, int size)
{
	unsigned long dest = (unsigned long)p_dest;
	unsigned long src = (unsigned long)p_src;

	if (size <= 0) {
		return;
	}

	if (dest & 1) {
		byte_align(dest, src);
		dest++, src++;
		size--;
	}

	if (size & 1) {
		byte_align(dest + size - 1, src + size - 1);
		size--;
	}

	if (src & 1) {
		unsigned char *s = (unsigned char *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) {
			*d++ = s[0] | (s[1] << 8);
			s += 2;
		}
	}

	else {
		u16 *s = (u16 *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) { *d++ = *s++; }
	}
}

static inline int _memcmp(u8 *dest, u8 *src, int size)
{
	int i = 0;
	while (i++ < size) {
		if (*(dest + i) != *(src + i)) {
			return 1;
		}
	}
	return 0;
}

#define WRITE_TO_DPRAM(dest, src, size) \
	_memcpy((void *)(DPRAM_VBASE + dest), src, size)

#define READ_FROM_DPRAM(dest, src, size) \
	_memcpy(dest, (void *)(DPRAM_VBASE + src), size)


static inline int WRITE_TO_DPRAM_VERIFY(u32 dest, void *src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)(DPRAM_VBASE + dest), (void *)src, size);

		if (!_memcmp((u8 *)(DPRAM_VBASE + dest), (u8 *)src, size))
			return 0;
	}

	return -1;
}

static inline int READ_FROM_DPRAM_VERIFY(void *dest, u32 src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)dest, (void *)(DPRAM_VBASE + src), size);

		if (!_memcmp((u8 *)dest, (u8 *)(DPRAM_VBASE + src), size))
			return 0;
	}

	return -1;
}

static int dpram_get_lock_write(void)
{
	return atomic_read(&g_dpram_lock_write);
}

static int dpram_lock_write(const char* func)
{	
	int lock_value;

	lock_value = (g_dpram_wpend == IDPRAM_WPEND_LOCK)?-3:atomic_inc_return(&g_dpram_lock_write);
	if(lock_value !=1)
		LOGE("(%s, lock) lock_value: %d\n", func, lock_value);
/*
	if(!(lock_value = atomic_inc_return(&g_dpram_lock)))
		printk(KERN_ERR "[IDPRAM] (%s, lock) fail to locking IDPRAM access. %d\n", func, lock_value);
	
	if(lock_value != 1)
		printk(KERN_ERR "[IDPRAM] (%s, lock) lock_value: %d\n", func, lock_value);
*/
	return lock_value;	
}

static void dpram_unlock_write(const char* func)
{
	int lock_value;

	lock_value = atomic_dec_return(&g_dpram_lock_write);

	if(lock_value !=0)
		LOGE("(%s, lock) lock_value: %d\n", func, lock_value);

/*	
	if((lock_value = atomic_dec_return(&g_dpram_lock)) < 0)
		printk(KERN_ERR "[IDPRAM] (%s, release) fail to unlocking IDPRAM access. %d\n", func, lock_value);
	
	if(lock_value != 0)
		printk(KERN_ERR "[IDPRAM] (%s, release) lock_value: %d\n", func, lock_value);
*/
}

// TODO:
// because it is dpram device, I think read_lock don't need but I will use for holding wake_lock
static int dpram_get_lock_read(void)
{
	return atomic_read(&g_dpram_lock_read);
}

static int dpram_lock_read(const char* func)
{	
	int lock_value;

	lock_value = atomic_inc_return(&g_dpram_lock_read);
	if(lock_value !=1)
		LOGE("(%s, lock) lock_value: %d\n", func, lock_value);
	wake_lock_timeout(&g_wakelock_dpram, HZ*6);
	return 0;	
}

static void dpram_unlock_read(const char* func)
{
	int lock_value;

	lock_value = atomic_dec_return(&g_dpram_lock_read);

	if(lock_value !=0)
		LOGE("(%s, lock) lock_value: %d\n", func, lock_value);

	wake_unlock(&g_wakelock_dpram);
}


static int dpram_write(dpram_device_t *device,
		const unsigned char *buf, int len)
{
	int retval = 0;
	int size = 0;
	u16 head, tail;
	u16 irq_mask = 0;


	if(g_phone_sync!=1)
	{
		LOGE("Phone dosen't boot!! phone sync (%d)\n", g_phone_sync);
		return -EFAULT;
	}

	if(dpram_lock_write(__func__) < 0) {
		return -EAGAIN;
	}

	READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

//printk(KERN_ERR "%s, head: %d, tail: %d\n", __func__, head, tail);

	// +++++++++ head ---------- tail ++++++++++ //
	if (head < tail) {
		size = tail - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// tail +++++++++++++++ head --------------- //
	else if (tail == 0) {
		size = device->out_buff_size - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// ------ tail +++++++++++ head ------------ //
	else {
		size = device->out_buff_size - head;
		size = (len > size) ? size : len;
		
		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;

		if (len > retval) {
			size = (len - retval > tail - 1) ? tail - 1 : len - retval;
			
			WRITE_TO_DPRAM(device->out_buff_addr, buf + retval, size);
			retval += size;
		}
	}

	/* @LDK@ calculate new head */
	head = (u16)((head + retval) % device->out_buff_size);
	WRITE_TO_DPRAM_VERIFY(device->out_head_addr, &head, sizeof(head));
	

	device->out_head_saved = head;
	device->out_tail_saved = tail;

	/* @LDK@ send interrupt to the phone, if.. */
	irq_mask = INT_MASK_VALID;

	if (retval > 0)
		irq_mask |= device->mask_send;

	if (len > retval)
		irq_mask |= device->mask_req_ack;

	dpram_unlock_write(__func__);
	dpram_send_mbx_BA(irq_mask);
//	up(&write_mutex);
	LOGL(DL_DEBUG, "WRITE: return: %d\n", retval);
	return retval;
	
}

static inline int dpram_tty_insert_data(dpram_device_t *device, const u8 *psrc, u16 size)
{
#define CLUSTER_SEGMENT	1500

	u16 copied_size = 0;
	int retval = 0;
	
#ifdef PRINT_READ
	int i;
		printk(KERN_ERR "READ: %d\n", size);
		for(i=0; i<size; i++)	
			printk(KERN_ERR "%02x ", *(psrc+ + i));
		printk(KERN_ERR "\n");
#endif
#ifdef PRINT_READ_SHORT
	printk(KERN_ERR "READ: size:  %d\n", size);
#endif

	if (size > CLUSTER_SEGMENT && (device->serial.tty->index == 1)) {
		while (size) {
			copied_size = (size > CLUSTER_SEGMENT) ? CLUSTER_SEGMENT : size;
			tty_insert_flip_string(device->serial.tty, psrc + retval, copied_size);

			size -= copied_size;
			retval += copied_size;
		}

		return retval;
	}

	return tty_insert_flip_string(device->serial.tty, psrc, size);
}

static int dpram_read_fmt(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int retval_add = 0;
	int size = 0;
	u16 head, tail;

	dpram_lock_read(__func__);

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	LOGL(DL_DEBUG, "FMT head: %d, tail: %d\n", head, tail);

	if (head != tail) {
		u16 up_tail = 0;

		// ------- tail ########## head -------- //
		if (head > tail) {
			size = head - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail)), size);
			if(size!= retval)
				LOGE("size: %d, retval: %d\n", size, retval);

			LOGL(DL_DEBUG, "READ -return: %d\n", retval );
		}

		// ####### head ------------ tail ###### //
		else {
			int tmp_size = 0;

			// Total Size.
			size = device->in_buff_size - tail + head;

			// 1. tail -> buffer end.
			tmp_size = device->in_buff_size - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail)), tmp_size);
			if(tmp_size!= retval)
				LOGE("size: %d, retval: %d\n", size, retval);

			LOGL(DL_DEBUG, "READ -return: %d\n", retval );

			// 2. buffer start -> head.
			if (size > tmp_size) {
				retval_add = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + device->in_buff_addr), size - tmp_size);
				retval += retval_add;
		
				if((size - tmp_size)!= retval_add)
					LOGE("size - tmp_size: %d, retval_add: %d\n", size - tmp_size, retval_add);

				LOGL(DL_DEBUG,"READ -return_add: %d\n", retval_add);
			}
		}

		/* new tail */
		up_tail = (u16)((tail + retval) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}
		

	device->in_head_saved = head;
	device->in_tail_saved = tail;

	dpram_unlock_read(__func__);
	if (non_cmd & device->mask_req_ack) 	
		dpram_send_mbx_BA(INT_NON_COMMAND(device->mask_res_ack));

	return retval;
	
}

static int dpram_read_raw(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int size = 0;
	u16 head, tail;
	u16 up_tail = 0;
	int ret;
	size_t len;
	struct pdp_info *dev = NULL;
	struct pdp_hdr hdr;
	u16 read_offset;
	u8 len_high, len_low, id, control;
	u16 pre_data_size; //pre_hdr_size,
	u8 ch;

	dpram_lock_read(__func__);

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	LOGL(DL_DEBUG,"RAW head: %d, tail: %d\n", head, tail );

	if(head != tail) {
	
		up_tail = 0;

		if (head > tail) {
			size = head - tail;									/* ----- (tail) 7f 00 00 7e (head) ----- */ 
		}
		else
			size = device->in_buff_size - tail + head;			/* 00 7e (head) ----------- (tail) 7f 00 */ 

		read_offset = 0;
		LOGL(DL_DEBUG, "head: %d, tail: %d, size: %d\n", head, tail, size);

		while(size){			
			READ_FROM_DPRAM(&ch, device->in_buff_addr +((u16)(tail + read_offset) % device->in_buff_size), sizeof(ch));

			if(ch == 0x7f) {
				read_offset ++;
			}
			else {
                LOGE("First byte: 0x%02x, drop bytes: %d, buff addr: 0x%08x, read addr: 0x%08x\n",
					ch, size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));

				dpram_drop_data(device);
				dpram_unlock_read(__func__);
				return -1;
			}

			len_high = len_low = id = control = 0;
			READ_FROM_DPRAM(&len_low, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(len_high));
			read_offset ++;
			READ_FROM_DPRAM(&len_high, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(len_low));
			read_offset ++;
			READ_FROM_DPRAM(&id, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(id));
			read_offset ++;
			READ_FROM_DPRAM(&control, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(control));
			read_offset ++;

			hdr.len = len_high <<8 | len_low;
			hdr.id = id;
			hdr.control = control;
	
			len = hdr.len - sizeof(struct pdp_hdr);	
			if(len <= 0) {
				LOGE("RAW uups..\n read_offset: %d, len: %d hdr.id: %d\n", read_offset, len, hdr.id);
				dpram_drop_data(device);
				dpram_unlock_read(__func__);
				return -1;


			}
			dev = pdp_get_dev(hdr.id);
			LOGL(DL_DEBUG, "RAW read_offset: %d, len: %d hdr.id: %d\n", read_offset, len, hdr.id );

			if(!dev) {
				LOGE(" RAW READ Failed.. NULL dev detected \n");
				check_pdp_table(__func__, __LINE__);
				dpram_drop_data(device);
				dpram_unlock_read(__func__);
				return -1;
			}

				
			if (dev->vs_dev.tty != NULL && dev->vs_dev.refcount) {
			
				if((u16)(tail + read_offset) % device->in_buff_size + len < device->in_buff_size) {
					ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
                 dev->vs_dev.tty->low_latency = 0;
					tty_flip_buffer_push(dev->vs_dev.tty);
				}else {
					pre_data_size = device->in_buff_size - (tail + read_offset); 
					ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail + read_offset)), pre_data_size);
					ret += tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr)),len - pre_data_size);
                 dev->vs_dev.tty->low_latency = 0;
					tty_flip_buffer_push(dev->vs_dev.tty);
					LOGL(DL_DEBUG, "RAW pre_data_size: %d, len-pre_data_size: %d, ret: %d\n", pre_data_size, len- pre_data_size, ret);
				}
			}
			else {
				LOGE("tty channel(id:%d) is not opened.\n", dev->id);
				ret = len;
			}
			
			if(!ret) {
                LOGE("(tty_insert_flip_string) drop bytes: %d, buff addr: 0x%08x\n, read addr: 0x%08x\n",
					size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));
				dpram_drop_data(device);
				dpram_unlock_read(__func__);
				return -1;
			}
			
			read_offset += ret;
			LOGL(DL_DEBUG,"read_offset: %d ret= %d\n", read_offset, ret );

			READ_FROM_DPRAM(&ch, (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)), sizeof(ch));
			if(ch == 0x7e) 				
				read_offset ++;
			else {
                LOGE("Last byte: 0x%02x, drop bytes: %d, buff addr: 0x%08x, read addr: 0x%08x\n",
					ch, size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)) );
				dpram_drop_data(device);
				dpram_unlock_read(__func__);
				return -1;
			}

			size -= (ret + sizeof(struct pdp_hdr) + 2);
			retval += (ret + sizeof(struct pdp_hdr) + 2);

			if(size < 0) {
				LOGE("something wrong....\n");
				break;
			}

		}
		up_tail = (u16)((tail + read_offset) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}

	device->in_head_saved = head;
	device->in_tail_saved = tail;

	dpram_unlock_read(__func__);
	if (non_cmd & device->mask_req_ack) 	
		dpram_send_mbx_BA(INT_NON_COMMAND(device->mask_res_ack));

	return retval;
	
}
#ifdef _ENABLE_ERROR_DEVICE
void request_phone_reset(void)
{
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;

	memset((void *)buf, 0, sizeof (buf));

	LOG("CDMA reset cnt =%d\n",g_cp_reset_cnt );
	if(g_cp_reset_cnt>5)
	{
		buf[0] = '1';
		buf[1] = ' ';
		memcpy(buf+2, "$CDMA-DEAD", sizeof("$CDMA-DEAD"));
	}
	else
	{
		buf[0] = '8';
		buf[1] = ' ';
		memcpy(buf+2, "$PHONE-OFF", sizeof("$PHONE-OFF"));
	}
	LOGE("[PHONE ERROR] ->> %s\n", buf);
	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	dpram_err_len = 64;
	local_irq_restore(flags);
    
	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);
}
#endif


static void dpram_send_mbx_BA(u16 irq_mask)
{
	if(g_dump_on) return;

	*g_dpram_mbx_BA = irq_mask;
	LOGA("Send MBX: %x \n", irq_mask);
}

/*
 * dpram_send_mbx_BA_cmd()
 * - for prevent CP interrupt command miss issue,
 *  below function check the CP dpram interrupt level before send interrupt
 */
#define DPRAM_CMD_SEND_RETRY_CNT 0x5
static void dpram_send_mbx_BA_cmd(u16 irq_mask)
{
	int retry_cnt = DPRAM_CMD_SEND_RETRY_CNT;
	if(g_dump_on) return;

	while(gpio_get_value(GPIO_DPRAM_INT_CP_N)==0 && retry_cnt--)
	{
		msleep(1);
		LOGE("send cmd intr, retry cnt=%d\n", (DPRAM_CMD_SEND_RETRY_CNT-retry_cnt));
	}
	*g_dpram_mbx_BA = irq_mask;
	LOGA("Send MBX cmd: %x \n", irq_mask);
}


static void dpram_clear(void)
{
	long i = 0;
	unsigned long flags;
	
	u16 value = 0;

	/* @LDK@ clear DPRAM except interrupt area */
	local_irq_save(flags);

	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2)
	{
		*((u16 *)(DPRAM_VBASE + i)) = 0;
	}

	local_irq_restore(flags);

	value = *g_dpram_mbx_AB;
}

static int dpram_init_and_report(void)
{
	const u16 magic_code = 0x00aa;
	const u16 init_start = INT_COMMAND(MBX_CMD_INIT_START);
	const u16 init_end = INT_COMMAND(MBX_CMD_INIT_END|AP_PLATFORM_ANDROID);
 	u16 ac_code = 0;

	if(CP_CHIPSET_QUALCOMM != (*g_dpram_mbx_AB&0x300))
	{
		LOGE("Not suported Base Band chip\n");
		return -1;
	}
	dpram_send_mbx_BA(init_start);
 
	if(dpram_lock_write(__func__) < 0)
		return -EAGAIN;

	/* @LDK@ write DPRAM disable code */
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));

	/* @LDK@ dpram clear */
	dpram_clear();

	/* @LDK@ write magic code */
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));

	/* @LDK@ write DPRAM enable code */
	ac_code = 0x0001;
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));

	/* @LDK@ send init end code to phone */
	dpram_unlock_write(__func__);
	dpram_send_mbx_BA(init_end);
	LOGL(DL_INFO, "Send 0x%x to MailboxBA  (CDMA init finish).\n", init_end);

	g_phone_sync = 1;
	g_cp_reset_cnt=0;

	return 0;
}

static inline int dpram_get_read_available(dpram_device_t *device)
{
    u16 head = 0, tail = 0, size = 0;
	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));
    if ( tail >= device->in_buff_size || head >= device->in_buff_size )
    {
        return 0;
    }

    size = ( head >= tail )? (head - tail) : (device->in_buff_size - tail + head);
    return size;
}

static void dpram_drop_data(dpram_device_t *device)
{
	u16 head, tail;
	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));
    if( head >= device->in_buff_size || tail >= device->in_buff_size )
    {
        head = tail = 0 ;
        WRITE_TO_DPRAM_VERIFY(device->in_head_addr, &head, sizeof(head));
    }
    WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &head, sizeof(head));
	LOGE("DROP head: %d, tail: %d\n",head, tail);
}

static void dpram_phone_power_on(void)
{
	LOGL(DL_INFO,"  - GPIO_QSC_PHONE_ON : %s, GPIO_QSC_RESET_N_RST : %s\n", 
            gpio_get_value(GPIO_QSC_PHONE_ON)?"HIGH":"LOW", gpio_get_value(GPIO_QSC_RESET_N)?"HIGH":"LOW");

	gpio_set_value(GPIO_QSC_PHONE_ON,GPIO_LEVEL_HIGH);
	mdelay(50);
    gpio_set_value(GPIO_QSC_RESET_N,GPIO_LEVEL_LOW);
	mdelay(100);
		LOGL(DL_INFO,"  - GPIO_QSC_PHONE_ON : %s, GPIO_QSC_RESET_N_RST : %s\n", 
            gpio_get_value(GPIO_QSC_PHONE_ON)?"HIGH":"LOW", gpio_get_value(GPIO_QSC_RESET_N)?"HIGH":"LOW");

    gpio_set_value(GPIO_QSC_RESET_N,GPIO_LEVEL_HIGH);
    mdelay(500);
    gpio_set_value(GPIO_QSC_PHONE_ON,GPIO_LEVEL_LOW);

	LOGL(DL_INFO,"  - GPIO_QSC_PHONE_ON : %s, GPIO_QSC_RESET_N_RST : %s\n", 
            gpio_get_value(GPIO_QSC_PHONE_ON)?"HIGH":"LOW", gpio_get_value(GPIO_QSC_RESET_N)?"HIGH":"LOW");


}

static void dpram_phone_power_off(void)
{
	phone_power_off_sequence = 1;
	LOGA("Set Power off sequence flags\n");
/*
	gpio_direction_output(GPIO_QSC_PHONE_ON, GPIO_LEVEL_LOW);
	gpio_direction_output(GPIO_QSC_RESET_N, GPIO_LEVEL_LOW);
	LOG("Phone power Off\n");
*/
}

static int dpram_phone_getstatus(void)
{
	return gpio_get_value(GPIO_QSC_ACTIVE);
}

static void dpram_phone_reset(void)
{
	LOGE("Phone Reset!\n");
	gpio_set_value(GPIO_QSC_RESET_N, GPIO_LEVEL_LOW);
	//mdelay(100);
	msleep(100);
	gpio_set_value(GPIO_QSC_RESET_N, GPIO_LEVEL_HIGH);
	g_cp_reset_cnt++;
}

static int dpram_extra_mem_rw(struct _param_em *param)
{

	if(param->offset + param->size > 0xFFF800) {
		LOGE("wrong rage of external memory access\n");
		return -1;
	}

	if (param->rw) {	//write
		if(dpram_lock_write(__func__) < 0)
			return -EAGAIN;
		WRITE_TO_DPRAM(param->offset, param->addr, param->size);
		dpram_unlock_write(__func__);
	}
	else {				//read
		dpram_lock_read(__func__);
		READ_FROM_DPRAM(param->addr, param->offset, param->size);
		dpram_unlock_read(__func__);
	}
	return 0;
}

static int dpram_qsc_timeout_handler(void)
{
	const u16 rdump_flag1 = 0xdead;
	const u16 rdump_flag2 = 0xdead;
	const u16 temp1, temp2;
	
	LOGL(DL_INFO,"Ramdump ON.\n");

	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS,    &rdump_flag1, sizeof(rdump_flag1));
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &rdump_flag2, sizeof(rdump_flag2));

	READ_FROM_DPRAM((void *)&temp1, DPRAM_MAGIC_CODE_ADDRESS, sizeof(temp1));
	READ_FROM_DPRAM((void *)&temp2, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(temp2));
	LOGL(DL_INFO,"flag1: %x flag2: %x\n", temp1, temp2);

	g_dump_on = 1;
    // If it is configured to dump both AP and CP, reset both AP and CP here.
	LOGE("Configure to restart AP and collect dump on restart...\n");

	return 0;

}


static int dpram_phone_ramdump_on(void)
{
	const u16 rdump_flag1 = 0xdead;
	const u16 rdump_flag2 = 0xdead;
	const u16 temp1, temp2;
	
	LOGL(DL_INFO,"Ramdump ON.\n");
	if(dpram_lock_write(__func__) < 0)
		return -EAGAIN;

	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS,    &rdump_flag1, sizeof(rdump_flag1));
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &rdump_flag2, sizeof(rdump_flag2));

	READ_FROM_DPRAM((void *)&temp1, DPRAM_MAGIC_CODE_ADDRESS, sizeof(temp1));
	READ_FROM_DPRAM((void *)&temp2, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(temp2));
	LOGL(DL_INFO,"flag1: %x flag2: %x\n", temp1, temp2);

	/* @LDK@ send init end code to phone */
	dpram_unlock_write(__func__);

	g_dump_on = 1;
    // If it is configured to dump both AP and CP, reset both AP and CP here.
    
	return 0;

}

static int dpram_phone_ramdump_off(void)
{
	const u16 rdump_flag1 = 0x00aa;
	const u16 rdump_flag2 = 0x0001;

	LOGL(DL_INFO, "Ramdump OFF.\n");
	
	g_dump_on = 0;
	if(dpram_lock_write(__func__) < 0)
		return -EAGAIN;

	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS,    &rdump_flag1, sizeof(rdump_flag1));
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &rdump_flag2, sizeof(rdump_flag2));
	/* @LDK@ send init end code to phone */
	dpram_unlock_write(__func__);

//	usb_switch_mode(1);
	
	g_phone_sync = 0;

	dpram_phone_reset();
	return 0;

}

#ifdef CONFIG_PROC_FS
static int dpram_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	char *p = page;
	int len;

	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;

	int fih, fit, foh, fot;
	int rih, rit, roh, rot;

#ifdef _ENABLE_ERROR_DEVICE
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;
#endif	/* _ENABLE_ERROR_DEVICE */

	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));
	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fmt_out_tail));
	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(raw_out_tail));

	fih = dpram_table[FORMATTED_INDEX].in_head_saved;
	fit = dpram_table[FORMATTED_INDEX].in_tail_saved;
	foh = dpram_table[FORMATTED_INDEX].out_head_saved;
	fot = dpram_table[FORMATTED_INDEX].out_tail_saved;
	rih = dpram_table[RAW_INDEX].in_head_saved;
	rit = dpram_table[RAW_INDEX].in_tail_saved;
	roh = dpram_table[RAW_INDEX].out_head_saved;
	rot = dpram_table[RAW_INDEX].out_tail_saved;
	in_interrupt = *g_dpram_mbx_AB;
	out_interrupt = *g_dpram_mbx_BA;

#ifdef _ENABLE_ERROR_DEVICE
	memset((void *)buf, '\0', DPRAM_ERR_MSG_LEN);
	local_irq_save(flags);
	memcpy(buf, dpram_err_buf, DPRAM_ERR_MSG_LEN - 1);
	local_irq_restore(flags);
#endif	/* _ENABLE_ERROR_DEVICE */

	p += sprintf(p,
			"-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"|R MAGIC CODE\t\t| 0x%04x\n"
			"|R ENABLE CODE\t\t| 0x%04x\n"
			"|R PHONE->PDA FMT HEAD\t| %u\n"
			"|R PHONE->PDA FMT TAIL\t| %u\n"
			"|R PDA->PHONE FMT HEAD\t| %u\n"
			"|R PDA->PHONE FMT TAIL\t| %u\n"
			"|R PHONE->PDA RAW HEAD\t| %u\n"
			"|R RPHONE->PDA RAW TAIL\t| %u\n"
			"|R PDA->PHONE RAW HEAD\t| %u\n"
			"|R PDA->PHONE RAW TAIL\t| %u\n"
			"-------------------------------------\n"
			"| FMT PHONE->PDA HEAD\t| %d\n"
			"| FMT PHONE->PDA TAIL\t| %d\n"
			"| FMT PDA->PHONE HEAD\t| %d\n"
			"| FMT PDA->PHONE TAIL\t| %d\n"
			"-------------------------------------\n"
			"| RAW PHONE->PDA HEAD\t| %d\n"
			"| RAW PHONE->PDA TAIL\t| %d\n"
			"| RAW PDA->PHONE HEAD\t| %d\n"
			"| RAW PDA->PHONE TAIL\t| %d\n"
			"-------------------------------------\n"
			"| PHONE->PDA MAILBOX\t| 0x%04x\n"
			"| PDA->PHONE MAILBOX\t| 0x%04x\n"
			"-------------------------------------\n"
#ifdef _ENABLE_ERROR_DEVICE
			"| LAST PHONE ERR MSG\t| %s\n"
#endif	/* _ENABLE_ERROR_DEVICE */
			"| PHONE ACTIVE\t\t| %s\n"
			"| DPRAM INT Level\t| %d\n"
			"-------------------------------------\n",
			magic, enable,
			fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
			raw_in_head, raw_in_tail, raw_out_head, raw_out_tail,
			fih, fit, foh, fot, 
			rih, rit, roh, rot,
			in_interrupt, out_interrupt,

#ifdef _ENABLE_ERROR_DEVICE
			(buf[0] != '\0' ? buf : "NONE"),
#endif	/* _ENABLE_ERROR_DEVICE */
			(dpram_phone_getstatus() ? "ACTIVE" : "INACTIVE"),
				gpio_get_value(IRQ_QSC_ACTIVE)
		);

	len = (p - page) - off;
	if (len < 0) {
		len = 0;
	}

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif /* CONFIG_PROC_FS */

/* dpram tty file operations. */
static int dpram_tty_open(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = &dpram_table[tty->index];

	device->serial.tty = tty;
	device->serial.open_count++;

	if (device->serial.open_count > 1) {
		device->serial.open_count--;
		return -EBUSY;
	}

	tty->driver_data = (void *)device;
	tty->low_latency = 1;
	return 0;
}

static void dpram_tty_close(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device && (device == &dpram_table[tty->index])) {
		down(&device->serial.sem);
		device->serial.open_count--;
		device->serial.tty = NULL;
		up(&device->serial.sem);
	}
}

static int dpram_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (!device) {
		return 0;
	}

	return dpram_write(device, buffer, count);
}

static int dpram_tty_write_room(struct tty_struct *tty)
{
	int avail;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		head = device->out_head_saved;
		tail = device->out_tail_saved;
		avail = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;

		return avail;
	}

	return 0;
}


static long dpram_tty_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned int val;
	LOGL(DL_DEBUG, "IOCTL cmd=0x%x\n",cmd);
	switch (cmd) {
		case DPRAM_PHONE_ON:
			g_phone_sync = 0;
			g_dump_on = 0;
			phone_power_off_sequence=0;
            LOGA("IOCTL cmd = DPRAM_PHONE_ON\n");
			dpram_phone_power_on();
			return 0;

		case DPRAM_PHONE_GETSTATUS:
            LOGA("IOCTL cmd = DPRAM_PHONE_GETSTATUS\n");
			val = dpram_phone_getstatus();
			return copy_to_user((unsigned int *)arg, &val, sizeof(val));

		case DPRAM_PHONE_RESET:
			g_phone_sync = 0;
			phone_power_off_sequence =0;
            LOGA("IOCTL cmd = DPRAM_PHONE_RESET\n");
			dpram_phone_reset();
			return 0;

		case DPRAM_PHONE_OFF:
            g_phone_sync = 0;
            phone_power_off_sequence = 1;
            LOGA("IOCTL cmd = DPRAM_PHONE_OFF\n");
			dpram_phone_power_off();
			return 0;

		// Slient reset
		case MBX_CMD_PHONE_RESET:
            LOGA("IOCTL cmd = MBX_CMD_PHONE_RESET\n");
			request_phone_reset();
			return 0;

		case DPRAM_PHONE_RAMDUMP_ON:
            LOGA("IOCTL cmd = DPRAM_PHONE_RAMDUMP_ON\n");
			dpram_phone_ramdump_on();
			return 0;

		case DPRAM_PHONE_RAMDUMP_OFF:
            LOGA("IOCTL cmd = DPRAM_PHONE_RAMDUMP_OFF\n");
			dpram_phone_ramdump_off();
			return 0;

		case DPRAM_EXTRA_MEM_RW:
		{
			struct _param_em param;

			val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
			if (dpram_extra_mem_rw(&param) < 0) {
				LOGE("external memory access fail..\n");
				return -1;
			}
			if (!param.rw) {	//read
				return copy_to_user((unsigned long *)arg, &param, sizeof(param));
			}

			return 0;
		}

		default:
		LOGE("unknown cmd IOCTL cmd=0x%x\n!!\n",cmd);
	
			break;
	}

	return -ENOIOCTLCMD;
}

static int dpram_tty_chars_in_buffer(struct tty_struct *tty)
{
	int data;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		head = device->out_head_saved;
		tail = device->out_tail_saved;
		data = (head > tail) ? head - tail - 1 :
			device->out_buff_size - tail + head;

		return data;
	}

	return 0;
}

#ifdef _ENABLE_ERROR_DEVICE
static int dpram_err_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);

	unsigned long flags;
	ssize_t ret;
	size_t ncopy;

	add_wait_queue(&dpram_err_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	while (1) {
		local_irq_save(flags);

		if (dpram_err_len) {
			ncopy = min(count, dpram_err_len);

			if (copy_to_user(buf, dpram_err_buf, ncopy)) {
				ret = -EFAULT;
			}

			else {
				ret = ncopy;
			}

			dpram_err_len = 0;
			
			local_irq_restore(flags);
			break;
		}

		local_irq_restore(flags);

		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		schedule();
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&dpram_err_wait_q, &wait);

	return ret;
}

static int dpram_err_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &dpram_err_async_q);
}

static unsigned int dpram_err_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	poll_wait(filp, &dpram_err_wait_q, wait);
	return ((dpram_err_len) ? (POLLIN | POLLRDNORM) : 0);
}
#endif	/* _ENABLE_ERROR_DEVICE */

/* handlers. */
static void res_ack_tasklet_handler(unsigned long data)
{
	dpram_device_t *device = (dpram_device_t *)data;

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
				tty->ldisc->ops->write_wakeup) {
			(tty->ldisc->ops->write_wakeup)(tty);
		}

		wake_up_interruptible(&tty->write_wait);
	}
}

static void fmt_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;
	int cnt = 0;

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		while (dpram_get_read_available(device)) {
			ret = dpram_read_fmt(device, non_cmd);

            if (!ret) cnt++;

			if (cnt > 10) {
				dpram_drop_data(device);
				break;
			}
			if (ret < 0) {
				LOGE("FMT dpram_read_fmt failed\n");
				/* TODO: ... wrong.. */
			}
         tty->low_latency = 0;
				tty_flip_buffer_push(tty);
			}
	}

	else {
		dpram_drop_data(device);
	}
}

static void raw_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;
	
	while (dpram_get_read_available(device)) {
		ret = dpram_read_raw(device, non_cmd);
		if (ret < 0) {
			LOGE("RAW dpram_read_raw failed\n");
			/* TODO: ... wrong.. */
		}
	}
}

static void cmd_req_active_handler(void)
{
	dpram_send_mbx_BA(INT_COMMAND(MBX_CMD_RES_ACTIVE));
}

/* static void cmd_error_display_handler(void)
 * 
 * this fucntion was called by dpram irq handler and phone active irq hander
 * first this fucntion check the log level then jump to send error message to ril
 * or CP Upload mode
 */
static void cmd_error_display_handler(void)
{

#ifdef _ENABLE_ERROR_DEVICE
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;

    memset((void *)buf, 0, sizeof (buf));

	// check the reset or crash
//	if (!dpram_phone_getstatus()) {  --- can't catch the CDMA watchdog reset!!
	if(dpram_err_cause==0x000000F3)
	{
		memcpy((void *)buf, "8 $PHONE-OFF", sizeof("8 $PHONE-OFF"));
    }
    else {
        buf[0] = 'C';
        buf[1] = 'D';
        buf[2] = 'M';
        buf[3] = 'A';
        buf[4] = ' ';

		READ_FROM_DPRAM((buf + 5), DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS, sizeof (buf) - 6);
    }                   

	LOGE("[PHONE ERROR] ->> %s\n", buf);

	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);

	dpram_err_len = 64;


	local_irq_restore(flags);

	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);

#endif	/* _ENABLE_ERROR_DEVICE */

}

void slot_switch_handler2(int slot_switch)
{
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;

	memset((void*)buf, 0, sizeof(buf));

	if(slot_switch == 8) {
		memcpy((void*)buf, "$SIM-SLOT-0", sizeof("$SIM-SLOT-0"));
	}
	else if(slot_switch == 9) {
		memcpy((void*)buf, "$SIM-SLOT-1", sizeof("$SIM-SLOT-1"));
	}
	else {
		memcpy((void*)buf, "$Unknown SIM Info", sizeof("$Unknown SIM Info"));
	}
	printk("dpram err string : %s\n", buf);
	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	dpram_err_len = 64;
	local_irq_restore(flags);
	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);
}
EXPORT_SYMBOL(slot_switch_handler2);


static void cmd_phone_start_handler(void)
{
	LOGL(DL_INFO, "Received 0xc8 from MailboxAB (Phone Boot OK).\n");
	if(!g_phone_sync) {
		dpram_init_and_report();
	}
}

static void cmd_req_time_sync_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_phone_deep_sleep_handler(void)
{
	/* TODO: add your codes here.. */
}

static void command_handler(u16 cmd)
{
	switch (cmd) {
		case MBX_CMD_REQ_ACTIVE:
			cmd_req_active_handler();
			break;

		case MBX_CMD_CDMA_DEAD:
            LOGE("received MBX_CMD_CDMA_DEAD\n");
            cmd_error_display_handler();
            break;
		case MBX_CMD_ERR_DISPLAY:
            LOGE("received MBX_CMD_ERR_DISPLAY\n");
			cmd_error_display_handler();
			break;

//		case MBX_CMD_PHONE_COMMON_BOOT:
		case MBX_CMD_PHONE_START:
		LOGE("received MBX_CMD_PHONE_START\n");

			cmd_phone_start_handler();
			break;

		case MBX_CMD_REQ_TIME_SYNC:
				LOGE("received MBX_CMD_REQ_TIME_SYNC\n");

			cmd_req_time_sync_handler();
			break;

		case MBX_CMD_PHONE_DEEP_SLEEP:
				LOGE("received MBX_CMD_PHONE_DEEP_SLEEP\n");

			cmd_phone_deep_sleep_handler();
			break;

		case MBX_CMD_DPRAM_DOWN:
				LOGE("received MBX_CMD_DPRAM_DOWN\n");

			dpram_power_down();
			break;

		case MBX_CMD_CP_WAKEUP_START:
				LOGE("received MBX_CMD_CP_WAKEUP_START\n");

			dpram_powerup_start();
			break;

		default:
            LOGA("Unknown command (0x%04X)\n", cmd);
	}
}

static void non_command_handler(u16 non_cmd)
{
	u16 head, tail;

	/* @LDK@ formatted check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	if (head != tail) {
		non_cmd |= INT_MASK_SEND_F;
	}else {
		if(non_cmd & INT_MASK_REQ_ACK_F)
			LOGE("=====> FMT DATA EMPTY & REQ_ACK_F\n");
            dpram_send_mbx_BA(INT_NON_COMMAND(INT_MASK_RES_ACK_F));
	}
	
	/* @LDK@ raw check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(tail));

	if (head != tail) {
		non_cmd |= INT_MASK_SEND_R;
	}else {
		if(non_cmd & INT_MASK_REQ_ACK_R)
			LOGE("=====> RAW DATA EMPTY & REQ_ACK_R\n");
            dpram_send_mbx_BA(INT_NON_COMMAND(INT_MASK_RES_ACK_R));
	}

	/* @LDK@ +++ scheduling.. +++ */
	if (non_cmd & INT_MASK_SEND_F) {
		dpram_tasklet_data[FORMATTED_INDEX].device = &dpram_table[FORMATTED_INDEX];
		dpram_tasklet_data[FORMATTED_INDEX].non_cmd = non_cmd;
		fmt_send_tasklet.data = (unsigned long)&dpram_tasklet_data[FORMATTED_INDEX];
		tasklet_schedule(&fmt_send_tasklet);
	}
	if (non_cmd & INT_MASK_SEND_R) {
		dpram_tasklet_data[RAW_INDEX].device = &dpram_table[RAW_INDEX];
		dpram_tasklet_data[RAW_INDEX].non_cmd = non_cmd;
		raw_send_tasklet.data = (unsigned long)&dpram_tasklet_data[RAW_INDEX];
		/* @LDK@ raw buffer op. -> soft irq level. */
		tasklet_hi_schedule(&raw_send_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_F) {
		tasklet_schedule(&fmt_res_ack_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_R) {
		tasklet_hi_schedule(&raw_res_ack_tasklet);
	}

}

static inline void check_int_pin_level(void)
{
	u16 mask = 0, cnt = 0;

	while (cnt++ < 3) {
		mask = *g_dpram_mbx_AB;
		if (gpio_get_value(IRQ_INT_IDPRAM_AP_N))
			break;
	}
}

/* @LDK@ interrupt handlers. */
static irqreturn_t dpram_irq_handler(int irq, void *dev_id)
{

	u16 irq_mask = 0;

	irq_mask = *g_dpram_mbx_AB;

	LOGA("IRQ: %x\n", irq_mask);


	/* valid bit verification. @LDK@ */
	if (!(irq_mask & INT_MASK_VALID)) {
		LOGE("Invalid interrupt mask: 0x%04x\n", irq_mask);
		return IRQ_NONE;
	}

	/* command or non-command? @LDK@ */
	if (irq_mask & INT_MASK_COMMAND) {
		irq_mask &= ~(INT_MASK_VALID | INT_MASK_COMMAND);
		command_handler(irq_mask);
	}
	else {
		irq_mask &= ~INT_MASK_VALID;
		non_command_handler(irq_mask);
	}
	
	IDPRAM_INT_CLEAR();
	LOGL(DL_DEBUG, "dpram_irq_handler\n");
	return IRQ_HANDLED;
}


static irqreturn_t idpram_wake_from_CP_irq_handler(int irq, void *dev_id)
{
	wake_lock_timeout(&g_wakelock_dpram, 5*HZ);
	LOGE("idpram_qsc_int_irq_handler() wake_lock_timeout 5 sec \n");


	return IRQ_HANDLED;
}

static irqreturn_t phone_active_irq_handler(int irq, void *dev_id)
{
	LOGE("PHONE_ACTIVE level: %s, phone_sync: %d\n", 
			gpio_get_value(GPIO_QSC_ACTIVE)?"HIGH":"LOW ", g_phone_sync);

#ifdef _ENABLE_ERROR_DEVICE
/* If CDMA was reset by watchdog, phone active low time is very short So, change the IRQ type 
 * to FALING EDGE and don't check the GPIO_QSC_ACTIVE
 */
	//if((g_phone_sync) && (!gpio_get_value(GPIO_QSC_ACTIVE)))
	if(g_phone_sync)
	{
		dpram_err_cause = 0x000000F3;
		//request_phone_reset();	

		if(phone_power_off_sequence != 0x1)
			cmd_error_display_handler();
	}
#endif

	IDPRAM_INT_CLEAR();
	return IRQ_HANDLED;
}
/* basic functions. */
#ifdef _ENABLE_ERROR_DEVICE
static struct file_operations dpram_err_ops = {
	.owner = THIS_MODULE,
	.read = dpram_err_read,
	.fasync = dpram_err_fasync,
	.poll = dpram_err_poll,
	.llseek = no_llseek,

	/* TODO: add more operations */
};
#endif	/* _ENABLE_ERROR_DEVICE */

static struct tty_operations dpram_tty_ops = {
	.open 		= dpram_tty_open,
	.close 		= dpram_tty_close,
	.write 		= dpram_tty_write,
	.write_room = dpram_tty_write_room,
	.ioctl 		= dpram_tty_ioctl,
	.chars_in_buffer = dpram_tty_chars_in_buffer,

	/* TODO: add more operations */
};

#ifdef _ENABLE_ERROR_DEVICE

static void unregister_dpram_err_device(void)
{
	unregister_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE);
	class_destroy(dpram_class);
}

static int register_dpram_err_device(void)
{
	/* @LDK@ 1 = formatted, 2 = raw, so error device is '0' */
	struct device *dpram_err_dev_t;
	int ret = register_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE, &dpram_err_ops);

	if ( ret < 0 )
	{
		return ret;
	}

	dpram_class = class_create(THIS_MODULE, "err");

	if (IS_ERR(dpram_class))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	dpram_err_dev_t = device_create(dpram_class, NULL,
			MKDEV(DRIVER_MAJOR_NUM, 0), NULL, DPRAM_ERR_DEVICE);

	if (IS_ERR(dpram_err_dev_t))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	return 0;
}
#endif	/* _ENABLE_ERROR_DEVICE */

static int register_dpram_driver(void)
{
	int retval = 0;

	/* @LDK@ allocate tty driver */
	dpram_tty_driver = alloc_tty_driver(MAX_INDEX);

	if (!dpram_tty_driver) {
		return -ENOMEM;
	}

	/* @LDK@ initialize tty driver */
	dpram_tty_driver->owner = THIS_MODULE;
	dpram_tty_driver->magic = TTY_DRIVER_MAGIC;
	dpram_tty_driver->driver_name = DRIVER_NAME;
	dpram_tty_driver->name = "dpramCDMA";
	dpram_tty_driver->major = DRIVER_MAJOR_NUM;
	dpram_tty_driver->minor_start = 1;
	dpram_tty_driver->num = 1;
	dpram_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	dpram_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	dpram_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	dpram_tty_driver->init_termios = tty_std_termios;
	dpram_tty_driver->init_termios.c_cflag =
		(B115200 | CS8 | CREAD | CLOCAL | HUPCL);

	tty_set_operations(dpram_tty_driver, &dpram_tty_ops);

	dpram_tty_driver->ttys = dpram_tty;
	dpram_tty_driver->termios = dpram_termios;
	dpram_tty_driver->termios_locked = dpram_termios_locked;

	/* @LDK@ register tty driver */
	retval = tty_register_driver(dpram_tty_driver);

	if (retval) {
		LOGE("tty_register_driver error\n");
		put_tty_driver(dpram_tty_driver);
		return retval;
	}

	return 0;
}

static void unregister_dpram_driver(void)
{
	tty_unregister_driver(dpram_tty_driver);
}

/*
 * MULTI PDP FUNCTIONs
 */

static int multipdp_ioctl(struct inode *inode, struct file *file, 
			      unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static struct file_operations multipdp_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl =	multipdp_ioctl,
	.llseek =	no_llseek,
};

static struct miscdevice multipdp_dev = {
	.minor =	132, //MISC_DYNAMIC_MINOR,
	.name =		APP_DEVNAME,
	.fops =		&multipdp_fops,
};

static inline struct pdp_info * pdp_get_serdev(const char *name)
{
	int slot;
	struct pdp_info *dev;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		dev = pdp_table[slot];
		if (dev && dev->type == DEV_TYPE_SERIAL &&
		    strcmp(name, dev->vs_dev.tty_name) == 0) {
			return dev;
		}
	}
	return NULL;
}

static inline struct pdp_info * pdp_remove_dev(u8 id)
{
	int slot;
	struct pdp_info *dev;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			dev = pdp_table[slot];
			pdp_table[slot] = NULL;
			return dev;
		}
	}
	return NULL;
}

static int vs_open(struct tty_struct *tty, struct file *filp)
{
	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name); // 2.6 kernel porting

	if (dev == NULL) {
		return -ENODEV;
	}

	tty->driver_data = (void *)dev;
	tty->low_latency = 1;
	dev->vs_dev.tty = tty;
	dev->vs_dev.refcount++;
	LOGE("%s, refcount: %d \n", tty->driver->name, dev->vs_dev.refcount);
	return 0;
}

static void vs_close(struct tty_struct *tty, struct file *filp)
{
	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name); 

	if (!dev )
		return;
	dev->vs_dev.refcount--;
	LOGE("%s, refcount: %d \n", tty->driver->name, dev->vs_dev.refcount);

	return;
}

static int pdp_mux(struct pdp_info *dev, const void *data, size_t len   )
{
	int ret;
	size_t nbytes;
	u8 *tx_buf;
	struct pdp_hdr *hdr;
	const u8 *buf;

	tx_buf = dev->tx_buf;
	hdr = (struct pdp_hdr *)(tx_buf + 1);
	buf = data;

	hdr->id = dev->id;
	hdr->control = 0;

	while (len) {
		if (len > MAX_PDP_DATA_LEN) {
			nbytes = MAX_PDP_DATA_LEN;
		} else {
			nbytes = len;
		}
		hdr->len = nbytes + sizeof(struct pdp_hdr);

		tx_buf[0] = 0x7f;
		
		memcpy(tx_buf + 1 + sizeof(struct pdp_hdr), buf,  nbytes);
		
		tx_buf[1 + hdr->len] = 0x7e;

		ret = dpram_write(&dpram_table[RAW_INDEX], tx_buf, hdr->len + 2);

		if (ret < 0) {
			LOGE("write_to_dpram() failed: %d\n", ret);
			return ret;
		}
		buf += nbytes;
		len -= nbytes;
	}
	return 0;
}


static int vs_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	int ret;
	struct pdp_info *dev = (struct pdp_info *)tty->driver_data;

	ret = pdp_mux(dev, buf, count);

	if (ret == 0) {
		ret = count;
	}

	return ret;
}

static int vs_write_room(struct tty_struct *tty) 
{
	return 8192*2;
}

static int vs_chars_in_buffer(struct tty_struct *tty) 
{
	return 0;
}

static int vs_ioctl(struct tty_struct *tty, struct file *file, 
		    unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static struct tty_operations multipdp_tty_ops = {
	.open 		= vs_open,
	.close 		= vs_close,
	.write 		= vs_write,
	.write_room = vs_write_room,
	.ioctl 		= vs_ioctl,
	.chars_in_buffer = vs_chars_in_buffer,

	/* TODO: add more operations */
};

static struct tty_driver* get_tty_driver_by_id(struct pdp_info *dev)
{
	int index = 0;

	switch (dev->id) {
		case 1:		index = 0;	break;
		case 7:		index = 1;	break;
		case 9:		index = 2;	break;
//		case 27:	index = 3;	break;
		default:	index = 0;
	}

	return &dev->vs_dev.tty_driver[index];
}

static int get_minor_start_index(int id)
{
	int start = 0;

	switch (id) {
		case 1:		start = 0;	break;
		case 7:		start = 1;	break;
		case 9:		start = 2;	break;
//		case 27:	start = 3;	break;
		default:	start = 0;
	}

	return start;
}

static int vs_add_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver;

	tty_driver = get_tty_driver_by_id(dev);

	if (!tty_driver) {
		LOGE("tty driver is NULL!\n");
		return -1;
	}

	kref_init(&tty_driver->kref);

	tty_driver->magic	= TTY_DRIVER_MAGIC;
	tty_driver->driver_name	= APP_DEVNAME;//"multipdp";
	tty_driver->name	= dev->vs_dev.tty_name;
	tty_driver->major	= CSD_MAJOR_NUM;
	tty_driver->minor_start = get_minor_start_index(dev->id);
	tty_driver->num		= 1;
	tty_driver->type	= TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype	= SERIAL_TYPE_NORMAL;
	tty_driver->flags	= TTY_DRIVER_REAL_RAW;
//	kref_set(&tty_driver->kref, dev->vs_dev.refcount);
	tty_driver->ttys	= dev->vs_dev.tty_table; // 2.6 kernel porting
	tty_driver->termios	= dev->vs_dev.termios;
	tty_driver->termios_locked	= dev->vs_dev.termios_locked;

	tty_set_operations(tty_driver, &multipdp_tty_ops);
	return tty_register_driver(tty_driver);
}

static void vs_del_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver = NULL;

	tty_driver = get_tty_driver_by_id(dev);
	tty_unregister_driver(tty_driver);
}

static inline void check_pdp_table(char * func, int line)
{
	int slot;
	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if(pdp_table[slot])
            LOGA("[%s,%d] addr: %x slot: %d id: %d, name: %s\n", func, line, pdp_table[slot], slot, pdp_table[slot]->id, pdp_table[slot]->vs_dev.tty_name);
	}
}

static inline struct pdp_info * pdp_get_dev(u8 id)
{
	int slot;


	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			return pdp_table[slot];
		}
	}
	return NULL;
}

static inline int pdp_add_dev(struct pdp_info *dev)
{
	int slot;

	if (pdp_get_dev(dev->id)) {
		return -EBUSY;
	}

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] == NULL) {
			pdp_table[slot] = dev;
			return slot;
		}
	}
	return -ENOSPC;
}

static int pdp_activate(pdp_arg_t *pdp_arg, unsigned type, unsigned flags)
{
	int ret;
	struct pdp_info *dev;

	LOGL(DL_INFO, "id: %d\n", pdp_arg->id);

	dev = kmalloc(sizeof(struct pdp_info) + MAX_PDP_PACKET_LEN, GFP_KERNEL);
	if (dev == NULL) {
		LOGE("out of memory\n");
		return -ENOMEM;
	}
	memset(dev, 0, sizeof(struct pdp_info));

	dev->id = pdp_arg->id;

	dev->type = type;
	dev->flags = flags;
	dev->tx_buf = (u8 *)(dev + 1);

	if (type == DEV_TYPE_SERIAL) {
		init_MUTEX(&dev->vs_dev.write_lock);
		strcpy(dev->vs_dev.tty_name, pdp_arg->ifname);

		ret = vs_add_dev(dev);
		if (ret < 0) {
			kfree(dev);
			return ret;
		}

		mutex_lock(&pdp_lock);
		ret = pdp_add_dev(dev);
		if (ret < 0) {
			LOGE("pdp_add_dev() failed\n");
			mutex_unlock(&pdp_lock);
			vs_del_dev(dev);
			kfree(dev);
			return ret;
		}
		mutex_unlock(&pdp_lock);

		{
			struct tty_driver * tty_driver = get_tty_driver_by_id(dev);

			LOGL(DL_INFO, "%s(id: %u) serial device is created.\n",
					tty_driver->name, dev->id);
		}
	}

	return 0;
}

static int multipdp_init(void)
{
	int i;

	pdp_arg_t pdp_args[NUM_PDP_CONTEXT] = {
		{ .id = 1, .ifname = "ttyCSD" },
		{ .id = 7, .ifname = "ttyCDMA" },
		{ .id = 9, .ifname = "ttyTRFB" },
//		{ .id = 27, .ifname = "ttyCIQ" },
	};


	/* create serial device for Circuit Switched Data */
	for (i = 0; i < NUM_PDP_CONTEXT; i++) {
		if (pdp_activate(&pdp_args[i], DEV_TYPE_SERIAL, DEV_FLAG_STICKY) < 0) {
			LOGE("failed to create a serial device for %s\n", pdp_args[i].ifname);
		}
	}

	return 0;
}

/*
* DPRAM DRIVER INITIALIZE FUNCTIONs
*/
static int dpram_init_hw(void)
{
	//1) Initialize the interrupt pins
	irq_set_irq_type(IRQ_INT_IDPRAM_AP_N,IRQ_TYPE_LEVEL_LOW);
//	set_irq_type(IRQ_QSC_ACTIVE,IRQ_TYPE_EDGE_BOTH);
	irq_set_irq_type(IRQ_QSC_INT,IRQ_TYPE_EDGE_RISING);
	irq_set_irq_type(IRQ_QSC_ACTIVE,IRQ_TYPE_EDGE_FALLING);

    // 2)EINT8 QSC_ACTIVE (GPIO_INTERRUPT)
	#define FILTER_EINT8_EN (0x1<<7)
	#define FILTER_EINT8_SEL_DEGIT (0x1<<6)
	__raw_writel(__raw_readl(S5PV210_EINT1FLTCON0)|(FILTER_EINT8_EN & (~FILTER_EINT8_SEL_DEGIT)),S5PV210_EINT1FLTCON0);
	
	//3)EINT14 : QSC_INT (GPIO_INTERRUPT)
	#define FILTER_EINT14_EN (0x1<<7)
	#define FILTER_EINT14_SEL_DEGIT (0x1<<6)
	__raw_writel(__raw_readl(S5PV210_EINT1FLTCON1)|(FILTER_EINT14_EN & (~FILTER_EINT14_SEL_DEGIT)),S5PV210_EINT1FLTCON1);

	// 4)gpioj2-j4 are for Modem if
	__raw_writel( 
		(S5PV210_GPJ2_0_MSM_DATA_0 | S5PV210_GPJ2_1_MSM_DATA_1 | S5PV210_GPJ2_2_MSM_DATA_2 
		| S5PV210_GPJ2_3_MSM_DATA_3	| S5PV210_GPJ2_4_MSM_DATA_4 | S5PV210_GPJ2_5_MSM_DATA_5 
		| S5PV210_GPJ2_6_MSM_DATA_6 | S5PV210_GPJ2_7_MSM_DATA_7)
		,S5PV210_GPJ2_BASE);
	__raw_writel(
		(S5PV210_GPJ3_0_MSM_DATA_8 | S5PV210_GPJ3_1_MSM_DATA_9 | S5PV210_GPJ3_2_MSM_DATA_10
		| S5PV210_GPJ3_3_MSM_DATA_11 | S5PV210_GPJ3_4_MSM_DATA_12 | S5PV210_GPJ3_5_MSM_DATA_13
		| S5PV210_GPJ3_6_MSM_DATA_14 | S5PV210_GPJ3_7_MSM_DATA_15)
		,S5PV210_GPJ3_BASE);
	__raw_writel(
		(S5PV210_GPJ4_0_MSM_CSn | S5PV210_GPJ4_1_MSM_WEn | S5PV210_GPJ4_2_MSM_Rn 
		| S5PV210_GPJ4_3_MSM_IRQn | S5PV210_GPJ4_4_MSM_ADVN)
		,S5PV210_GPJ4_BASE);


	// 5)PDA_ACTIVE,QSC_PHONE_ON,QSC_RESET_N
	gpio_direction_output(GPIO_PDA_ACTIVE, GPIO_LEVEL_HIGH);
//	gpio_direction_output(GPIO_PDA_ACTIVE2, GPIO_LEVEL_HIGH);

	gpio_direction_output(GPIO_QSC_PHONE_ON, GPIO_LEVEL_LOW);
	gpio_direction_output(GPIO_QSC_RESET_N, GPIO_LEVEL_LOW);

}


static int dpram_shared_bank_remap(void)
{
	// Get internal DPRAM / SFR Virtual address 
	//1) dpram base
	g_idpram_region = (volatile void *)ioremap_nocache(IDPRAM_PHYSICAL_ADDR, IDPRAM_SIZE); 
	if(g_idpram_region == NULL) {
//		printk(KERN_ERR "failed ioremap g_idpram_region\n");
		LOGE("failed ioremap g_idpram_region\n");
	}
	// 2) sfr base
	g_dpram_sfr_base = (volatile IDPRAM_SFR __iomem *)ioremap_nocache(IDPRAM_SFR_PHYSICAL_ADDR, IDPRAM_SFR_SIZE); 
	if(g_dpram_sfr_base == NULL) {
//		printk(KERN_ERR "failed ioremap g_dpram_sfr_base\n");
		LOGE("failed ioremap g_dpram_sfr_base\n");
        iounmap(g_idpram_region);
	}
		
	// 3) Initialize the Modem interface block(internal DPRAM) 
	// TODO : Use DMA controller? ask to sys.lsi
	// set Modem interface config register
	g_dpram_sfr_base->mifcon = (IDPRAM_MIFCON_FIXBIT|IDPRAM_MIFCON_INT2APEN|IDPRAM_MIFCON_INT2MSMEN); //FIXBIT enable, interrupt enable AP,MSM(CP)
	g_dpram_sfr_base->mifpcon = (IDPRAM_MIFPCON_ADM_MODE); //mux mode

	g_dpram_mbx_BA = (volatile u16*)(g_idpram_region + IDPRAM_AP2MSM_INT_OFFSET);
	g_dpram_mbx_AB = (volatile u16*)(g_idpram_region + IDPRAM_MSM2AP_INT_OFFSET);

	// write the normal boot magic key for CDMA boot
	*((unsigned int *)g_idpram_region)= DPRAM_BOOT_NORMAL;
	
	atomic_set(&g_dpram_lock_read, 0);
	atomic_set(&g_dpram_lock_write, 0);

	return 0;
}

static void dpram_init_devices(void)
{
	int i;

	for (i = 0; i < MAX_INDEX; i++) {
		init_MUTEX(&dpram_table[i].serial.sem);

		dpram_table[i].serial.open_count = 0;
		dpram_table[i].serial.tty = NULL;
	}
}

void dpram_wakeup_init(void)
{
	g_dpram_sfr_base->mifcon = (IDPRAM_MIFCON_FIXBIT|IDPRAM_MIFCON_INT2APEN|IDPRAM_MIFCON_INT2MSMEN);
	g_dpram_sfr_base->mifpcon = (IDPRAM_MIFPCON_ADM_MODE);

	// mux GPIO_DPRAM_INT_CP_N to dpram interrupt
	__raw_writel(
	(S5PV210_GPJ4_0_MSM_CSn | S5PV210_GPJ4_1_MSM_WEn | S5PV210_GPJ4_2_MSM_Rn 
	| S5PV210_GPJ4_3_MSM_IRQn | S5PV210_GPJ4_4_MSM_ADVN)
	,S5PV210_GPJ4_BASE);
}

#define WAKESTART_TIMEOUT (HZ/10)
#define WAKESTART_TIMEOUT_RETRY 5

void dpram_wait_wakeup_start(void)
{
	int wakeup_retry=WAKESTART_TIMEOUT_RETRY, timeout_ret=0;

	//LOGA("%d %x %u \n", wakeup_retry, wakeup_retry, wakeup_retry );
	do {
		init_completion(&g_complet_dpramdown);
		dpram_send_mbx_BA_cmd(INT_COMMAND(MBX_CMD_PDA_WAKEUP_INT));
		timeout_ret = wait_for_completion_timeout(&g_complet_dpramdown, WAKESTART_TIMEOUT);
		printk("wake_up start cnt=%d\n", (WAKESTART_TIMEOUT_RETRY-wakeup_retry));
	} while(!timeout_ret && wakeup_retry--);

	if (!timeout_ret) {
        LOGE("dpram wake up start  T-I-M-E-O-U-T !! \n");
		//dpram_qsc_timeout_handler();
	}
	g_dpram_wpend = IDPRAM_WPEND_UNLOCK; // dpram write unlock
	
	LOGA("wakeup_start done!!\n");
}

static void kill_tasklets(void)
{
	tasklet_kill(&fmt_res_ack_tasklet);
	tasklet_kill(&raw_res_ack_tasklet);

	tasklet_kill(&fmt_send_tasklet);
	tasklet_kill(&raw_send_tasklet);
}

static int register_interrupt_handler(void)
{

	unsigned int dpram_irq,idpram_wakeup_irq_, phone_active_irq;
	int retval = 0;
	
	dpram_irq = IRQ_INT_IDPRAM_AP_N;  //change it
	idpram_wakeup_irq_ = IRQ_QSC_INT;
	phone_active_irq = IRQ_QSC_ACTIVE;

	//dpram_clear();

	//1)dpram interrupt 
	IDPRAM_INT_CLEAR();
	retval = request_irq(dpram_irq, dpram_irq_handler, IRQF_DISABLED, "idpram irq", NULL);
	if (retval) {
		LOGE("DPRAM interrupt handler failed.\n");
		unregister_dpram_driver();
		return -1;
	}

    //2) wake up for internal dpram
	retval = request_irq(idpram_wakeup_irq_, idpram_wake_from_CP_irq_handler, IRQF_DISABLED, "QSC_INT irq", NULL);
	if (retval) {
		LOGE("DPRAM interrupt handler failed.\n");
		unregister_dpram_driver();
		return -1;
	}

	//3) phone active interrupt
	retval = request_irq(phone_active_irq, phone_active_irq_handler, IRQF_DISABLED, "QSC Active", NULL);
	if (retval) {
		LOGE("Phone active interrupt handler failed.\n");
		free_irq(phone_active_irq, NULL);
		unregister_dpram_driver();
		return -1;
	}

	return 0;
}

static void check_miss_interrupt(void)
{
	unsigned long flags;

	if(gpio_get_value(GPIO_QSC_ACTIVE) &&(!gpio_get_value(IRQ_INT_IDPRAM_AP_N))){
		local_irq_save(flags);
		dpram_irq_handler(IRQ_INT_IDPRAM_AP_N, NULL);
		local_irq_restore(flags);
	}
}

/*
 * INTERANL DPRAM POWER DOWN FUNCTION
 */
 
/*
 * void dpram_pre_power_down()
 * 
 * lock the AP write dpram and send the SLEEP INT to Phone 
 */
 #define DPRAMDOWN_TIMEOUT (HZ*3)
 #define DPRAM_PDNINTR_RETRY_CNT 2
 static inline void dpram_pre_power_down(void)
{
	int ret=0;
	u16 in_intr=0, timeout_ret, suspend_retry=2;
	g_dpram_wpend = IDPRAM_WPEND_LOCK;		// dpram write lock

	/*
	 * if some intrrupt was received by cp, dpram hold the wake lock and pass the suspend mode.
	 */
	if(dpram_get_lock_read()==0)
	{
		/*
		 * retry sending 0xCE if cp dosn't send in timout
		 */
		do {
			init_completion(&g_complet_dpramdown);
			dpram_send_mbx_BA_cmd(INT_COMMAND(MBX_CMD_PDA_SLEEP_INT));
			//timeout_ret = wait_for_completion_timeout(&g_complet_dpramdown, DPRAMDOWN_TIMEOUT);
            timeout_ret = wait_for_completion_timeout(&g_complet_dpramdown, msecs_to_jiffies(500));
			LOG("suspend_enter cnt=%d\n", (DPRAM_PDNINTR_RETRY_CNT-suspend_retry));
		}while(!timeout_ret && suspend_retry--);

		in_intr = *g_dpram_mbx_AB;
		if(in_intr != 0xCB)
		{				
			LOGE("dpram power down T-I-M-E-O-U-T !! intr=0%4x \n", in_intr);
			dpram_qsc_timeout_handler();
		}
		LOGA("Check The intr after TIMEOUT, aleady 0xCE received.\n");
		LOGL(DL_DEBUG, "MAILBOX_CHECK MB_AB =0x%8x\n",*g_dpram_mbx_AB);

		/*
		 * Because, if dprmam was powered down, cp dpram random intr was ocurred,
		 * So, fixed by muxing cp dpram intr pin to GPIO outup high,..
		 */
		__raw_writel(							// output high
			__raw_readl(S5PV210_GPJ4DAT)| 0x1<<3
			,S5PV210_GPJ4DAT);
		__raw_writel(							// GPIO output mux
			(S5PV210_GPJ4_0_MSM_CSn | S5PV210_GPJ4_1_MSM_WEn | S5PV210_GPJ4_2_MSM_Rn 
			|S5PV210_GPJ4_OUTPUT(3) | S5PV210_GPJ4_4_MSM_ADVN)
			,S5PV210_GPJ4_BASE);

		gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_LOW);
//		gpio_set_value(GPIO_PDA_ACTIVE2, GPIO_LEVEL_LOW);				
	}else {
		//wake_lock_timeout(&g_wakelock_dpram, HZ/4);
		LOGA("Skip the suspned mode - read lock \n");
	}
}

/*
 * void dpram_power_down()
 *
 * This function release the wake_lock 
 * Phone send DRPAM POWER DOWN interrupt and handler call this function.
 */
static void dpram_power_down(void)
{
	LOGA("receive DPRAM POWER DOWN Int 0xCB \n");
	LOGL(DL_INFO, "dpram_power_down lock cnt=%d \n", dpram_get_lock_read());
	complete(&g_complet_dpramdown);
}

static void dpram_powerup_start(void)
{
	LOGA("received Wake done int 0xCE\n");
	complete(&g_complet_dpramdown);
}
	
/*
 * void dpram_power_up()
 *
 * Initialize dpram when ap wake up and send WAKEUP_INT to phone
 * This function will be called by Onedram_resume()
 */
static inline void dpram_power_up(void)
{
    const u16 magic_code = 0x00AA;
 	u16 ac_code = 0x0001;

	dpram_clear();
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));

	dpram_wakeup_init();// initialize the dpram controller

	// check for QSC_INT for debugging
	LOGL(DL_INFO, "dpram_wakeup_init, QSC_INT=%s\n", gpio_get_value(GPIO_QSC_INT)?"HIGH":"LOW");
}

/*
 * DPRAM DRIVER FUNCTIONs
 */
static int dpram_suspend(struct platform_device *dev, pm_message_t state)
{
	dpram_pre_power_down();
/*   - controled by dpram_pre_power_down()
	gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_LOW);
	gpio_set_value(GPIO_PDA_ACTIVE2, GPIO_LEVEL_LOW);
*/
	return 0;
}

static int dpram_resume(struct platform_device *dev)
{
	gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_HIGH);
//	gpio_set_value(GPIO_PDA_ACTIVE2, GPIO_LEVEL_HIGH);
	dpram_power_up();
	dpram_wait_wakeup_start(); // wait the CP ack 0xCE
	check_miss_interrupt();
	return 0;
}

static int __devinit dpram_probe(struct platform_device *dev)
{
	int retval;

	retval = register_dpram_driver();
	if (retval) {
		LOGE("Failed to register dpram (tty) driver.\n");
		return -1;
	}

#ifdef _ENABLE_ERROR_DEVICE
	retval = register_dpram_err_device();
	if (retval) {
		LOGE("Failed to register dpram error device.\n");

		unregister_dpram_driver();
		return -1;
	}
	memset((void *)dpram_err_buf, '\0', sizeof dpram_err_buf);
#endif /* _ENABLE_ERROR_DEVICE */

	retval = misc_register(&multipdp_dev);	/* create app. interface device */
	if (retval < 0) {
		LOGE("misc_register() failed\n");
		return -1;
	}
	multipdp_init();

    retval = dpram_init_hw();
    if (retval < 0)
    {
        LOGE("dpram_init_hw() failed\n");
        return -1;
    }

	dpram_shared_bank_remap();

	dpram_init_devices();

	wake_lock_init(&g_wakelock_dpram, WAKE_LOCK_SUSPEND, "DPRAM_PWDOWN");

	if ((retval = register_interrupt_handler()) < 0) {
        gpio_free(GPIO_QSC_RESET_N);
        gpio_free(GPIO_QSC_PHONE_ON);
        gpio_free(GPIO_PDA_ACTIVE);
	free_irq(IRQ_QSC_ACTIVE, NULL);
	free_irq(IRQ_QSC_INT, NULL);
		return -1;
	}
#ifdef CONFIG_PROC_FS
	create_proc_read_entry(DRIVER_PROC_ENTRY, 0, 0, dpram_read_proc, NULL);
#endif	/* CONFIG_PROC_FS */
//	cdma_slot_switch_handler = slot_switch_handler2;

	//check_miss_interrupt();
	return 0;
}

static int __devexit dpram_remove(struct platform_device *dev)
{

   wake_lock_destroy(&g_wakelock_dpram);

	/* @LDK@ unregister dpram (tty) driver */
	unregister_dpram_driver();

	/* @LDK@ unregister dpram error device */
#ifdef _ENABLE_ERROR_DEVICE
	unregister_dpram_err_device();
#endif
	
	/* remove app. interface device */
	misc_deregister(&multipdp_dev);

	free_irq(IRQ_INT_IDPRAM_AP_N, NULL);
	gpio_free(GPIO_QSC_RESET_N);
    gpio_free(GPIO_QSC_PHONE_ON);
    gpio_free(GPIO_PDA_ACTIVE);
	free_irq(IRQ_QSC_ACTIVE, NULL);
	free_irq(IRQ_QSC_INT, NULL);

	kill_tasklets();

	return 0;
}
static int dpram_shutdown(struct platform_device *dev)
{
   int ret = FALSE;
   
        ret = dpram_remove(dev);
        return ret;
}

u32 dpram_get_phone_dump_stat(void)
{
    return g_dump_on;   
}

EXPORT_SYMBOL(dpram_get_phone_dump_stat);

static struct platform_driver platform_dpram_driver = {
	.probe		= dpram_probe,
	.remove		= __devexit_p(dpram_remove),
	.suspend	= dpram_suspend,
	.resume 	= dpram_resume,
    .shutdown = dpram_shutdown,
	.driver	= {
		.name	= "dpram0-cdma",
	},
};

/* init & cleanup. */
static int __init dpram_init(void)
{
	return platform_driver_register(&platform_dpram_driver);
}

static void __exit dpram_exit(void)
{
	platform_driver_unregister(&platform_dpram_driver);
}

module_init(dpram_init);
module_exit(dpram_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");
MODULE_DESCRIPTION("Internal DPRAM Device Driver.");
MODULE_LICENSE("GPL");


#ifndef __JZ_CAMERA_H__
#define __JZ_CAMERA_H__

#include <media/soc_camera.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>

#define JZ_CAMERA_DATA_HIGH         1
#define JZ_CAMERA_PCLK_RISING       2
#define JZ_CAMERA_VSYNC_HIGH        4
#define CAMERA_GSENSOR_VCC "vcc_gsensor"

//static int debug = 3;
//module_param(debug, int, S_IRUGO);
#define debug 3
#define dprintk(level, fmt, arg...)                                     \
	do {                                                            \
		if (debug >= level)                                     \
		printk("jz-camera: " fmt, ## arg);  \
	} while (0)

/* define the maximum number of camera sensor that attach to cim controller */
#define MAX_SOC_CAM_NUM         	1
struct camera_sensor_priv_data {
	unsigned int gpio_rst;
	unsigned int gpio_power;
	unsigned int gpio_en;
};

struct jz_camera_pdata {
	unsigned long mclk_10khz;
	unsigned long flags;
	struct camera_sensor_priv_data cam_sensor_pdata[MAX_SOC_CAM_NUM];
};

/*
 * CIM registers
 */
#define CIM_CFG			(0x00)
#define CIM_CTRL		(0x04)
#define CIM_STATE		(0x08)
#define CIM_IID			(0x0c)
#define CIM_DA			(0x20)
#define CIM_FA			(0x24)
#define CIM_FID			(0x28)
#define CIM_CMD			(0x2c)
#define CIM_SIZE		(0x30)
#define CIM_OFFSET		(0x34)
#define CIM_CTRL2		(0x50)
#define CIM_FS			(0x54)
#define CIM_IMR			(0x58)

/*CIM Configuration Register (CIMCFG)*/
#define CIM_CFG_VSP_HIGH		(1 << 14) 			/* VSYNC Polarity: 1-falling edge active */
#define CIM_CFG_HSP_HIGH		(1 << 13) 			/* HSYNC Polarity: 1-falling edge active */
#define CIM_CFG_PCP_HIGH		(1 << 12) 			/* PCLK working edge: 1-falling */

#define CIM_CFG_DMA_BURST_TYPE		10
#define CIM_CFG_DMA_BURST_INCR8		(0 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR16	(1 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR32	(2 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR64	(3 << CIM_CFG_DMA_BURST_TYPE)

#define CIM_CFG_PACK			4
#define CIM_CFG_PACK_VY1UY0		(0 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y0VY1U		(1 << CIM_CFG_PACK)
#define CIM_CFG_PACK_UY0VY1		(2 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y1UY0V		(3 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y0UY1V		(4 << CIM_CFG_PACK)
#define CIM_CFG_PACK_UY1VY0		(5 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y1VY0U		(6 << CIM_CFG_PACK)
#define CIM_CFG_PACK_VY0UY1		(7 << CIM_CFG_PACK)

#define CIM_CFG_BS0             16
#define CIM_CFG_BS0_2_OBYT0		(0 << CIM_CFG_BS0)
#define CIM_CFG_BS1_2_OBYT0     (1 << CIM_CFG_BS0)
#define CIM_CFG_BS2_2_OBYT0     (2 << CIM_CFG_BS0)
#define CIM_CFG_BS3_2_OBYT0     (3 << CIM_CFG_BS0)

#define CIM_CFG_BS1             18
#define CIM_CFG_BS0_2_OBYT1		(0 << CIM_CFG_BS1)
#define CIM_CFG_BS1_2_OBYT1     (1 << CIM_CFG_BS1)
#define CIM_CFG_BS2_2_OBYT1     (2 << CIM_CFG_BS1)
#define CIM_CFG_BS3_2_OBYT1     (3 << CIM_CFG_BS1)

#define CIM_CFG_BS2             20
#define CIM_CFG_BS0_2_OBYT2		(0 << CIM_CFG_BS2)
#define CIM_CFG_BS1_2_OBYT2     (1 << CIM_CFG_BS2)
#define CIM_CFG_BS2_2_OBYT2     (2 << CIM_CFG_BS2)
#define CIM_CFG_BS3_2_OBYT2     (3 << CIM_CFG_BS2)

#define CIM_CFG_BS3             22
#define CIM_CFG_BS0_2_OBYT3		(0 << CIM_CFG_BS3)
#define CIM_CFG_BS1_2_OBYT3     (1 << CIM_CFG_BS3)
#define CIM_CFG_BS2_2_OBYT3     (2 << CIM_CFG_BS3)
#define CIM_CFG_BS3_2_OBYT3     (3 << CIM_CFG_BS3)

#define CIM_CFG_DSM				0
#define CIM_CFG_DSM_CPM			(0 << CIM_CFG_DSM)		/* CCIR656 Progressive Mode */
#define CIM_CFG_DSM_CIM			(1 << CIM_CFG_DSM)		/* CCIR656 Interlace Mode */
#define CIM_CFG_DSM_GCM			(2 << CIM_CFG_DSM)		/* Gated Clock Mode */

/* CIM State Register  (CIM_STATE) */
#define CIM_STATE_DMA_EEOF	(1 << 11)				/* DMA Line EEOf irq */
#define CIM_STATE_DMA_STOP	(1 << 10)				/* DMA stop irq */
#define CIM_STATE_DMA_EOF	(1 << 9)				/* DMA end irq */
#define CIM_STATE_DMA_SOF	(1 << 8)				/* DMA start irq */
#define CIM_STATE_SIZE_ERR	(1 << 3)				/* Frame size check error */
#define CIM_STATE_RXF_OF	(1 << 2)				/* RXFIFO over flow irq */
#define CIM_STATE_RXF_EMPTY	(1 << 1)				/* RXFIFO empty irq */
#define CIM_STATE_STP_ACK	(1 << 0)				/* CIM disabled status */
#define CIM_STATE_RXOF_STOP_EOF	(CIM_STATE_RXF_OF | CIM_STATE_DMA_STOP | CIM_STATE_DMA_EOF)
/* CIM DMA Command Register (CIM_CMD) */
#define CIM_CMD_SOFINT		(1 << 31)				/* enable DMA start irq */
#define CIM_CMD_EOFINT		(1 << 30)				/* enable DMA end irq */
#define CIM_CMD_EEOFINT		(1 << 29)				/* enable DMA EEOF irq */
#define CIM_CMD_STOP		(1 << 28)				/* enable DMA stop irq */
#define CIM_CMD_OFRCV		(1 << 27)

/*CIM Control Register (CIMCR)*/
#define CIM_CTRL_FRC_BIT	16
#define CIM_CTRL_FRC_1		(0x0 << CIM_CTRL_FRC_BIT) 		/* Sample every n+1 frame */
#define CIM_CTRL_FRC_10		(0x9 << CIM_CTRL_FRC_BIT)

#define CIM_CTRL_DMA_SYNC	(1 << 7)        		/*when change DA, do frame sync */
#define CIM_CTRL_STP_REQ	(1 << 4) 				/*request to stop */
#define CIM_CTRL_CIM_RST	(1 << 3)
#define CIM_CTRL_DMA_EN		(1 << 2) 				/* Enable DMA */
#define CIM_CTRL_RXF_RST	(1 << 1) 				/* RxFIFO reset */
#define CIM_CTRL_ENA		(1 << 0) 				/* Enable CIM */

/* CIM Control Register 2 (CIMCR2) */
#define CIM_CTRL2_FSC		(1 << 23)				/* enable frame size check */
#define CIM_CTRL2_ARIF		(1 << 22)				/* enable auto-recovery for incomplete frame */
#define CIM_CTRL2_OPG_BIT	4					/* option priority configuration */
#define CIM_CTRL2_OPG_MASK	(0x3 << CIM_CTRL2_OPG_BIT)
#define CIM_CTRL2_OPE		(1 << 2)				/* optional priority mode enable */
#define CIM_CTRL2_APM		(1 << 0)				/* auto priority mode enable*/

/*CIM Interrupt Mask Register (CIMIMR)*/
#define CIM_IMR_STPM		(1<<10)
#define CIM_IMR_EOFM		(1<<9)
#define CIM_IMR_SOFM		(1<<8)
#define CIM_IMR_FSEM		(1<<3)
#define CIM_IMR_RFIFO_OFM	(1<<2)
#define CIM_IMR_STPM_1		(1<<0)

/* CIM Frame Size Register (CIM_FS) */
#define CIM_FS_FVS_BIT		16					/* vertical size of the frame */
#define CIM_FS_FVS_MASK		(0x1fff << CIM_FS_FVS_BIT)
#define CIM_FS_BPP_BIT		14					/* bytes per pixel */
#define CIM_FS_BPP_MASK		(0x3 << CIM_FS_BPP_BIT)
#define CIM_FS_FHS_BIT		0       				/* horizontal size of the frame */
#define CIM_FS_FHS_MASK		(0x1fff << CIM_FS_FHS_BIT)

#define CIM_BUS_FLAGS		(SOCAM_MASTER | SOCAM_VSYNC_ACTIVE_HIGH | \
				SOCAM_VSYNC_ACTIVE_LOW | SOCAM_HSYNC_ACTIVE_HIGH | \
				SOCAM_HSYNC_ACTIVE_LOW | SOCAM_PCLK_SAMPLE_RISING | \
				SOCAM_PCLK_SAMPLE_FALLING | SOCAM_DATAWIDTH_8)

#define VERSION_CODE		KERNEL_VERSION(0, 0, 1)
#define DRIVER_NAME		"jz-cim"
#define MAX_VIDEO_MEM		16        /* Video memory limit in megabytes */

/*
 * Structures
 */
struct jz_camera_dma_desc {
	dma_addr_t next;
	unsigned int id;
	unsigned int buf;
	unsigned int cmd;
	/* only used when SEP = 1 */
	unsigned int cb_frame;
	unsigned int cb_len;
	unsigned int cr_frame;
	unsigned int cr_len;
} __attribute__ ((aligned (32)));

/* buffer for one video frame */
struct jz_buffer {
	/* common v4l buffer stuff -- must be first */
	struct	videobuf_buffer vb;
	enum	v4l2_mbus_pixelcode code;
	int	inwork;
};

struct jz_camera_dev {
	struct soc_camera_host soc_host;
	struct soc_camera_device *icd[MAX_SOC_CAM_NUM];
	struct jz_camera_pdata *pdata;

	//struct jz_buffer *active;
	struct resource *res;
	struct clk *clk;
	struct clk *mclk;

	//struct list_head capture;
	void __iomem *base;

	unsigned int irq;
	unsigned long mclk_freq;
	spinlock_t lock;

	unsigned int buf_cnt;
	struct jz_camera_dma_desc *dma_desc;
	void *desc_vaddr;
	int is_first_start;
	//int stop_flag;
	int poll_flag;
	//struct	videobuf_buffer *vb_address;
	struct  videobuf_buffer *vb_address[20];
	int vb_address_flag;
	struct jz_camera_dma_desc *dma_desc_head;
	struct jz_camera_dma_desc *dma_desc_tail;
	unsigned int is_tlb_enabled;
};

#endif

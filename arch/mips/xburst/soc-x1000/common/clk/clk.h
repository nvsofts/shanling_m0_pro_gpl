#ifndef _CLK_H_
#define _CLK_H_
#include <linux/init.h>
#include <linux/kernel.h>
struct clk;
struct cpm_pwc;
struct clk_ops {
	int (*enable)(struct clk *,int);
	struct clk* (*get_parent)(struct clk *);
	int (*set_parent)(struct clk *,struct clk *);
	unsigned long (*get_rate)(struct clk *);
	int (*set_rate)(struct clk *,unsigned long);
	int (*set_round_rate)(struct clk *,unsigned long);
};
struct clk {
	const char *name;
	unsigned long rate;
	struct clk *parent;
	unsigned long flags;
#define CLK_FLG_NOALLOC	BIT(0)
#define CLK_FLG_ENABLE	BIT(1)
#define CLK_GATE_BIT(flg)	((flg) >> 24)
#define CLK_FLG_GATE	BIT(2)
#define CLK_CPCCR_NO(flg)	(((flg) >> 24) & 0xff)
#define CLK_FLG_CPCCR	BIT(3)
#define CLK_CGU_NO(flg) 	(((flg) >> 24) & 0xff)
#define CLK_FLG_CGU	BIT(4)
#define CLK_PLL_NO(flg) 	(((flg) >> 24) & 0xff)
#define CLK_FLG_PLL	BIT(5)
#define CLK_CGU_AUDIO_NO(flg) 	(((flg) >> 24) & 0xff)
#define CLK_FLG_CGU_AUDIO	BIT(6)
#define CLK_WDT_NO(flg) 	(((flg) >> 24) & 0xff)
#define CLK_FLG_WDT	BIT(9)
#define CLK_PARENT(flg) 	(((flg) >> 16) & 0xff)
#define CLK_RELATIVE(flg) 	(((flg) >> 16) & 0xff)
#define CLK_FLG_PARENT	BIT(7)
#define CLK_FLG_RELATIVE BIT(8)
#define CLK_SOFTCLK_BIT(flg)	((flg) >> 24)
#define CLK_FLG_SOFTCLK BIT(10)
	struct clk_ops *ops;
	atomic_t count;
	int init_state;
	struct clk *source;
	struct clk *child;
	unsigned int CLK_ID;
};

enum {
	CLK_ID_EXT     = 0,
	CLK_ID_EXT0,
#define CLK_NAME_EXT0		"ext0"
	CLK_ID_EXT1,
#define CLK_NAME_EXT1		"ext1"
	CLK_ID_OTGPHY,
#define CLK_NAME_OTGPHY	        "otg_phy"

	CLK_ID_PLL,
	CLK_ID_APLL,
#define CLK_NAME_APLL		"apll"
	CLK_ID_MPLL,
#define CLK_NAME_MPLL		"mpll"
	CLK_ID_SCLKA,
#define CLK_NAME_SCLKA		"sclka"
	/**********************************************************************************/
	CLK_ID_CPPCR,
	CLK_ID_CCLK,
#define CLK_NAME_CCLK		"cclk"
	CLK_ID_L2CLK,
#define CLK_NAME_L2CLK		"l2clk"
	CLK_ID_H0CLK,
#define CLK_NAME_H0CLK		"h0clk"
	CLK_ID_H2CLK,
#define CLK_NAME_H2CLK		"h2clk"
	CLK_ID_PCLK,
#define CLK_NAME_PCLK		"pclk"
	CLK_ID_MSC,
#define CLK_NAME_MSC		"msc"
	/**********************************************************************************/
/**********************************************************************************/
	CLK_ID_CGU,
	CLK_ID_CGU_PCM1,
#define CLK_NAME_CGU_PCM1	"cgu_pcm1"
	CLK_ID_CGU_PCM,
#define CLK_NAME_CGU_PCM	"cgu_pcm"
	CLK_ID_CGU_CIM,
#define CLK_NAME_CGU_CIM	"cgu_cim"
	CLK_ID_CGU_SFC,
#define CLK_NAME_CGU_SFC	"cgu_sfc"
	CLK_ID_CGU_MSC_MUX,
#define CLK_NAME_CGU_MSC_MUX	"cgu_msc_mux"
	CLK_ID_CGU_USB,
#define CLK_NAME_CGU_USB	"cgu_usb"
	CLK_ID_CGU_MSC1,
#define CLK_NAME_CGU_MSC1	"cgu_msc1"
	CLK_ID_CGU_MSC0,
#define CLK_NAME_CGU_MSC0	"cgu_msc0"
	CLK_ID_CGU_LCD,
#define CLK_NAME_CGU_LCD	"cgu_lcd"
	CLK_ID_CGU_I2S1,
#define CLK_NAME_CGU_I2S1	"cgu_i2s1"
	CLK_ID_CGU_I2S,
#define CLK_NAME_CGU_I2S	"cgu_i2s"
	CLK_ID_CGU_MACPHY,
#define CLK_NAME_CGU_MACPHY	"cgu_macphy"
	CLK_ID_CGU_DDR,
#define CLK_NAME_CGU_DDR	"cgu_ddr"

/**********************************************************************************/
	CLK_ID_DEVICES,
	CLK_ID_DDR,
#define CLK_NAME_DDR		"ddr"
	CLK_ID_CPU,
#define CLK_NAME_CPU		"cpu"
	CLK_ID_AHB0,
#define CLK_NAME_AHB0		"ahb0"
	CLK_ID_APB0,
#define CLK_NAME_APB0		"apb0"
	CLK_ID_RTC,
#define CLK_NAME_RTC		"rtc"
	CLK_ID_PCM,
#define CLK_NAME_PCM		"pcm"
	CLK_ID_MAC,
#define CLK_NAME_MAC		"mac"
	CLK_ID_AES,
#define CLK_NAME_AES		"aes"
	CLK_ID_LCD,
#define CLK_NAME_LCD		"lcd"
	CLK_ID_CIM,
#define CLK_NAME_CIM		"cim"
	CLK_ID_PDMA,
#define CLK_NAME_PDMA		"pdma"
	CLK_ID_SYS_OST,
#define CLK_NAME_SYS_OST	"sys_ost"
	CLK_ID_SSI,
#define CLK_NAME_SSI		"ssi0"
	CLK_ID_TCU,
#define CLK_NAME_TCU		"tcu"
	CLK_ID_DMIC,
#define CLK_NAME_DMIC		"dmic"
	CLK_ID_UART2,
#define CLK_NAME_UART2		"uart2"
	CLK_ID_UART1,
#define CLK_NAME_UART1		"uart1"
	CLK_ID_UART0,
#define CLK_NAME_UART0		"uart0"
	CLK_ID_SADC,
#define CLK_NAME_SADC		"sadc"
	CLK_ID_VPU,
#define CLK_NAME_VPU		"vpu"
	CLK_ID_AIC,
#define CLK_NAME_AIC		"aic"
	CLK_ID_I2C3,
#define CLK_NAME_I2C3		"i2c3"
	CLK_ID_I2C2,
#define CLK_NAME_I2C2		"i2c2"
	CLK_ID_I2C1,
#define CLK_NAME_I2C1		"i2c1"
	CLK_ID_I2C0,
#define CLK_NAME_I2C0		"i2c0"
	CLK_ID_SCC,
#define CLK_NAME_SCC		"scc"
	CLK_ID_MSC1,
#define CLK_NAME_MSC1		"msc1"
	CLK_ID_MSC0,
#define CLK_NAME_MSC0		"msc0"
	CLK_ID_OTG,
#define CLK_NAME_OTG		"otg1"
	CLK_ID_SFC,
#define CLK_NAME_SFC		"sfc"
	CLK_ID_EFUSE,
#define CLK_NAME_EFUSE		"efuse"
	CLK_ID_NEMC,
#define CLK_NAME_NEMC		"nemc"
	CLK_ID_WDT,
#define CLK_NAME_WDT		"wdt"

	CLK_ID_STOP,
	CLK_ID_INVALID,

/**********************************************************************************/
	CLK_ID_SOFTCLK,
	CLK_ID_DMIC_ENABLE,
#define CLK_NAME_DMIC_ENABLE "dmic_enable"
	CLK_ID_I2S_ENABLE,
#define CLK_NAME_I2S_ENABLE "i2s_enable"
	CLK_ID_AEC_ENABLE,
#define CLK_NAME_AEC_ENABLE "aec_enable"

};


enum {
	CGU_PCM1,CGU_CIM,CGU_SFC,
	CGU_USB,CGU_MSC1,CGU_MSC0,CGU_LCD,
	CGU_MACPHY,CGU_DDR,
	CGU_MSC_MUX
};

enum {
	CDIV = 0,L2CDIV,H0DIV,H2DIV,PDIV,SCLKA,
};

enum {
	CGU_AUDIO_I2S,CGU_AUDIO_I2S1,CGU_AUDIO_PCM,CGU_AUDIO_PCM1
};

#define I2S_PRI_DIV 0xb0020030
//#define PCM_PRI_DIV 0xb0030014
#define PCM_PRI_DIV 	0xb0071014
#define CPM_APLL_CTRL	0xb0000010
#define CPM_MPLL_CTRL	0xb0000014
struct  freq_udelay_jiffy {
	unsigned int  max_num;
	unsigned int  cpufreq;
	unsigned int  udelay_val;
	unsigned int  loops_per_jiffy;
};


int get_clk_sources_size(void);
struct clk *get_clk_from_id(int clk_id);
int get_clk_id(struct clk *clk);
typedef int (*DUMP_CALLBACK)(char *, const char *, ...);
int dump_out_clk(char *str,DUMP_CALLBACK dump_callback);
void dump_clk(void);

void __init init_cgu_clk(struct clk *clk);
void __init init_cgu_audio_clk(struct clk *clk);
void __init init_cpccr_clk(struct clk *clk);
void __init init_ext_pll(struct clk *clk);
void __init init_gate_clk(struct clk *clk);
void __init init_wdt_clk(struct clk *clk);
void __init init_softclk_clk(struct clk *clk);
#endif /* _CLK_H_ */

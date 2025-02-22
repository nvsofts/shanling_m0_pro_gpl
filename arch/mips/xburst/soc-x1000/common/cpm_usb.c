#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jz_dwc.h>
#include <soc/base.h>
#include <soc/extal.h>
#include <soc/cpm.h>

#define USBRDT_VBFIL_LD_EN		25
#define USBRDT_IDDIG_EN			24
#define USBRDT_IDDIG_REG		23
#define USBPCR_TXPREEMPHTUNE		6
#define USBPCR_POR			22
#define USBPCR_USB_MODE			31
#define USBPCR_COMMONONN		25
#define USBPCR_VBUSVLDEXT		24
#define USBPCR_VBUSVLDEXTSEL		23
#define USBPCR_OTG_DISABLE		20
#define USBPCR_SIDDQ			21
#define USBPCR_IDPULLUP_MASK		28
#define OPCR_SPENDN0			7
#define USBPCR1_USB_SEL			28
#define USBPCR1_WORD_IF0		19
#define USBPCR1_WORD_IF1		18
#define SRBC_USB_SR			12


void jz_otg_ctr_reset(void)
{
	cpm_set_bit(SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	cpm_clear_bit(SRBC_USB_SR, CPM_SRBC);
}

EXPORT_SYMBOL(jz_otg_ctr_reset);

void jz_otg_phy_init(otg_mode_t mode)
{
	unsigned int ref_clk_div = CONFIG_EXTAL_CLOCK / 24;
	unsigned int usbpcr1, usbrdt;

	/* select dwc otg */
	cpm_set_bit(USBPCR1_USB_SEL, CPM_USBPCR1);

	/* select utmi data bus width of port0 to 16bit/30M */
	cpm_set_bit(USBPCR1_WORD_IF0, CPM_USBPCR1);

	usbpcr1 = cpm_inl(CPM_USBPCR1);
	usbpcr1 &= ~(0x3 << 24 | 1 << 30);
	usbpcr1 |= (ref_clk_div << 24);
	cpm_outl(usbpcr1, CPM_USBPCR1);

	/*unsuspend*/
	cpm_set_bit(7, CPM_OPCR);
	udelay(45);
	cpm_clear_bit(USBPCR_SIDDQ, CPM_USBPCR);

	/* fil */
	cpm_outl(0, CPM_USBVBFIL);

	/* rdt */
	usbrdt = cpm_inl(CPM_USBRDT);
	usbrdt &= ~(USBRDT_VBFIL_LD_EN | ((1 << 23) - 1));
	usbrdt |= 0x96;
	cpm_outl(usbrdt, CPM_USBRDT);

	/* rdt - filload_en */
	cpm_set_bit(USBRDT_VBFIL_LD_EN, CPM_USBRDT);

	/* TXRISETUNE & TXVREFTUNE. */
	//cpm_outl(0x3f, CPM_USBPCR);
	//cpm_outl(0x35, CPM_USBPCR);

	/* enable tx pre-emphasis */
	//cpm_set_bit(USBPCR_TXPREEMPHTUNE, CPM_USBPCR);

	/* OTGTUNE adjust */
	//cpm_outl(7 << 14, CPM_USBPCR);

	cpm_outl(0x83803857, CPM_USBPCR);

	if (mode == DEVICE_ONLY) {
		pr_info("DWC IN DEVICE ONLY MODE\n");
		cpm_clear_bit(USBPCR_USB_MODE, CPM_USBPCR);
		cpm_clear_bit(USBPCR_OTG_DISABLE, CPM_USBPCR);
		cpm_clear_bit(USBPCR_SIDDQ, CPM_USBPCR);
		cpm_set_bit(USBPCR_COMMONONN, CPM_USBPCR);
	} else {
		unsigned int tmp;
		pr_info("DWC IN OTG MODE\n");
		tmp = cpm_inl(CPM_USBPCR);
		tmp |= 1 << USBPCR_USB_MODE | 1 << USBPCR_COMMONONN;
		tmp &= ~(1 << USBPCR_OTG_DISABLE | 1 << USBPCR_SIDDQ |
				0x03 << USBPCR_IDPULLUP_MASK | 1 << USBPCR_VBUSVLDEXT |
				1 << USBPCR_VBUSVLDEXTSEL);
		cpm_outl(tmp, CPM_USBPCR);
	}

	cpm_set_bit(USBPCR_POR, CPM_USBPCR);
	mdelay(1);
	cpm_clear_bit(USBPCR_POR, CPM_USBPCR);
	mdelay(1);
}
EXPORT_SYMBOL(jz_otg_phy_init);

int jz_otg_phy_is_suspend(void)
{
	return (!(cpm_test_bit(7, CPM_OPCR)));
}
EXPORT_SYMBOL(jz_otg_phy_is_suspend);

static int sft_id_set = false;
void jz_otg_sft_id(int level)
{
	if (level) {
		cpm_set_bit(USBRDT_IDDIG_REG, CPM_USBRDT);
	} else {
		cpm_clear_bit(USBRDT_IDDIG_REG, CPM_USBRDT);
	}
	cpm_set_bit(USBRDT_IDDIG_EN, CPM_USBRDT);
	if (!jz_otg_phy_is_suspend())
		mdelay(150);
	else
		sft_id_set = true;
}
EXPORT_SYMBOL(jz_otg_sft_id);

void jz_otg_sft_id_off(void)
{
	cpm_clear_bit(USBRDT_IDDIG_EN, CPM_USBRDT);
	if (!jz_otg_phy_is_suspend())
		mdelay(150);
	else
		sft_id_set = true;
}
EXPORT_SYMBOL(jz_otg_sft_id_off);

void jz_otg_phy_suspend(int suspend)
{
	if (!suspend && jz_otg_phy_is_suspend()) {
		printk("EN PHY\n");
		cpm_set_bit(7, CPM_OPCR);
		if (sft_id_set == true)
			mdelay(150);	/*2d6c0 phy clocks*/
		sft_id_set = false;
		udelay(45);
	} else if (suspend && !jz_otg_phy_is_suspend()) {
		printk("DIS PHY\n");
		cpm_clear_bit(7, CPM_OPCR);
		udelay(5);
	}
}
EXPORT_SYMBOL(jz_otg_phy_suspend);

void jz_otg_phy_powerdown(void)
{
	cpm_set_bit(USBPCR_OTG_DISABLE,CPM_USBPCR);
	cpm_set_bit(USBPCR_SIDDQ ,CPM_USBPCR);
}
EXPORT_SYMBOL(jz_otg_phy_powerdown);

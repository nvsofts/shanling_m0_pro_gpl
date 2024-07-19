/*
 * AXS touchscreen driver.
 *
 * Copyright (c) 2020-2021 AiXieSheng Technology. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "axs_core.h"
#define AXS_APP_BIN_VER_OFFSET          0x5035
#define AXS_MAX_FW_LEN                  (128 * 1024)
#define AXS_DOWNLOAD_APP_FILE           "firmware/firmware_app.i"
#define AXS_DOWNLOAD_READ_CHECK         0

struct upgrade_func
{
    u32 fwveroff;
    bool (*upgrade)(u8 *, u32);
};

struct axs_upgrade
{
    //struct axs_ts_data *ts_data;
    struct upgrade_func *func;
    //struct upgrade_setting_nf *setting_nf;
    //int module_id;
    u8 *fw;
    u32 fw_length;
};

static struct axs_upgrade *fwupgrade=NULL;


#if AXS_DOWNLOAD_APP_EN
static const u8 fw_file[] =
{
#include AXS_DOWNLOAD_APP_FILE
};

static bool axs_download_get_fw_file(struct axs_upgrade *upg)
{
    AXS_DEBUG("get upgrade fw file");
    if (!upg)
    {
        AXS_ERROR("upg is null");
        return false;
    }


    upg->fw = (u8 *)fw_file;
    upg->fw_length = sizeof(fw_file);

    AXS_DEBUG("upgrade fw file len:%d", upg->fw_length);
    if ((upg->fw_length > AXS_MAX_FW_LEN)||(upg->fw_length < 1024))
    {
        AXS_ERROR("fw file len(%d) fail!", upg->fw_length);
        return false;
    }
    return true;
}
#endif


bool selectPram(void)
{
    int ret = 0;
    u8 write_buf[] = {0xa5, 0x5a, 0xb5, 0xab, 0x01};
    axs_reset_level(0);
    msleep(1);
    ret = axs_write(write_buf, sizeof(write_buf));
    if (ret < 0)
    {
        AXS_ERROR("upgrade firmware selectPram fail");
        axs_reset_level(1);
        return false;
    }
    axs_reset_level(1);
    msleep(15);
    return true;
}


bool axs_download_app(struct axs_upgrade *upg)
{
    bool ret = false;
    int upgrade_count = 0;

    AXS_DEBUG("axs_download_app function");
    if ((NULL == upg) || (NULL == upg->func))
    {
        AXS_ERROR("upg/upg->func is null");
        return false;
    }

    do
    {
        upgrade_count++;
        AXS_DEBUG("download fw app (times:%d) begin", upgrade_count);
        if (!selectPram())
        {
            AXS_ERROR("selectPram fail");
            return false;
        }
        if (upg->func->upgrade)
        {
            ret = upg->func->upgrade(upg->fw, upg->fw_length);
            if (!ret)
            {
                AXS_DEBUG("download fw app (times:%d) fail", upgrade_count);
            }
            else
            {
                AXS_DEBUG("fw app download (times:%d) success!",upgrade_count);
                break;
            }
        }
        else
        {
            AXS_ERROR("upgrade func/upgrade is null, return immediately");
            ret = false;
            break;
        }
    }
    while (upgrade_count < 2);

    return ret;
}


#if AXS_DOWNLOAD_APP_EN
static void axs_download_work(struct work_struct *work)
{
    struct axs_upgrade *upg = fwupgrade;
    AXS_DEBUG("axs_download_work begin");
    if (!upg || !g_axs_data)
    {
        AXS_ERROR("upg/g_axs_data is null");
        return ;
    }

    g_axs_data->fw_loading = 1;
    //axs_irq_disable();

    /* get fw */
    if (!axs_download_get_fw_file(upg))
    {
        AXS_ERROR("get file fail, can't upgrade");
    }
    else
    {
        /* run download firmware app*/
        axs_download_app(upg);
    }

    //axs_irq_enable();
    g_axs_data->fw_loading = 0;
    AXS_DEBUG("axs_download_work end");
}
#endif


bool axs_download_write_pram(u8 *fw_buf, u32 fw_len)
{
    int ret = 0;
    const int packet_len = 256;
    int currentNum = 1;
    int currentSize = 0;
    u32 addr = 0;
    u8 write_buf[512] = { 0xab, 0xb5, 0xa5, 0x5a };
    u8 *pAddr = (u8*)(&addr);
    int totalPackets = fw_len / packet_len;
    if (fw_len % packet_len)
        ++totalPackets;
    AXS_DEBUG("axs_download_write_pram begin,totalPackets:%x",totalPackets);
    while (currentNum <= totalPackets)
    {
        currentSize = (currentNum == totalPackets) ? (fw_len - (currentNum - 1) * packet_len) : packet_len;
        addr = (currentNum - 1) * packet_len;
        write_buf[4] = (currentSize + 3)>>8; // write  3 more byte for offset
        write_buf[5] = (currentSize + 3)&0xff;

        write_buf[6] = 0x00;
        write_buf[7] = 0x00;
        write_buf[8] =  *(pAddr + 2);
        write_buf[9] = *(pAddr + 1);
        write_buf[10] = *(pAddr + 0);

        memcpy(&write_buf[11], fw_buf + addr, currentSize);
        //AXS_DEBUG("write sector num:%d",currentNum);
        ret = axs_write(write_buf, currentSize+11);
        if (ret < 0)
        {
            AXS_ERROR("step2 fail, sector num:%d",currentNum);
            return false;
        }
        //msleep(1);
        ++currentNum;
    }

    AXS_DEBUG("axs_download_write_pram end");
    return true;
}

bool axs_download_finish(void)
{
    //send finish flag
    int ret = 0;
    u8 write_buf[11] = { 0xab, 0xb5, 0xa5, 0x5a };
    write_buf[4] = 0x00;
    write_buf[5] = 0x00;
    write_buf[6] = 0x00;
    write_buf[7] = 0x00;
    ret = axs_write(write_buf, 8);
    if (ret<0)
    {
        AXS_ERROR("axs_download_read_pram, send finish flag fail");
        return false;
    }
    return true;
}

#if AXS_DOWNLOAD_READ_CHECK
bool axs_download_read_pram(u32 fw_len, u8*read_buf)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    const int packet_len = 256;
    int currentNum = 1;
    int currentSize = 0;
    u32 addr = 0;
    u8 write_buf[11] = { 0xab, 0xb5, 0xa5, 0x5a };
    u8 *pAddr = (u8*)(&addr);
    int totalPackets = fw_len / packet_len;
    if (fw_len % packet_len)
        ++totalPackets;
    AXS_DEBUG("function begin,fw_len:%x",fw_len);

    tmpbuf = kzalloc(packet_len+5, GFP_KERNEL); // alloc 256+5 byte buff
    if (!tmpbuf)
    {
        AXS_ERROR("axs_download_read_pram tmpbuf alloc fail");
        return false;
    }

    while (currentNum <= totalPackets)
    {
        currentSize = (currentNum == totalPackets) ? (fw_len - (currentNum - 1) * packet_len) : packet_len;
        addr = (currentNum - 1) * packet_len;

        write_buf[4] = 0x00;
        write_buf[5] = 0x03; // write 3 byte for  offset
#if AXS_BUS_SPI
        write_buf[6] = (currentSize + 5)>>8;
        write_buf[7] = (currentSize + 5)&0xff; // spi need to read 5 more byte for dummy
#else
        write_buf[6] = (currentSize)>>8;
        write_buf[7] = (currentSize)&0xff;
#endif
        write_buf[8] =  *(pAddr + 2);
        write_buf[9] = *(pAddr + 1);
        write_buf[10] = *(pAddr + 0);
        //AXS_DEBUG("read sector num:%d",currentNum);
#if AXS_BUS_SPI
        ret = axs_write_read_onecs(write_buf, 11, tmpbuf,currentSize+5); // spi need to read 5 more byte for dummy
#else
        ret = axs_write_read(write_buf, 11, tmpbuf,currentSize); // i2c need not read 5 more byte for dummy
#endif
        if (ret < 0)
        {
            AXS_ERROR("step2 fail, sector num:%d",currentNum);
            kfree_safe(tmpbuf);
            return false;
        }
#if AXS_BUS_SPI
        memcpy(read_buf + addr, &tmpbuf[5], currentSize); // spi skip 5 byte  dummy
#else
        memcpy(read_buf + addr, &tmpbuf[0], currentSize);
#endif
        //msleep(1);
        ++currentNum;
    }
#if 0
    //send finish flag begin
    write_buf[4] = 0x00;
    write_buf[5] = 0x00;
    write_buf[6] = 0x00;
    write_buf[7] = 0x00;
    ret = axs_write(write_buf, 8);
    if (ret<0)
    {
        AXS_ERROR("axs_download_read_pram, send finish flag fail");
    }
    //send finish flag end
#endif
    kfree_safe(tmpbuf);

    AXS_DEBUG("function end");
    return true;
}
#endif

static bool axs_Y15205_download(u8 *fw_buf, u32 fw_len)
{
#if AXS_DOWNLOAD_READ_CHECK
    u8 *tmpbuf = NULL;
#endif
    u8 ver = 0;

    AXS_DEBUG("function begin");
    if (NULL == fw_buf)
    {
        AXS_ERROR("fw buf is null");
        return false;
    }

    // write firmware
    if (!axs_download_write_pram(fw_buf, fw_len))
    {
        AXS_ERROR("axs_download_write_pram fail");
        return false;
    }
    msleep(20); // 100
#if AXS_DOWNLOAD_READ_CHECK
    // read firmware
    tmpbuf = vmalloc(fw_len+10);//, GFP_KERNEL);
    if (!tmpbuf)
    {
        AXS_ERROR("axs_Y15205_download tmpbuf alloc fail");
        return false;
    }

    if (!axs_download_read_pram(fw_len, tmpbuf))
    {
        AXS_ERROR("axs_download_read_pram fail");
        vfree_safe(tmpbuf);
        return false;
    }

    if ( memcmp(fw_buf,tmpbuf,fw_len))
    {
        AXS_ERROR("axs_download read app is not same with write app");
        vfree_safe(tmpbuf);
        return false;
    }
    vfree_safe(tmpbuf);
#endif
    if (!axs_download_finish())
    {
        return false;
    }

    msleep(20);
    if (!axs_fwupg_get_ver_in_tp(&ver))
    {
        AXS_ERROR("get fw ver in tp fail");
        return false;
    }

    AXS_DEBUG("new version:0x%02X",ver);

    AXS_DEBUG("axs_Y15205_download..ok");
    return true;
}

struct upgrade_func download_func_15205 =
{
    .fwveroff = AXS_APP_BIN_VER_OFFSET,
    .upgrade = axs_Y15205_download,
};


#if AXS_DOWNLOAD_APP_EN
bool axs_download_init(void)
{
    AXS_DEBUG("fw download begin");

    if (!g_axs_data || !g_axs_data->ts_workqueue)
    {
        AXS_ERROR("g_axs_data/workqueue is NULL, can't run upgrade function");
        return false;
    }

    if (NULL == fwupgrade)
    {
        fwupgrade = (struct axs_upgrade *)kzalloc(sizeof(*fwupgrade), GFP_KERNEL);
        if (NULL == fwupgrade)
        {
            AXS_ERROR("malloc memory for upgrade fail");
            return false;
        }
        fwupgrade->func = &download_func_15205;
    }

    //fwupgrade->ts_data = ts_data;
    INIT_WORK(&g_axs_data->fwupg_work, axs_download_work);
    queue_work(g_axs_data->ts_workqueue, &g_axs_data->fwupg_work);
    AXS_DEBUG("fw download end");
    return true;
}
#endif


#if AXS_DEBUG_SYSFS_EN
void axs_debug_download_app(char *fw_name)
{
    int ret = 0;
    struct axs_upgrade *upg;
    AXS_DEBUG("fw upgrade work function");

    if (!g_axs_data || !g_axs_data->ts_workqueue)
    {
        AXS_ERROR("g_axs_data/workqueue is NULL, can't run upgrade function");
        return;
    }

    if (NULL == fwupgrade)
    {
        fwupgrade = (struct axs_upgrade *)kzalloc(sizeof(*fwupgrade), GFP_KERNEL);
        if (NULL == fwupgrade)
        {
            AXS_ERROR("malloc memory for upgrade fail");
            return;
        }
        fwupgrade->func = &download_func_15205;
    }

    upg = fwupgrade;
    g_axs_data->fw_loading = 1;
    axs_irq_disable();

    upg->fw = NULL;
    ret = axs_read_file(fw_name, &(upg->fw));
    upg->fw_length = ret;
    AXS_DEBUG("fw bin file len:%x", upg->fw_length);

    if ((ret < 0) || (ret >AXS_MAX_FW_LEN)||(upg->fw_length < 1024))
    {
        AXS_ERROR("read fw bin file fail, len:%d", ret);
        goto err;
    }
    else
    {
        axs_download_app(upg);
    }

err:
    if (upg->fw)
    {
        vfree(upg->fw);
        upg->fw = NULL;
        upg->fw_length = 0;
    }
    axs_irq_enable();
    g_axs_data->fw_loading = 0;
}
#endif






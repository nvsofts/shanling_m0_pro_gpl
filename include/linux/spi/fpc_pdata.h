#ifndef __LINUX_SPI_FPC_FP_H
#define __LINUX_SPI_FPC_FP_H

struct fpc_platform_data {
    int int_pin;
    int reset_pin;
    int cs_pin;
    int encrypt_ic_rst_pin;
};

#endif

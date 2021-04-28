
/*
================================================================================
                            INLCUDE FILES
================================================================================
*/

#ifndef NOR_H_
#define NOR_H_

#include <stdio.h>
//#include <ti/pspiom/cslr/soc_OMAPL137.h>
#include <ti/pspiom/cslr/cslr_spi.h>
#include <ti/pspiom/cslr/cslr.h>
#include "OMAPL138_common.h"

/*
================================================================================
                         LOCAL FUNCTION PROTOTYPES
================================================================================
*/

#define NOR_BASE		(0x770000)
#define SEGMENT_LEN		(0x10000)
#define SEQ_SEGMENTS	(8)
#define CHUNK_SIZE		(256)


void nor_cfg ();

int nor_rd (const int adr, char *buf, int n);

int nor_wr (const int adr, char *buf, int n);

int nor_erase (const int adr);

//-------------Internal functions-------------------

int spi_flash_rd_status ( void );

int spi_wait_for_tx_unsafe(void);

int spi_wait_for_tx(void);

int spi_flash_wait_wr_complete ( void );

void spi_flash_wr_en ( void );

void spi_flash_wr_dis ( void );

char spi_send_byte_with_cs_start(const char byte);

char spi_send_byte_with_cs_nochange(const char byte);

char spi_send_byte_with_cs_finish(const char byte);

//-----------------------------------------------------

void spi_flash_test_buf_inc_fill (char *buf, int n);

void spi_flash_test_buf_ff_fill (char *buf, int n);

void spi_txbuf_prepare(int *data, int count);

//-----------------------------------------------------

interrupt void spi_isr(void);

#endif

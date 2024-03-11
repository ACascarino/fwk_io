// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <xs1.h>
#include <xclib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "xcore/port.h"
#include "xcore/clock.h"
#include "xcore/parallel.h"

#include "tdm.h"

port_t p_bclk = XS1_PORT_1A;
port_t p_fsync = XS1_PORT_1C;
port_t p_dout[] = {XS1_PORT_1D};
port_t p_din[] = {XS1_PORT_1H};
size_t const num_out = 1;
size_t const num_in = 1;

xclock_t bclk = XS1_CLKBLK_1;

port_t setup_strobe_port = XS1_PORT_1E;
port_t setup_data_port = XS1_PORT_16B;
port_t setup_resp_port = XS1_PORT_1F;

#ifndef TEST_FRAME_COUNT
#define TEST_FRAME_COUNT 5
#endif
#ifndef TEST_NUM_CH
#define TEST_NUM_CH 16
#endif

int32_t test_data[TEST_FRAME_COUNT][TEST_NUM_CH] = {{0}};
int32_t rx_data[TEST_FRAME_COUNT][TEST_NUM_CH] = {{0}};
volatile int32_t cnt = 0;

DECLARE_JOB(burn, (void));
void burn(void)
{
    for (;;)
        ;
}

static void send_data_to_tester(
    port_t setup_strobe_port,
    port_t setup_data_port,
    unsigned data)
{
    port_out(setup_data_port, data);
    asm volatile("syncr res[%0]" : : "r"(setup_data_port));
    port_out(setup_strobe_port, 1);
    port_out(setup_strobe_port, 0);
    asm volatile("syncr res[%0]" : : "r"(setup_data_port));
}

static void broadcast_settings(
    port_t setup_strobe_port,
    port_t setup_data_port)
{
    port_out(setup_strobe_port, 0);

    send_data_to_tester(setup_strobe_port, setup_data_port, TX_OFFSET);
}

static uint32_t request_response(
    port_t setup_strobe_port,
    port_t setup_resp_port)
{
    port_enable(setup_resp_port);
    port_out(setup_strobe_port, 1);
    port_out(setup_strobe_port, 0);
    uint32_t tmp = port_in(setup_resp_port);
    return tmp;
}

TDM_CALLBACK_ATTR
void tdm_process(void *app_data,
                 size_t num_out,
                 size_t num_in,
                 size_t num_chans,
                 size_t num_data_bits,
                 tdm_sample_buffer_t *receive_samples,
                 tdm_sample_buffer_t *send_samples)
{
    memcpy(send_samples->line[0].channel, test_data[cnt], num_chans * sizeof(int32_t));
    memcpy(rx_data[cnt], receive_samples->line[0].channel, num_chans * sizeof(int32_t));
}

TDM_CALLBACK_ATTR
void tdm_init(void *app_data, tdm_config_t *tdm_config)
{
    printf("tdm_init\n");
    (void)app_data;

    tdm_config->offset = TX_OFFSET;
    tdm_config->slave_bclk_polarity = TDM_SLAVE_SAMPLE_ON_BCLK_RISING;

    if (cnt > 0)
    {
        printf("Restart likely due to fsynch error at frame count: %ld\n", cnt);
        _Exit(1);
    }

    /* Initialize test data */
    for (int i = 1; i <= TEST_FRAME_COUNT; i++)
    {
        for (int j = 0; j < TEST_NUM_CH; j++)
        {
            /* bit rev and start with 1 to make it easier to see on the wire */
            test_data[i - 1][j] = j % 2 ? 0x89ABCDEF : 0x12345678;//bitrev((j << 24) | i);
        }
    }

    broadcast_settings(setup_strobe_port, setup_data_port);
}

TDM_CALLBACK_ATTR
tdm_restart_t tdm_restart_check(void *app_data)
{
    cnt++;

    if (cnt == TEST_FRAME_COUNT)
    {
        port_sync(p_dout[0]); // Wait for the port to empty so we get the whole frame before quitting
        _Exit(1);
    }

    return TDM_NO_RESTART;
}

int main(void)
{
    tdm_callback_group_t i_tdm = {
        .init = (tdm_init_t)tdm_init,
        .restart_check = (tdm_restart_check_t)tdm_restart_check,
        .process = (tdm_process_t)tdm_process,
        .app_data = NULL,
    };

    port_enable(setup_strobe_port);
    port_enable(setup_data_port);
    port_enable(setup_resp_port);
    port_enable(p_bclk);

    PAR_JOBS(
        PJOB(tdm_slave, (&i_tdm,
                         p_dout,
                         num_out,
                         p_din,
                         num_in,
                         16,
                         16,
                         p_bclk,
                         p_fsync,
                         bclk)),
        PJOB(burn, ()),
        PJOB(burn, ()),
        PJOB(burn, ()),
        PJOB(burn, ()),
        PJOB(burn, ()),
        PJOB(burn, ()),
        PJOB(burn, ()));

    return 0;
}

// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <xcore/assert.h>
#include <xcore/clock.h>
#include <xcore/interrupt.h>
#include <xcore/interrupt_wrappers.h>
#include <xcore/port.h>
#include <xcore/triggerable.h>

#include "tdm.h"

/**
 * Bits in the transfer register plus shift register.
 * This will cause the data-loading interrupt to be called when the shift
 *     register runs out of data.
 * Set to 32 to instead have it called when only the transfer register is empty;
 *     i.e. the shift register has just been loaded
 */
#ifndef TDM_PORT_BUFFER_BITS
#define TDM_PORT_BUFFER_BITS 32
#endif

/**
 * The interrupt routine takes time to complete.
 * Use this macro to increase the amount of headroom allowed for the routine.
 */
#ifndef TDM_INTERRUPT_OVERHEAD_BCLK_CYCLES
#define TDM_INTERRUPT_OVERHEAD_BCLK_CYCLES 3
#endif

#define MAX(x, y) ((x > y) ? x : y)

/**
 * OUT16(X, Y)
 *
 * input:  X = ABCD,       Y = EFGH
 * output: X is clobbered, Y = bitrev(CDGH)
 *
 */
#define OUT16(X, Y)          \
    asm(                     \
        "zip    %0, %1, 4\n" \
        "bitrev %1, %1"      \
        : "+r"(X), "+r"(Y))

/**
 * IN16(X, Y)
 *
 * input:  X = XXXXXXXX, Y = HGFEDCBA
 * output: X = XXXXABCD, Y = XXXXEGFH
 *
 */

#define IN16(X, Y)        \
    asm(                  \
        "bitrev %1, %1\n" \
        "unzip %0, %1, 4" \
        : "+r"(X), "+&r"(Y))

/**
 * OUT24(W, X, Y, Z, BLANK, C)
 *
 * input:  W = ABCD, X = EFGH, Y = IJKL, Z = MNOP, BLANK = ----, C = ----
 * output: W is unchanged,
 *         X = bitrev(GHJK),
 *         Y is clobbered,
 *         Z = bitrev(LNOP),
 *         BLANK is clobbered,
 *         C = bitrev(BCDF)
 *
 *         When outputting over TDM, send C, then Y, then W.
 */
#define OUT24(W, X, Y, Z, BLANK, C)                          \
    asm(                                                     \
        "linsert  %3, %0, %5, 24, 24\n"                      \
        "lextract %4, %3, %0, 16, 32\n"                      \
        "shli     %0, 16\n"                                  \
        "linsert  %0, %2, %1, 24, 24\n"                      \
        "bitrev   %5, %5\n"                                  \
        "bitrev   %1, %1\n"                                  \
        "bitrev   %4, %4\n"                                  \
        : "+&r"(X), "+&r"(Y), "r"(Z), "+&r"(BLANK), "+&r"(C) \
        : "r"(W))

typedef struct
{
    tdm_callback_group_t *const tdm_cbg;
    const port_t *p_dout;
    const size_t num_out;
    const port_t *p_din;
    const size_t num_in;
    const size_t num_chans;
    const size_t num_data_bits;
    const size_t frame_len;
    const port_t p_bclk;
    const port_t p_fsync;
    const xclock_t bclk;
    size_t num_bclk_cycles_offset;
} tdm_slave_loop_args_t;

typedef struct
{
    tdm_sample_buffer_t buffer[2];
} tdm_sample_double_buffer_t;

typedef struct
{
    tdm_slave_loop_args_t *args;
    tdm_sample_buffer_t *working_in_buffer;
    tdm_sample_buffer_t *working_out_buffer;
    port_timestamp_t fsync_time;
    bool first_time;
    uint8_t tx_fragment_no;
    uint8_t rx_fragment_no;
} tdm_interrupt_data_t;

static inline void commit_buffers(tdm_sample_double_buffer_t *in_buffer,
                                  tdm_sample_double_buffer_t *out_buffer,
                                  tdm_sample_buffer_t **safe_in_buf_addr,
                                  tdm_sample_buffer_t **safe_out_buf_addr,
                                  tdm_interrupt_data_t *interrupt_data)
{
    static uint8_t current_buf_no = 0;
    interrupt_data->working_in_buffer = &(in_buffer->buffer[current_buf_no]);
    interrupt_data->working_out_buffer = &(out_buffer->buffer[current_buf_no]);
    current_buf_no ^= 0b1; // flip to other of the double buffers
    *safe_in_buf_addr = &(in_buffer->buffer[current_buf_no]);
    *safe_out_buf_addr = &(out_buffer->buffer[current_buf_no]);
}

static inline void tdm_slave_init_ports(const port_t p_dout[],
                                        const size_t num_out,
                                        const port_t p_din[],
                                        const size_t num_in,
                                        const port_t p_bclk,
                                        const port_t p_fsync,
                                        const xclock_t bclk)
{
    clock_enable(bclk);
    port_reset(p_bclk);
    clock_set_source_port(bclk, p_bclk);
    port_set_clock(p_bclk, bclk);

    port_enable(p_fsync);
    port_set_clock(p_fsync, bclk);

    for (int output_line = 0; output_line < num_out; output_line++)
    {
        port_start_buffered(p_dout[output_line], 32);
        port_set_clock(p_dout[output_line], bclk);
        port_clear_buffer(p_dout[output_line]);
        port_out(p_dout[output_line], 0);
    }

    for (int input_line = 0; input_line < num_in; input_line++)
    {
        port_start_buffered(p_din[input_line], 32);
        port_set_clock(p_din[input_line], bclk);
        port_clear_buffer(p_din[input_line]);
    }
}

DEFINE_INTERRUPT_CALLBACK(tdm_isr_permitted, tdm_isr, tdm_interrupt_data)
{
    tdm_interrupt_data_t *data = tdm_interrupt_data;
    triggerable_disable_trigger(data->args->p_bclk);
    /* In a more general TDM case, this interrupt should schedule itself to
     * repeat periodically throughout the frame to manage _in and _out
     * operations. In this specific implementation, it is only called once per
     * frame to retrieve the first two 16b words recieved. */
    port_timestamp_t now = port_get_trigger_time(data->args->p_bclk);
    port_clear_trigger_time(data->args->p_bclk);
    /* In this interrupt we need to _in to retrieve the first 2 RX words */
    for (int in_line = 0; in_line < data->args->num_in; in_line++)
    {
        uint32_t d0 = 0;
        uint32_t d1 = port_in(data->args->p_din[in_line]);
        IN16(d0, d1);
        data->working_in_buffer->line[in_line].channel[0] = d0;
        data->working_in_buffer->line[in_line].channel[1] = d1;
        break;
    }
    /* Schedule self to trigger at this time next frame */
    port_set_trigger_time(data->args->p_bclk, now + data->args->frame_len);
    triggerable_enable_trigger(data->args->p_bclk);
}

/**
 * This part implies:
 *
 *  - 32b words - TDM 1 thru 16 supported
 *  - 24b words - TDM 4, 8, 12, 16 supported
 *  - 16b words - TDM 2, 4, 8, 10, 12, 14, 16 supported
 *  - 8b words  - TDM 4, 8, 12, 16 supported
 *
 * There may be other parts that narrow these down further
 *
 * Currently only implemented for 16b
 */
static inline void preload_data(tdm_interrupt_data_t *data)
{
    for (int out_line = 0; out_line < data->args->num_out; out_line++)
    {
        uint32_t out = data->working_out_buffer->line[out_line].channel[1];
        uint32_t tmp = data->working_out_buffer->line[out_line].channel[0];
        OUT16(tmp, out);
        port_out_at_time(data->args->p_dout[out_line],
                         (data->fsync_time +
                          data->args->frame_len +
                          data->args->num_bclk_cycles_offset),
                         out);
    }
}

static inline port_timestamp_t hold_for_frame_sync(port_t p_fsync)
{
    port_in_when_pinseq(p_fsync, PORT_UNBUFFERED, 1);
    return port_get_trigger_time(p_fsync);
}

DEFINE_INTERRUPT_PERMITTED(tdm_isr_permitted, void, tdm_main_loop,
                           tdm_slave_loop_args_t *args)
{
    void *app_data = args->tdm_cbg->app_data;

    tdm_config_t tdm_config;
    memset(&tdm_config, 0, sizeof(tdm_config_t));

    tdm_sample_double_buffer_t in_s;
    memset(&in_s, 0, sizeof(tdm_sample_double_buffer_t));

    tdm_sample_double_buffer_t out_s;
    memset(&out_s, 0, sizeof(tdm_sample_double_buffer_t));

    tdm_interrupt_data_t tdm_interrupt_data;
    memset(&tdm_interrupt_data, 0, sizeof(tdm_interrupt_data_t));
    tdm_interrupt_data.first_time = true;
    tdm_interrupt_data.args = args;
    tdm_interrupt_data.working_in_buffer = &in_s.buffer[1];
    tdm_interrupt_data.working_out_buffer = &out_s.buffer[1];

    tdm_sample_buffer_t *safe_in_buffer = &in_s.buffer[0];
    tdm_sample_buffer_t *safe_out_buffer = &out_s.buffer[0];

    const size_t port_buffer_bits = 64;
    const size_t ops_per_frame = MAX(args->frame_len / port_buffer_bits, 1);

    tdm_restart_t restart = TDM_NO_RESTART;

    while (restart != TDM_SHUTDOWN)
    {
        interrupt_mask_all();

        args->tdm_cbg->init(app_data, &tdm_config);

        if (tdm_config.slave_bclk_polarity == TDM_SLAVE_SAMPLE_ON_BCLK_FALLING)
        {
            port_set_invert(args->p_bclk);
        }
        else
        {
            port_set_no_invert(args->p_bclk);
        }

        args->num_bclk_cycles_offset = tdm_config.offset;

        if (args->num_out > 0)
        {
            args->tdm_cbg->process(app_data,
                                   args->num_out,
                                   args->num_in,
                                   args->num_chans,
                                   args->num_data_bits,
                                   safe_in_buffer,
                                   safe_out_buffer);
        }
        commit_buffers(&in_s, &out_s,
                       &safe_in_buffer, &safe_out_buffer,
                       &tdm_interrupt_data);

        port_clear_buffer(args->p_fsync);

        clock_start(args->bclk);
        tdm_interrupt_data.fsync_time = hold_for_frame_sync(args->p_fsync);

        triggerable_setup_interrupt_callback(args->p_bclk,
                                             &tdm_interrupt_data,
                                             INTERRUPT_CALLBACK(tdm_isr));
        port_set_trigger_time(args->p_bclk,
                              (tdm_interrupt_data.fsync_time +
                               args->frame_len +
                               args->num_bclk_cycles_offset +
                               TDM_PORT_BUFFER_BITS));

        triggerable_enable_trigger(args->p_bclk);

        interrupt_unmask_all();

        for (int input_line = 0; input_line < args->num_in; input_line++)
        {
            port_clear_buffer(args->p_din[input_line]);
            port_set_trigger_time(args->p_din[input_line],
                                  (tdm_interrupt_data.fsync_time +
                                   args->frame_len +
                                   args->num_bclk_cycles_offset +
                                   TDM_PORT_BUFFER_BITS));
        }
        for (int out_line = 0; out_line < args->num_out; out_line++)
        {
            port_clear_buffer(args->p_dout[out_line]);
        }

        preload_data(&tdm_interrupt_data);

        while (restart == TDM_NO_RESTART)
        {
            tdm_interrupt_data.fsync_time = hold_for_frame_sync(args->p_fsync);
            restart = args->tdm_cbg->restart_check(app_data);
            args->tdm_cbg->process(app_data,
                                   args->num_out,
                                   args->num_in,
                                   args->num_chans,
                                   args->num_data_bits,
                                   safe_in_buffer,
                                   safe_out_buffer);
            commit_buffers(&in_s, &out_s,
                           &safe_in_buffer, &safe_out_buffer,
                           &tdm_interrupt_data);
            preload_data(&tdm_interrupt_data);
        }
    }
}

void tdm_slave(tdm_callback_group_t *const tdm_cbg,
               const port_t p_dout[],
               const size_t num_out,
               const port_t p_din[],
               const size_t num_in,
               const size_t num_chans,
               const size_t num_data_bits,
               const port_t p_bclk,
               const port_t p_fsync,
               const xclock_t bclk)
{
    xassert(num_chans <= TDM_MAX_CHANS);
    xassert(num_in <= TDM_MAX_PORTS);
    xassert(num_out <= TDM_MAX_PORTS);

    tdm_slave_init_ports(p_dout, num_out, p_din, num_in, p_bclk, p_fsync, bclk);
    const size_t frame_len = num_chans * num_data_bits;

    tdm_slave_loop_args_t args = {tdm_cbg,
                                  (port_t *)p_dout,
                                  num_out,
                                  (port_t *)p_din,
                                  num_in,
                                  num_chans,
                                  num_data_bits,
                                  frame_len,
                                  p_bclk,
                                  p_fsync,
                                  bclk};

    INTERRUPT_PERMITTED(tdm_main_loop)
    (&args);
}
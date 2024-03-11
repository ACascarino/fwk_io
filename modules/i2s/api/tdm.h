// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stddef.h>
#include <stdint.h>

#include <xcore/clock.h>
#include <xcore/parallel.h>
#include <xcore/port.h>

#define TDM_MAX_CHANS 16
#define TDM_MAX_PORTS 4

/**
 * TDM slave bit clock polarity.
 *
 * TDM is positive, that is toggle data and fsync on falling
 * edge of bit clock and sample them on rising edge of bit clock. Some
 * masters have it the other way around.
 */
typedef enum
{
    /**
     * Toggle falling, sample rising (default if not set)
     */
    TDM_SLAVE_SAMPLE_ON_BCLK_RISING = 0,
    /**
     * Toggle rising, sample falling
     */
    TDM_SLAVE_SAMPLE_ON_BCLK_FALLING,
} tdm_slave_bclk_polarity_t;

/**
 * TDM sample buffer container
 * 
 * This contains exactly TDM_MAX_PORTS lines (corresponding to physical ports),
 * each with exactly TDM_MAX_CHANS int32_t elements. A very explicit 2D array.
 */
typedef struct
{
    struct
    {
        int32_t channel[TDM_MAX_CHANS];
    } line[TDM_MAX_PORTS];
} tdm_sample_buffer_t;

/**
 * TDM configuration structure.
 *
 * This structure describes the configuration of an TDM bus.
 */
typedef struct
{
    /**
     * The ratio between the master clock and bit clock signals.
     */
    uint32_t mclk_bclk_ratio;
    /**
     * Slave bit clock polarity.
     */
    tdm_slave_bclk_polarity_t slave_bclk_polarity;
    /**
     * Offset in bclk cycles between frame sync rising edge and first channel
     */
    uint32_t offset;
} tdm_config_t;

/**
 * Restart command type.
 *
 * Restart commands that can be signalled to the TDM component.
 */
typedef enum
{
    /**
     * Do not restart.
     */
    TDM_NO_RESTART = 0,
    /**
     * Restart the bus (causes the TDM to stop and a new init callback to occur
     * allowing reconfiguration of the bus).
     */
    TDM_RESTART,
    /**
     * Shutdown. This will cause the TDM component to exit.
     */
    TDM_SHUTDOWN
} tdm_restart_t;

/**
 * TDM initialization event callback.
 *
 * The TDM component will call this
 * when it first initializes on first run of after a restart.
 *
 * \param app_data    Points to application specific data supplied
 *                    by the application. May be used for context
 *                    data specific to each TDM task instance.
 *
 * \param tdm_config  This structure is provided if the connected
 *                    component drives an TDM bus. The members
 *                    of the structure should be set to the
 *                    required configuration.
 */
typedef void (*tdm_init_t)(void *app_data,
                           tdm_config_t *tdm_config);

/**
 * TDM restart check callback.
 *
 * This callback is called once per frame. The application must return the
 * required restart behavior.
 *
 * \param app_data  Points to application specific data supplied
 *                  by the application. May be used for context
 *                  data specific to each TDM task instance.
 *
 * \return          The return value should be set to
 *                  #TDM_NO_RESTART, #TDM_RESTART or
 *                  #TDM_SHUTDOWN.
 */
typedef tdm_restart_t (*tdm_restart_check_t)(void *app_data);

/**
 * Receive and supply samples to the TDM component in parallel with the driver.
 *
 * This callback will be executed at the start of a frame. It will continue
 * execution until the end of the frame; the user is responsible for ensuring
 * that the function can execute in time. It will be periodically interrupted
 * by the TDM instance itself in order to manage data IO. It will be called with
 * a buffer of data recieved during the previous frame.
 * It provides a buffer that it expects to be populated with the
 * data to be sent in the next frame. The user may define any arbitrary
 * behaviour desired, so long as 1) interrupts remain able to be serviced, and
 * 2) the function returns before the next frame begins.
 *
 * \param app_data         Points to application specific data supplied
 *                         by the application. May be used for context
 *                         data specific to each TDM task instance.
 *
 * \param num_out          The number of output arrays in \p send_samples
 *
 * \param num_in           The number of input arrays in \p receive_samples
 *
 * \param num_chans        The number of channels per input/output array in
 *                         \p send_samples and \p receive_samples
 *
 * \param num_data_bits    For non-32b transmission bit depths, the
 *                         least-significant \p num_data_bits bits will be the
 *                         desired signal. All other bits will hold undefined
 *                         values.
 *
 * \param receive_samples  The recieved samples as signed 32-bit values.
 *                         Contains \p num_in arrays of \p num_chans values.
 *                         If \p num_out is greater than 0, then on first call
 *                         this array will always be set to 0. NULL if this TDM
 *                         component has no inputs.
 *
 * \param send_samples     The samples to send, as signed 32-bit values.
 *                         Contains \p num_out arrays of \p num_chans values.
 *                         NULL if this TDM component has no outputs.
 */
typedef void (*tdm_process_t)(void *app_data,
                              size_t num_out,
                              size_t num_in,
                              size_t num_chans,
                              size_t num_data_bits,
                              tdm_sample_buffer_t * receive_samples,
                              tdm_sample_buffer_t * send_samples);

/**
 * This attribute must be specified on all TDM callback functions
 * provided by the application.
 */
#define TDM_CALLBACK_ATTR __attribute__((fptrgroup("tdm_callback")))

/**
 * Callback group representing callback events that can occur during the
 * operation of the TDM task. Must be initialized by the application prior
 * to passing it to one of the TDM tasks.
 */
typedef struct
{
    /**
     * Pointer to the application's tdm_init_t function to be called by the
     * TDM device
     */
    TDM_CALLBACK_ATTR tdm_init_t init;

    /**
     * Pointer to the application's tdm_restart_check_t function to be called
     * by the TDM device
     */
    TDM_CALLBACK_ATTR tdm_restart_check_t restart_check;

    /**
     * Pointer to the application's tdm_process_t function to be called by the
     * TDM device
     */
    TDM_CALLBACK_ATTR tdm_process_t process;

    /**
     * Pointer to application specific data. Passed to each callback.
     */
    void *app_data;
} tdm_callback_group_t;

DECLARE_JOB(tdm_master, (tdm_callback_group_t *const, const port_t *,
                         size_t, const port_t *, size_t,
                         size_t, size_t, port_t, port_t,
                         port_t, xclock_t));

DECLARE_JOB(tdm_master_external_clock, (tdm_callback_group_t *const,
                                        const port_t *, size_t,
                                        const port_t *, size_t,
                                        size_t, size_t,
                                        port_t, port_t,
                                        xclock_t));

DECLARE_JOB(tdm_slave, (tdm_callback_group_t *const, const port_t *, size_t,
                        const port_t *, size_t, size_t, size_t,
                        port_t, port_t, xclock_t));

/**
 * TDM master task.
 *
 * This task performs TDM on the provided pins. It will perform callbacks over
 * the tdm_callback_group_t callback group to get/receive frames of data from
 * the application using this component.
 *
 * The task performs TDM master so will drive the word clock and
 * bit clock lines.
 *
 * \param tdm_cbg        The TDM callback group pointing to the application's
 *                       functions to use for initialization and getting and
 *                       receiving frames. Also points to application specific
 *                       data which will be shared between the callbacks.
 * \param p_dout         An array of data output ports
 * \param num_out        The number of output data ports
 * \param p_din          An array of data input ports
 * \param num_in         The number of input data ports
 * \param num_chans      The number of channels per input/output data port
 * \param num_data_bits  The number of data bits per input/output channel
 * \param p_bclk         The bit clock output port
 * \param p_fsync        The word clock output port
 * \param p_mclk         Input port which supplies the master clock
 * \param bclk           A clock that will get configured for use with the bit
 *                       clock
 */
void tdm_master(tdm_callback_group_t *const tdm_cbg,
                const port_t p_dout[],
                size_t num_out,
                const port_t p_din[],
                size_t num_in,
                size_t num_chans,
                size_t num_data_bits,
                port_t p_bclk,
                port_t p_fsync,
                port_t p_mclk,
                xclock_t bclk);

/**
 * TDM master task
 *
 * This task differs from tdm_master() in that \p bclk must already be
 * configured to the BCLK frequency. Other than that, it is identical.
 *
 * This task performs TDM on the provided pins. It will perform callbacks over
 * the tdm_callback_group_t callback group to get/receive frames of data from
 * the application using this component.
 *
 * The task performs TDM master so will drive the word clock and
 * bit clock lines.
 *
 * \param tdm_cbg        The TDM callback group pointing to the application's
 *                       functions to use for initialization and getting and
 *                       receiving frames. Also points to application specific
 *                       data which will be shared between the callbacks.
 * \param p_dout         An array of data output ports
 * \param num_out        The number of output data ports
 * \param p_din          An array of data input ports
 * \param num_in         The number of input data ports
 * \param num_chans      The number of channels per input/output data port
 * \param num_data_bits  The number of data bits per input/output channel
 * \param p_bclk         The bit clock output port
 * \param p_fsync        The word clock output port
 * \param bclk           A clock that is configured externally to be used as the
 *                       bit clock
 */
void tdm_master_external_clock(tdm_callback_group_t *const tdm_cbg,
                               const port_t p_dout[],
                               size_t num_out,
                               const port_t p_din[],
                               size_t num_in,
                               size_t num_chans,
                               size_t num_data_bits,
                               port_t p_bclk,
                               port_t p_fsync,
                               xclock_t bclk);

/**
 * TDM slave task
 *
 * This task performs TDM on the provided pins. It will perform callbacks over
 * the tdm_callback_group_t callback group to get/receive data from the
 * application using this component.
 *
 * The component performs TDM slave so will expect the word clock and
 * bit clock to be driven externally.
 *
 * \param tdm_cbg        The TDM callback group pointing to the application's
 *                       functions to use for initialization and getting and
 *                       receiving frames. Also points to application specific
 *                       data which will be shared between the callbacks.
 * \param p_dout         An array of data output ports
 * \param num_out        The number of output data ports
 * \param p_din          An array of data input ports
 * \param num_in         The number of input data ports
 * \param num_chans      The number of channels per input/output data port
 * \param num_data_bits  The number of data bits per input/output channel
 * \param p_bclk         The bit clock input port
 * \param p_fsync        The word clock input port
 * \param bclk           A clock that will get configured for use with
 *                       the bit clock
 */
void tdm_slave(tdm_callback_group_t *const tdm_cbg,
               const port_t p_dout[],
               size_t num_out,
               const port_t p_din[],
               size_t num_in,
               size_t num_chans,
               size_t num_data_bits,
               port_t p_bclk,
               port_t p_fsync,
               xclock_t bclk);
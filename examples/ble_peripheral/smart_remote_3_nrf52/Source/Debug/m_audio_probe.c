/**
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <stdlib.h>
#include <string.h>

#include "nrf_cli.h"
#include "nrf_assert.h"

#include "m_audio_probe.h"

#include "SEGGER_RTT.h"
#include "sr3_config.h"

#if CONFIG_AUDIO_PROBE_ENABLED

#define NRF_LOG_MODULE_NAME m_audio_probe
#define NRF_LOG_LEVEL CONFIG_AUDIO_MODULE_LOG_LEVEL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

# if (!(CONFIG_CLI_ENABLED))
#  error "Audio Tool feature requires interactive console NRF_CLI to be enabled."
# endif

# if ((CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP) > SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS)
#  error "RTT channels configuration in sdk_config.h is incompatible with Smart Remote Audio Tool config. Too few up channels!"
# endif
# if ((CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN) > SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS)
#  error "RTT channels configuration in sdk_config.h is incompatible with Smart Remote Audio Tool config. Too few down channels!"
# endif

/*
 * "audio probe gain.out none\r\n" is a command 27 chars long. It is generated by Audio Tool
 * and pasted to the console, unlike when user is typing (sending one char at a time)
 * when console has enough time to process input.
 */
# define AUDIO_TOOL_COMMAND_MAX_SIZE 30

/*
 * With default SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN = 16, commands sent from
 * Audio Tool application are sometimes received by console only partially.
 * Increasing this value in sdk_config.h lets the console receive commands
 * without breaking them into parts and work more fluently.
 */
# if SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN < AUDIO_TOOL_COMMAND_MAX_SIZE
#  warning "When using Audio Tool it is recommended to increase SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN in sdk_config.h"
# endif

#define AUDIO_PROBE_POINT_NOT_CONNECTED         (int8_t)(-1)
#define AUDIO_PROBE_POINT_NOT_CONNECTED_STR     "none"
#define AUDIO_PROBE_INFO_SUBCOMMAND             "info"
#define AUDIO_CHANNEL_STRING_SIZE               5 // Maximum length of channel number strings ("1", "2", "none"), including trailing zero

/* Sizes are increased by 1 byte because SEGGER RTT channels are actually 1 byte smaller than demanded */
#define AUDIO_CHANNEL_UP_BUFFER_SIZE            ((sizeof(int16_t) * CONFIG_PDM_BUFFER_SIZE_SAMPLES * CONFIG_AUDIO_PROBE_RTT_TAP_BUFFERS) + 1)
#define AUDIO_CHANNEL_DOWN_BUFFER_SIZE          ((sizeof(int16_t) * CONFIG_PDM_BUFFER_SIZE_SAMPLES * CONFIG_AUDIO_PROBE_RTT_INJECT_BUFFERS) + 1)

/**@brief Type representing audio probe point direction. */
typedef enum {
    M_AUDIO_PROBE_INPUT,        /**< Inject audio to Smart Remote */
    M_AUDIO_PROBE_OUTPUT,       /**< Tap audio from Smart Remote  */
    M_AUDIO_PROBE_INVALID_TYPE  /**< Invalid probe point type */
} m_audio_probe_point_type_t;

/**@brief Type representing audio probe point in the audio chain. */
typedef struct
{
    const char                  *name;
    m_audio_probe_point_type_t   type;
    int8_t                      *p_channel;
} m_audio_probe_point_t;

static int8_t m_audio_test_point_channels[M_AUDIO_PROBE_POINTS_NUM];

#define DEFINE_AUDIO_POINT(_test_point, _name, _point_type)             \
        [_test_point] = {                                               \
            .name = (_name),                                            \
            .type = (_point_type),                                      \
            .p_channel = (&m_audio_test_point_channels[_test_point])    \
        }

/*
 * Array of audio probe points. Sorted in order of occurence in audio chain.
 * Note that in the defined array the points will be sorted alphabetically,
 * due to order in m_audio_probe_point_enum_t type.
 */
const m_audio_probe_point_t audio_test_points[] = {
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_PDM_OUT, "pdm.out", M_AUDIO_PROBE_OUTPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_ANR_IN, "anr.in", M_AUDIO_PROBE_INPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_ANR_OUT, "anr.out", M_AUDIO_PROBE_OUTPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_EQ_IN, "eq.in", M_AUDIO_PROBE_INPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_EQ_OUT, "eq.out", M_AUDIO_PROBE_OUTPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_GAIN_IN, "gain.in", M_AUDIO_PROBE_INPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_GAIN_OUT, "gain.out", M_AUDIO_PROBE_OUTPUT),
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_POINT_CODEC_IN, "codec.in", M_AUDIO_PROBE_INPUT),

    // Dummy entry for emulating static subcommand "audio probe info" along with dynamic subcommands
    DEFINE_AUDIO_POINT(M_AUDIO_PROBE_INFO_SUBCOMMAND, AUDIO_PROBE_INFO_SUBCOMMAND, M_AUDIO_PROBE_INPUT)
};

STATIC_ASSERT(M_AUDIO_PROBE_POINTS_NUM == ARRAY_SIZE(audio_test_points));

/* RTT buffers for audio */
#if CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP
static uint8_t m_channels_up[CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP][AUDIO_CHANNEL_UP_BUFFER_SIZE];
#endif
#if CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN
static uint8_t m_channels_down[CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN][AUDIO_CHANNEL_DOWN_BUFFER_SIZE];
#endif

/* Arrays with channel strings for autocompletion. Size is the number of available channels + 1 for word "none". */
#define M_AUDIO_PROBE_CHANNEL_STRINGS (1 + MAX(CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP, CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN))
static char m_audio_channels_str[M_AUDIO_PROBE_CHANNEL_STRINGS][AUDIO_CHANNEL_STRING_SIZE];

/**@brief Generate array of string channels for valid up or down channels for autocompletion.
 *
 * Catch overrun when there is not enough place to write the string in buffer. */
static void m_audio_probe_generate_channel_strings()
{
    unsigned int printed_chars = snprintf(m_audio_channels_str[0],
                                          AUDIO_CHANNEL_STRING_SIZE,
                                          "%s",
                                          AUDIO_PROBE_POINT_NOT_CONNECTED_STR);
    ASSERT(printed_chars < AUDIO_CHANNEL_STRING_SIZE);

    for (unsigned int channel = CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST, index = 1;
         index < CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + M_AUDIO_PROBE_CHANNEL_STRINGS;
         channel++, index++)
    {
        ASSERT(index < M_AUDIO_PROBE_CHANNEL_STRINGS + CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST);
        printed_chars = snprintf(m_audio_channels_str[index],
                                 AUDIO_CHANNEL_STRING_SIZE,
                                 "%u",
                                 channel);
        ASSERT(printed_chars < AUDIO_CHANNEL_STRING_SIZE);
    }

    UNUSED_VARIABLE(printed_chars); // with assertions disabled, the variable would be unused
}

/**@brief Allocate RTT buffers for audio probe channels. */
static void m_audio_probe_channels_init(size_t channels_num,
                                        size_t channel_size,
                                        m_audio_probe_point_type_t direction)
{
    for (unsigned int channel = CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST;
         channel < CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + channels_num;
         channel++)
    {
        if (direction == M_AUDIO_PROBE_OUTPUT)
        {
# if CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP
            SEGGER_RTT_ConfigUpBuffer(channel,
                                      "Audio Tap Channel",
                                      m_channels_up[channel - CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST],
                                      channel_size,
                                      SEGGER_RTT_MODE_NO_BLOCK_SKIP);
# endif
        }
        else
        {
# if CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN
            SEGGER_RTT_ConfigDownBuffer(channel,
                                      "Audio Inject Channel",
                                      m_channels_down[channel - CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST],
                                      channel_size,
                                      SEGGER_RTT_MODE_NO_BLOCK_SKIP);
# endif
        }
    }
}

static void m_audio_probe_points_init()
{
    // Set all points to not connected
    for (unsigned int point = 0; point < M_AUDIO_PROBE_POINTS_NUM; point++)
    {
        *(audio_test_points[point].p_channel) = AUDIO_PROBE_POINT_NOT_CONNECTED;

        if (point > 0)
        {
            if ((strcmp(audio_test_points[point - 1].name, audio_test_points[point].name) >= 0))
            {
                NRF_LOG_ERROR("Array of audio probe points is not sorted in alphabetical order");
                APP_ERROR_CHECK_BOOL(false);
            }
        }
    }
}

void m_audio_probe_init(void)
{
    m_audio_probe_points_init();
    m_audio_probe_generate_channel_strings();

# if CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP
    m_audio_probe_channels_init(CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP,
                                AUDIO_CHANNEL_UP_BUFFER_SIZE,
                                M_AUDIO_PROBE_OUTPUT);
# endif
# if CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN
    m_audio_probe_channels_init(CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN,
                                AUDIO_CHANNEL_DOWN_BUFFER_SIZE,
                                M_AUDIO_PROBE_INPUT);
# endif

    NRF_LOG_INFO("Initialized audio instrumentation subsystem");
    NRF_LOG_INFO("Available %u down, %u up audio channels", CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN, CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP);
}

/**@brief Check if audio probe channel number is valid in current cofiguration. */
static bool m_audio_probe_channel_valid(int8_t channel, m_audio_probe_point_type_t point_type)
{
    bool channel_valid;

    if (point_type == M_AUDIO_PROBE_INPUT)
    {
        channel_valid = ((channel < (CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + CONFIG_AUDIO_PROBE_RTT_CHANNELS_DOWN)) &&
                         (channel >= CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST));
    }
    else
    {
        channel_valid = ((channel < (CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST + CONFIG_AUDIO_PROBE_RTT_CHANNELS_UP)) &&
                        (channel >= CONFIG_AUDIO_PROBE_RTT_CHANNEL_FIRST));
    }

    return channel_valid;
}

/**@brief Check if a probe channel is connected to a probe point */
static bool m_audio_probe_channel_is_occupied(int8_t channel, m_audio_probe_point_type_t point_type)
{
    for (unsigned int point = 0; point < M_AUDIO_PROBE_POINTS_NUM; point++)
    {
        if (*(audio_test_points[point].p_channel) == channel) // channel occupied
        {
            if (audio_test_points[point].type == point_type)
            {
                return true;
            }
        }
    }

    return false;
}

/**@brief Check if provided name is valid probe point name. */
static bool m_audio_probe_point_name_valid(const char *point_name)
{
    for (unsigned int point = 0; point < M_AUDIO_PROBE_POINTS_NUM; point++)
    {
        if (point == M_AUDIO_PROBE_INFO_SUBCOMMAND)
        {
            continue;
        }
        if (strcmp(point_name, audio_test_points[point].name) == 0)
        {
            return true;
        }
    }

    return false;
}

/**@brief Identify the direction (inject/tap) of point name typed by user.
 *
 * It should only be invoked after checking if the point is valid.
 */
static m_audio_probe_point_type_t m_audio_probe_point_type(const char *point_name)
{
    for (int point = 0; point < M_AUDIO_PROBE_POINTS_NUM; ++point)
    {
        if (strcmp(point_name, audio_test_points[point].name) == 0)
        {
            return audio_test_points[point].type;
        }
    }

    return M_AUDIO_PROBE_INVALID_TYPE;
}

/**@brief Function connecting chosen probe point to audio channel. */
static uint32_t m_audio_probe_connect_point(const char                  *name,
                                            int8_t                      channel,
                                            bool                        disconnect)
{
    m_audio_probe_point_type_t direction = m_audio_probe_point_type(name);

    if ((!disconnect) && (m_audio_probe_channel_is_occupied(channel, direction)))
    {
        return NRF_ERROR_BUSY;
    }

    for (int point = 0 ; point < M_AUDIO_PROBE_POINTS_NUM; ++point)
    {
        if (strcmp(name, audio_test_points[point].name) == 0)
        {
            if (disconnect)
            {
                *(audio_test_points[point].p_channel) = AUDIO_PROBE_POINT_NOT_CONNECTED;
            }
            else
            {
                *(audio_test_points[point].p_channel) = channel;
            }
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_INVALID_PARAM;
}

/**@brief CLI command "audio probe info" for inspecting current state of inject & tap points. */
static void m_audio_probe_info_cli_cmd(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%12s\t%s\r\n", "Point", "Channel");
    for (int point = 0; point < M_AUDIO_PROBE_POINTS_NUM; ++point)
    {
        if (point == M_AUDIO_PROBE_INFO_SUBCOMMAND)
        {
            continue;
        }
        if (*(audio_test_points[point].p_channel) == AUDIO_PROBE_POINT_NOT_CONNECTED)
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%12s\t%s\r\n",
                            audio_test_points[point].name, AUDIO_PROBE_POINT_NOT_CONNECTED_STR);
        }
        else
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%12s\t%d\r\n",
                            audio_test_points[point].name, *(audio_test_points[point].p_channel));
        }
    }
}

/**@brief CLI handler function "audio probe" */
void m_audio_probe_cmd(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    bool disconnect = false;
    int8_t channel;
    const char *name;
    uint32_t retval;

    static nrf_cli_getopt_option_t const opt[] = {
        NRF_CLI_OPT(
            "<point>",
            NULL,
            "probe point to be connected/disconnected"
        ),
        NRF_CLI_OPT(
            "<channel>",
            NULL,
            "audio channel to be connected to <point>"
        )
    };

    // Show help and usage if -h or --help was used.
    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, opt, ARRAY_SIZE(opt));
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL,
            "Example usage:\r\n"
            "\taudio probe anr.in 2\r\n"
            "\taudio probe pdm.out none\r\n");
        return;
    }

    if (strcmp(argv[1], AUDIO_PROBE_INFO_SUBCOMMAND) == 0)
    {
        // User typed "audio probe info"
        m_audio_probe_info_cli_cmd(p_cli, argc, argv);
        return;
    }

    // Handle user arguments
    if (argc < 3)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Not enough arguments!\r\n");
        return;
    }

    if (!m_audio_probe_point_name_valid(argv[1]))
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Probe point '%s' invalid.\r\n", argv[1]);
        return;
    }

    channel = strtol(argv[2], NULL, 10);
    if (strcmp(argv[2], AUDIO_PROBE_POINT_NOT_CONNECTED_STR) == 0)
    {
        disconnect = true;
    }
    else if (!m_audio_probe_channel_valid(channel, m_audio_probe_point_type(argv[1])))
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Channel '%s' invalid for audio %s.\r\n",
                        argv[2],
                        m_audio_probe_point_type(argv[1]) ? "tap" :  "inject");
        return;
    }

    name = argv[1];
    if (m_audio_probe_point_type(name) == M_AUDIO_PROBE_INVALID_TYPE)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Invalid type for probe point '%s'.\r\n", name);
        return;
    }

    // Connect parsed probe point
    retval = m_audio_probe_connect_point(name, channel, disconnect);

    // Handle internal state of audio probe
    switch (retval)
    {
        case NRF_ERROR_BUSY:
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Channel %d is already occupied.\r\n", channel);
            break;

        case NRF_ERROR_INVALID_PARAM:
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Cannot connect point '%s'.\r\n", argv[1]);
            break;

        case NRF_SUCCESS:
            break;

        default:
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Unknown error!\r\n");
    }
}

void m_audio_probe_point(m_audio_probe_point_enum_t point, int16_t *buffer, size_t buffer_size_samples)
{
    size_t bytes_transfered;
    size_t buffer_size_bytes = buffer_size_samples * sizeof(buffer[0]);

    if (*(audio_test_points[point].p_channel) != AUDIO_PROBE_POINT_NOT_CONNECTED)
    {
        if (audio_test_points[point].type == M_AUDIO_PROBE_INPUT)
        {
            /* inject data */
            bytes_transfered = SEGGER_RTT_Read(*(audio_test_points[point].p_channel), buffer, buffer_size_bytes);
            NRF_LOG_DEBUG("%s: received %d bytes.", audio_test_points[point].name, bytes_transfered);
            if (bytes_transfered < buffer_size_bytes)
            {
                NRF_LOG_WARNING("Injected audio buffer incomplete. Discarding injected data.");
                memset(buffer, 0, buffer_size_bytes);
            }
        }
        else
        {
            /* tap data */
            bytes_transfered = SEGGER_RTT_Write(*(audio_test_points[point].p_channel), buffer, buffer_size_bytes);
            NRF_LOG_DEBUG("%s: sent %d bytes.", audio_test_points[point].name, bytes_transfered);
            if (bytes_transfered < buffer_size_bytes)
            {
                NRF_LOG_WARNING("Tapped an incomplete audio buffer. Recorded sound may be corrupted.");
            }
        }
    }
}

/**@brief Dynamic subcommand for autocompletion of audio probe channels */
static void m_audio_probe_channels_get(size_t idx, nrf_cli_static_entry_t * p_static)
{
    ASSERT(p_static);

    p_static->handler = NULL;
    p_static->p_help  = NULL;
    p_static->p_subcmd = NULL;
    p_static->p_syntax = (idx < ARRAY_SIZE(m_audio_channels_str)) ? m_audio_channels_str[idx] : NULL;
}

NRF_CLI_CREATE_DYNAMIC_CMD(m_dynamic_cmd_probe_channel, m_audio_probe_channels_get);

/**@brief Dynamic subcommand for autocompletion of audio probe points */
static void m_audio_probe_points_get(size_t idx, nrf_cli_static_entry_t * p_static)
{
    ASSERT(p_static);

    p_static->handler = NULL;
    p_static->p_help  = NULL;
    p_static->p_subcmd = &m_dynamic_cmd_probe_channel;
    p_static->p_syntax = (idx < M_AUDIO_PROBE_POINTS_NUM) ? audio_test_points[idx].name : NULL;
}
//NRF_CLI_CREATE_DYNAMIC_CMD(m_audio_probe_subcmds, m_audio_probe_points_get);

const nrf_cli_cmd_entry_t m_audio_probe_subcmds =
{
    .is_dynamic      = true,
    .u.p_dynamic_get = &m_audio_probe_points_get,
};

#endif /* CONFIG_AUDIO_PROBE_ENABLED */

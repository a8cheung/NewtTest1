#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "arm_math.h"
#include "nordic_common.h"
#include "nrf_atomic.h"
#include "nrf_assert.h"
#include "nrf_gpio.h"
#include "app_debug.h"
#include "app_error.h"
#include "app_gpiote.h"

#include "drv_touchpad.h"

#include "twi_common.h"
#include "resources.h"

#define CONFIG_TOUCHPAD_ENABLED 1
#define CONFIG_TOUCHPAD_DRV 2

#if CONFIG_TOUCHPAD_ENABLED && (CONFIG_TOUCHPAD_DRV == 2)

#define NRF_LOG_MODULE_NAME drv_touchpad_mtch6102
#define NRF_LOG_LEVEL CONFIG_TOUCHPAD_DRV_LOG_LEVEL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

//#define PRODUCT_ID_BYTES       10U  //!< Number of bytes to be expected to be in the product ID.
#define DEVICE_ADDRESS         0x25 //!< Device address on Two-wire bus.

#define TP_GPIO_INT            9
#define TP_GPIO_SYNC           10
#define TP_GPIO_INPUTS         ( (1ul << TP_GPIO_INT) \
                                   | (1ul << TP_GPIO_SYNC) )
#define TP_GPIO_INTERRUPT      (1ul << TP_GPIO_INT)

/**
 * Touchpad register addresses
 */
#define TOUCHPAD_MODE_REG        0x05 //!< Device mode register.
#define TOUCHPAD_TOUCH_DATA_BASE 0x10 //!< Touch RAM memory map base adr

#define TOUCHPAD_MAX_X           576
#define TOUCHPAD_MAX_Y           384

#define TOUCHPAD_LOW_TH_X        ( TOUCHPAD_MAX_X / 4)
#define TOUCHPAD_HIGH_TH_X       ( (3*TOUCHPAD_MAX_X) / 4)
#define TOUCHPAD_LOW_TH_Y        ( TOUCHPAD_MAX_Y / 6)
#define TOUCHPAD_HIGH_TH_Y       ( (4*TOUCHPAD_MAX_Y) / 6)

//#define TOUCHPAD_PRODUCT_ID    0xA2 //!< Address of the product ID string.

/**@brief Operational states */
typedef enum {
    SleepmodeNormal = 0x03, /**< Normal operation. */
    SleepmodeSensorSleep = 0x00, /**< Low power operation. */
} TouchpadSleepMode_t;


//static ret_code_t touchpad_product_id_verify(void);
//static ret_code_t touchpad_product_id_read(uint8_t *product_id, uint8_t product_id_bytes);

static void touchpad_read_callback(ret_code_t status, void *p_user_data);
static void touchpad_enable_disable_callback(ret_code_t status, void *p_user_data);
static void touchpad_init_callback(ret_code_t status, void *p_user_data);

static drv_touchpad_read_handler_t m_read_handler;
//static uint8_t m_finger_state[3];
//static uint8_t m_buffer[7];
static uint8_t m_buffer[6];

static int16_t prev_touch_x;
static int16_t prev_touch_y;
static uint8_t touch_readings;

static drv_touchpad_data_t *p_user_data;

/**@brief Flag protecting shared data used in read operation */
static nrf_atomic_flag_t m_read_operation_active;

static app_gpiote_user_id_t  m_touchpad_gpiote; //!< GPIOTE Handle.


static uint8_t m_touch_data_base_reg = TOUCHPAD_TOUCH_DATA_BASE;

static const nrf_twi_mngr_transfer_t m_touchpad_read_transfers[] = {
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, &m_touch_data_base_reg, 1, NRF_TWI_MNGR_NO_STOP),
NRF_TWI_MNGR_READ (DEVICE_ADDRESS, &m_buffer[0], 6, 0), };

static uint8_t m_touch_layout_conf[] = { 0x20, // Reg 0x20
        0x9, 0x6, 0x6, 0x4F, 0x3F }; //

static uint8_t m_touch_gesture_conf[] = { 0x37, //Reg 0x37
        0x40, 0x40, 0x19, 0x19, 0x40, 0x32, 0x00, 0x0C, 0x20, 0x04, 0x2D, 0x2D };

static uint8_t m_touch_filter_conf[] = { 0x31, //Reg 0x31
        0x03, 0x01 };

static const nrf_twi_mngr_transfer_t m_touchpad_init_transfers[] = {
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, m_touch_layout_conf, sizeof(m_touch_layout_conf), 0),
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, m_touch_gesture_conf, sizeof(m_touch_gesture_conf), 0),
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, m_touch_filter_conf, sizeof(m_touch_filter_conf), 0), };

static const nrf_twi_mngr_transaction_t m_init_transaction = { .callback = touchpad_init_callback,
        .p_user_data = NULL, .p_transfers = m_touchpad_init_transfers, .number_of_transfers =
                ARRAY_SIZE(m_touchpad_init_transfers), .p_required_twi_cfg =
                &g_twi_bus_config[CONFIG_TOUCHPAD_TWI_BUS] };

static uint8_t m_enable_data[] = { TOUCHPAD_MODE_REG, SleepmodeNormal };
static uint8_t m_disable_data[] = { TOUCHPAD_MODE_REG, SleepmodeSensorSleep };

static const nrf_twi_mngr_transfer_t m_enable_transfers[] = {
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, m_enable_data, sizeof(m_enable_data), 0), };

static const nrf_twi_mngr_transfer_t m_disable_transfers[] = {
NRF_TWI_MNGR_WRITE (DEVICE_ADDRESS, m_disable_data, sizeof(m_disable_data), 0), };

static const nrf_twi_mngr_transaction_t m_enable_transaction = { .callback =
        touchpad_enable_disable_callback, .p_user_data = NULL, .p_transfers = m_enable_transfers,
        .number_of_transfers = ARRAY_SIZE(m_enable_transfers), .p_required_twi_cfg =
                &g_twi_bus_config[CONFIG_TOUCHPAD_TWI_BUS] };

static const nrf_twi_mngr_transaction_t m_disable_transaction = { .callback =
        touchpad_enable_disable_callback, .p_user_data = NULL, .p_transfers = m_disable_transfers,
        .number_of_transfers = ARRAY_SIZE(m_disable_transfers), .p_required_twi_cfg =
                &g_twi_bus_config[CONFIG_TOUCHPAD_TWI_BUS] };

/**@brief Set desired configuration on given GPIOs. */
static void drv_keyboard_gpio_cfg(uint32_t mask, uint32_t pin_cnf) {
    unsigned int i = 8 * sizeof(mask);

    while ((i--, mask != 0)) {
        unsigned int n = __CLZ(mask);

        i -= n;
        mask <<= n + 1;

        NRF_GPIO->PIN_CNF[i] = pin_cnf;
    }

    // Make sure that new confiuration is active.
    __DSB();
}

static void touchpad_read_callback(ret_code_t status, void *p_user_data) {
    drv_touchpad_data_t *p_data = p_user_data;
    //    static bool tap_is_held;
    //    float       x32f;
    //    float       y32f;
    //    float       sqrtf;

    if (status != NRF_SUCCESS) {
        nrf_atomic_flag_clear(&m_read_operation_active);
        m_read_handler(status, p_data);
        return;
    }

    if (p_data->scroll > 0)
        p_data->scroll--;
    else if (p_data->scroll < 0)
        p_data->scroll++;

    if (p_data->pan > 0)
        p_data->pan--;
    else if (p_data->pan < 0)
        p_data->pan++;

    p_data->tap = false;


#if 0
    p_data->x = 0;
    p_data->y = 0;

    if (m_buffer[0] & 0x02) { // Gesture
        uint8_t swipe_speed;
        if (touch_readings <= 4) {
            swipe_speed = 3;
        }
        else if (touch_readings <= 8) {
            swipe_speed = 2;
        }
        else {
            swipe_speed = 1;
        }

        if (m_buffer[4] == 0x10) {
            NRF_LOG_INFO("***IGOR* Gesture Single click");
            p_data->tap = true;
        } else if (m_buffer[4] == 0x31) { //True Down
            NRF_LOG_INFO("***IGOR* Gesture Left s=%u", swipe_speed);
            p_data->x = -swipe_speed;
        } else if (m_buffer[4] == 0x41) { //True Right
            NRF_LOG_INFO("***IGOR* Gesture Down s=%u", swipe_speed);
            p_data->y = swipe_speed;
        } else if (m_buffer[4] == 0x51) { //True Up
            NRF_LOG_INFO("***IGOR* Gesture Right s=%u", swipe_speed);
            p_data->x = swipe_speed;
        } else if (m_buffer[4] == 0x61) { //True Left
            NRF_LOG_INFO("***IGOR* Gesture Up s=%u", swipe_speed);
            p_data->y = -swipe_speed;
        } else {
            NRF_LOG_INFO("***IGOR* Other gesture");
        }
    }

    if (m_buffer[0] & 0x01)  // Touch
        touch_readings++;
    else
        touch_readings = 0;

#else
    if (m_buffer[0] & 0x02) { // Gesture
        if (m_buffer[4] == 0x10) {
            NRF_LOG_INFO("***IGOR* Gesture Single click");
            //p_data->tap = true;
        }
        else if (m_buffer[4] == 0x11) {
            //NRF_LOG_INFO("***IGOR* Gesture click & hold");
        }
        else if (m_buffer[4] == 0x20) {
            NRF_LOG_INFO("***IGOR* Gesture Double click");
        }
        else if (m_buffer[4] == 0x31) {
            //True Down
            NRF_LOG_INFO("***IGOR* Gesture Left swipe");
        }
        else if (m_buffer[4] == 0x41 && touch_readings <= 7) {
            //True Right
            NRF_LOG_INFO("***IGOR* Gesture Down swipe tr=%u", touch_readings);
            int8_t diff = (10 - touch_readings);
            if (diff > 0)
                p_data->scroll = diff;

            p_data->x = 0;
            p_data->y = 0;
            prev_touch_x = INT16_MAX;
            prev_touch_y = INT16_MAX;
        }
        else if (m_buffer[4] == 0x51) {
            //True Up
            NRF_LOG_INFO("***IGOR* Gesture Right swipe");
        }
        else if (m_buffer[4] == 0x61 && touch_readings <=7) {
            //True Left
            NRF_LOG_INFO("***IGOR* Gesture Up swipe tr=%u", touch_readings);
            int8_t diff = (10 - touch_readings);
            if (diff > 0)
                p_data->scroll = -diff;

            p_data->x = 0;
            p_data->y = 0;
            prev_touch_x = INT16_MAX;
            prev_touch_y = INT16_MAX;
        }
        else {
//            if (m_buffer[4] & 0x02) {
//                NRF_LOG_INFO("***IGOR* Gesture swipe&hold %02x", m_buffer[4]);
//            } else {
//                NRF_LOG_INFO("***IGOR* Gesture unkown %02x", m_buffer[4]);
//            }
        }
    }

    if (m_buffer[0] & 0x01) { // Touch
        int16_t x = (m_buffer[1] << 4) | (m_buffer[3] >> 4);
        int16_t y = (m_buffer[2] << 4) | ((m_buffer[3] & 0xF0) >> 4);

        touch_readings++;

        if (touch_readings > 4) {
            p_data->y = (x - prev_touch_x) / 5;
            p_data->x = (prev_touch_y - y) / 3;
        }
        else {
            p_data->y = 0;
            p_data->x =0;

            if (touch_readings > 1) {
                prev_touch_x = ((touch_readings-1)*prev_touch_x + x)/touch_readings;
                prev_touch_y = ((touch_readings-1)*prev_touch_y + y)/touch_readings;
            }
            else {
                prev_touch_x = 0;
                prev_touch_y = 0;
            }
        }

        prev_touch_x = (3*prev_touch_x + x)/4;
        prev_touch_y = (3*prev_touch_y + y)/4;

        //    	x32f = (float) (x - p_data->x);
        //    	arm_sqrt_f32(x32f * (x32f < 0 ? -1 : 1), &sqrtf);
        //    	p_data->x = (int16_t)(x32f * sqrtf);

        //    	y32f = (float) (y - p_data->y);
        //    	arm_sqrt_f32(y32f * (y32f < 0 ? -1 : 1), &sqrtf);
        //    	p_data->y = -(int16_t)(y32f * sqrtf);

        //NRF_LOG_INFO("*IGOR* Touch x=%04x, y=%04x, b0=%02x", x, y, m_buffer[0]);
    } else {
        if (p_data->x > 5) {
            p_data->x = p_data->x / 2;
        } else if (p_data->x < -5) {
            p_data->x = p_data->x / 2;
        } else {
            p_data->x = 0;

            prev_touch_x = INT16_MAX;
        }

        if (p_data->y > 5) {
            p_data->y = p_data->y / 2;
        } else if (p_data->y < -5) {
            p_data->y = p_data->y / 2;
        } else {
            p_data->y = 0;

            prev_touch_y = INT16_MAX;
        }

        NRF_LOG_INFO("****IGOR* OTA APP Touch End");
        touch_readings = 0;
    }
#endif

    if (m_buffer[0] & 0x04) { // Large Activation
        NRF_LOG_INFO("****IGOR* Large activation");
    }

    nrf_atomic_flag_clear(&m_read_operation_active);
    m_read_handler(status, p_data);
}

static void touchpad_enable_disable_callback(ret_code_t status, void *p_user_data) {
    APP_ERROR_CHECK(status);
}

static void touchpad_init_callback(ret_code_t status, void *p_user_data) {
    if (status != NRF_SUCCESS) {
        NRF_LOG_INFO("*IGOR* touchpad_init_callback");
    }
    APP_ERROR_CHECK(status);
}

static void drv_touchpad_interrupt_handler(uint32_t const *p_event_pins_low_to_high,
        uint32_t const *p_event_pins_high_to_low)
{
    //NRF_LOG_INFO("*IGOR********* Interrupt");
    drv_touchpad_schedule_read(p_user_data);
}

uint16_t drv_touchpad_dpad_ok_clicked(void) {
    NRF_LOG_INFO("*IGOR* x=%d, y=%d", prev_touch_x, prev_touch_y);
    if (prev_touch_x == INT16_MAX && prev_touch_y == INT16_MAX) {
        NRF_LOG_INFO("*IGOR* DPAD OK");
        return KEY_OK;
    }

    if ((prev_touch_x <= TOUCHPAD_LOW_TH_X) &&
            (prev_touch_y >= TOUCHPAD_LOW_TH_Y) && (prev_touch_y <= TOUCHPAD_HIGH_TH_Y)) {
        NRF_LOG_INFO("*IGOR* DPAD UP");
        return KEY_UP;
    }
    else if ((prev_touch_x >= TOUCHPAD_HIGH_TH_X) &&
            (prev_touch_y >= TOUCHPAD_LOW_TH_Y) && (prev_touch_y <= TOUCHPAD_HIGH_TH_Y)) {
        NRF_LOG_INFO("*IGOR* DPAD DOWN");
        return KEY_DOWN;
    }
    else if ((prev_touch_y <= TOUCHPAD_LOW_TH_Y) &&
            (prev_touch_x >= TOUCHPAD_LOW_TH_X) && (prev_touch_x <= TOUCHPAD_HIGH_TH_X)) {
        NRF_LOG_INFO("*IGOR* DPAD RIGHT");
        return KEY_RIGHT;
    }
    else if ((prev_touch_y >= TOUCHPAD_HIGH_TH_Y) &&
            (prev_touch_x >= TOUCHPAD_LOW_TH_X) && (prev_touch_x <= TOUCHPAD_HIGH_TH_X)) {
        NRF_LOG_INFO("*IGOR* DPAD LEFT");
        return KEY_LEFT;
    }

    return KEY_OK;
}

ret_code_t drv_touchpad_init(drv_touchpad_read_handler_t read_handler, drv_touchpad_data_t *p_u_data) {
    ret_code_t status;

    p_user_data = p_u_data;

    ASSERT(read_handler != NULL);

    // Read and verify product ID.
    //    status = touchpad_product_id_verify();
    //    if (status != NRF_SUCCESS)
    //    {
    //        return status;
    //    }

    prev_touch_x = INT16_MAX;
    prev_touch_y = INT16_MAX;
    touch_readings = 0;

    uint32_t low_to_high_mask = TP_GPIO_INTERRUPT;
    uint32_t high_to_low_mask = TP_GPIO_INTERRUPT;

    status = app_gpiote_user_register(&m_touchpad_gpiote,
            &low_to_high_mask,
            &high_to_low_mask,
            drv_touchpad_interrupt_handler);
    if (status != NRF_SUCCESS)
    {
        NRF_LOG_INFO("*IGOR* Interrupt init error");
        return status;
    }

    // Configure TP GPIOs.
    drv_keyboard_gpio_cfg(TP_GPIO_INPUTS,
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
            | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos));

    nrf_atomic_flag_clear(&m_read_operation_active);
    m_read_handler = read_handler;

    status = twi_schedule(&m_init_transaction);
    if (status != NRF_SUCCESS) {
        NRF_LOG_INFO("*IGOR* TP init err");
    }

    return drv_touchpad_disable();
}

ret_code_t drv_touchpad_enable(void) {
    NRF_LOG_INFO("*IGOR* TP_enable");
    ret_code_t status;

    // Disable GPIOTE sensing.
    status = app_gpiote_user_enable(m_touchpad_gpiote);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    return twi_schedule(&m_enable_transaction);
}

ret_code_t drv_touchpad_disable(void) {
    NRF_LOG_INFO("*IGOR* TP_disable");
    ret_code_t status;

    // Disable GPIOTE sensing.
    status = app_gpiote_user_disable(m_touchpad_gpiote);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    return twi_schedule(&m_disable_transaction);
}

ret_code_t drv_touchpad_schedule_read(drv_touchpad_data_t *p_data) {
    static nrf_twi_mngr_transaction_t read_transaction;
    ret_code_t status;

    ASSERT(p_data != NULL);

    if (nrf_atomic_flag_set_fetch(&m_read_operation_active)) {
        return NRF_ERROR_BUSY;
    }

    read_transaction.callback = touchpad_read_callback;
    read_transaction.p_user_data = p_data;
    read_transaction.p_transfers = m_touchpad_read_transfers;
    read_transaction.number_of_transfers = ARRAY_SIZE(m_touchpad_read_transfers);
    read_transaction.p_required_twi_cfg = &g_twi_bus_config[CONFIG_TOUCHPAD_TWI_BUS];

    status = twi_schedule(&read_transaction);
    if (status != NRF_SUCCESS) {
        nrf_atomic_flag_clear(&m_read_operation_active);
    }

    return status;
}


#endif /* CONFIG_TOUCHPAD_ENABLED */

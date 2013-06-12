/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup from button, advertise, get a connection
 * restart advertising on disconnect and if no new connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified with 'YOUR_JOB' indicates where
 * and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
//#include "ble_nrf6310_pins.h"
#include "ble_eval_board_pins.h"
#include "ble_stack_handler.h"
#include "app_timer.h"

#include "ble_bls.h"
#include "hexparser.h"

#define APPLICATION_BASE_ADDRESS 0x20000
//#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */
#define WAKEUP_BUTTON_PIN               EVAL_BOARD_BUTTON_0                            /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_PIN                   NRF6310_BUTTON_1

#define DEVICE_NAME                     "BLEloader"                           /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               6                  /**< Minimum acceptable connection interval (0.0075 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               8                  /**< Maximum acceptable connection interval (0.01 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   10                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                100                   /**< Connection supervisory timeout (0.1 seconds), Supervision Timeout uses 10 ms units. */


#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_bls_t                        m_bls;


/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);

    while(1);
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your 
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    
    // YOUR_JOB: Add additional LED initialiazations if needed.
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    
    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    // YOUR_JOB: Use UUIDs for service(s) used in your application.
    ble_uuid_t adv_uuids[] = {{BLS_UUID_SERVICE, m_bls.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

void erase_app(void)
{
    uint32_t last_page_address = NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE;

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;

    for (uint32_t address = APPLICATION_BASE_ADDRESS; address < last_page_address; address += NRF_FICR->CODEPAGESIZE)
    {
        NRF_NVMC->ERASEPAGE = address;
        while (!NRF_NVMC->READY);
    }

    NRF_NVMC->CONFIG = 0x0;
}

void write_record(uint16_t base_address, hexparser_record * record)
{
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;

    for (uint32_t i = 0; i < record->byte_count / 4; i++)
    {
        while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos))
        {
        }
        *(uint32_t *) (base_address << 4 | (record->address + (4*i))) = record->data.words[i];
    }
    
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
}

ble_bls_response_t write_line(ble_bls_evt_t * evt)
{
    static uint16_t base_address = 0;
    hexparser_record record;
    hexparser_parse_string((char * ) evt->data, evt->len, &record);

    if (!hexparser_is_record_valid(&record))
    {
        return BLE_BLS_RESPONSE_FAILURE;
    }

    switch (record.type)
    {
        case EXTENDED_LINEAR_ADDRESS_RECORD:
        case EXTENDED_SEGMENT_ADDRESS_RECORD:
            base_address = record.data.words[0];
            break;

        case DATA_RECORD:
            write_record(base_address, &record);
            break;

        default:
            break;
    }



    return BLE_BLS_RESPONSE_SUCCESS;
}

static void ble_bls_evt_handler(ble_bls_t * p_bls, ble_bls_evt_t * p_bls_evt)
{
    ble_bls_response_t result;
    switch (p_bls_evt->cmd)
    {
        case BLE_BLS_CMD_ERASE_APP:
            erase_app();
            ble_bls_response_send(p_bls, BLE_BLS_RESPONSE_SUCCESS);
            break;

        case BLE_BLS_CMD_WRITE_LINE:
            result = write_line(p_bls_evt);
            ble_bls_response_send(p_bls, result);
            break;

        case BLE_BLS_CMD_RESET_AND_RUN:
            ble_bls_response_send(p_bls, BLE_BLS_RESPONSE_SUCCESS);
            NVIC_SystemReset();
            break;

        case BLE_BLS_CMD_NOP:
            ble_bls_response_send(p_bls, BLE_BLS_RESPONSE_SUCCESS);
            break;
    }
}

/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    // YOUR_JOB: Add code to initialize the services used by the application.
    uint32_t err_code;
    ble_bls_init_t init;

    memset(&init, 0, sizeof(init));
    init.evt_handler = ble_bls_evt_handler;
    
    err_code = ble_bls_init(&m_bls, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}



/**@brief Start timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;
    
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            if (err_code == NRF_SUCCESS)
            {
                advertising_start();
            }
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
                err_code = sd_power_system_off();    
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_bls_on_ble_evt(&m_bls, p_ble_evt);
    /* 
    YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    */
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}





/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{

    // Note: If the only use of buttons is to wake up, the app_button module can be omitted, and
    //       the wakeup button can be configured by
    // GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    // Initialize
    leds_init();
    timers_init();
    buttons_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    sec_params_init();
    
    // Start execution
    timers_start();
    advertising_start();
    
    // Enter main loop
    for (;;)
    {
        power_manage();
    }
}

/** 
 * @}
 */

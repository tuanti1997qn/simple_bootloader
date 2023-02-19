#ifndef _XMODEM_
#define _XMODEM_

#include "stdint.h"

typedef enum 
{
    XMODEM_OK,
    XMODEM_FAIL,
    XMODEM_NULL_PTR,
    XMODEM_CRC_WRONG
}   xmodem_status_t;

typedef xmodem_status_t (*xmodem_receive_done_cb_t) (uint8_t *data, uint32_t data_length, uint8_t package_count);
typedef xmodem_status_t (*xmodem_send_data_t) (uint8_t* data, uint32_t length);
typedef xmodem_status_t (*xmodem_receive_data_t) (uint8_t* data, uint32_t length);
typedef uint32_t (*xmodem_custom_crc_calculator_t) (uint8_t *data, uint32_t length);

typedef struct 
{
    xmodem_receive_done_cb_t xmodem_receive_done_cb;        /* callback function called when xmodem process receive done */
    xmodem_send_data_t xmodem_send_data;                    
    xmodem_receive_data_t xmodem_receive_data;
    xmodem_custom_crc_calculator_t xmodem_custom_crc_calculator;
    uint32_t data_byte_receive;                             /* number of byte received*/


} xmodem_t;

xmodem_status_t process_receive_data(xmodem_t *xmodem, uint8_t data);
xmodem_status_t init_xmodem(xmodem_t *xmodem);
xmodem_status_t start_receive_data(xmodem_t *xmodem);

#endif
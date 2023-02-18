#include "xmodem.h"
#define NULL 0

const uint8_t SOH = 0x01;   //  Start of the Header	
const uint8_t EOT = 0x04;   //  End of Transmission
const uint8_t ACK = 0x06;   //  Acknowledge
const uint8_t NAK = 0x15;   //  Not Acknowledge
const uint8_t ETB = 0x17;   //  End of Transmission Block	
const uint8_t CAN = 0x18;   //  Cancel
const uint8_t C   = 0x43;   //  ASCII “C”

uint8_t _receive_xmodem_process() {};

typedef enum {
    START_OF_HEADER,
    COUNT,
    COUNT_REVERT,
    DATA,
    CRC
} state_state_t;

uint8_t receive_data[128];  // 128 byte data 
uint8_t crc[2];             // 2 byte crc

xmodem_status_t init_xmodem(xmodem_t *xmodem) {
    xmodem->data_byte_receive = 0;
    if(xmodem->xmodem_receive_done_cb == NULL) return XMODEM_NULL_PTR;
    if(xmodem->xmodem_send_data == NULL) return XMODEM_NULL_PTR;
    return XMODEM_OK;
}

xmodem_status_t start_receive_data(xmodem_t *xmodem) {
    uint8_t buff = C;
    xmodem->xmodem_send_data(&buff, 1);
}

uint32_t calcrc(char *ptr, int count)
{
    int  crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

/*TODO: handle fail code, exception, this just a simple function that can work, i will update it later*/ 
xmodem_status_t process_receive_data(xmodem_t *xmodem, uint8_t data) {
    uint8_t temp_buff;
    while (1) {
        temp_buff = C;
        xmodem->xmodem_send_data(&temp_buff,1);
        xmodem->xmodem_receive_data(&temp_buff, 1);
        if(temp_buff == SOH || temp_buff == EOT || temp_buff == ETB) break;
    }

    while (1) {
        // proccess byte count
        uint8_t byte_count[2];
        xmodem->xmodem_receive_data(byte_count, 2);
        if (byte_count[0] != (0xff - byte_count[1])) {
            uint8_t send_buff = NAK;
            xmodem->xmodem_send_data(&send_buff, 1);
            goto REPEAT;
        }

        // process get data
        xmodem->xmodem_receive_data(receive_data , 128);

        // calculate crc
        uint8_t crc[2];
        xmodem->xmodem_receive_data(crc,2);
        uint32_t crc_result;
        if (xmodem->xmodem_custom_crc_calculator == NULL) {
            crc_result =  (xmodem->xmodem_custom_crc_calculator(receive_data, 128));
        } else {
            crc_result = calcrc(receive_data, 128); 
        }
        uint8_t crc_temp[2];

        
        crc_temp[0] = crc_result >>8 & 0xFF;
        crc_temp[1] = crc_result & 0xFF;
        if (crc_temp[0] != crc[0] || crc_temp[1] != crc[1]) {       
            uint8_t send_buff = NAK;
            xmodem->xmodem_send_data(&send_buff, 1);
            
            xmodem->xmodem_send_data(&crc_temp[0],1);
            xmodem->xmodem_send_data(&crc_temp[1],1);
            xmodem->xmodem_send_data(&crc[0],1);
            xmodem->xmodem_send_data(&crc[1],1);
            goto REPEAT;
        }

        xmodem->xmodem_receive_done_cb(receive_data, byte_count[0]);
        uint8_t send_buff = ACK;
        xmodem->xmodem_send_data(&send_buff, 1);

 REPEAT: xmodem->xmodem_receive_data(&temp_buff,1);
        if (temp_buff == SOH) {       /* EOT*/
            continue;
        }
        if (temp_buff == EOT) {       /* EOT*/
            temp_buff = ACK;
            xmodem->xmodem_send_data(&temp_buff,1);
            break;
        }
    }

    // xmodem->xmodem_send_data(&temp_buff,1);
    // if (temp_buff == ETB) {       /* ETB*/
    //     temp_buff = ACK;
    //     xmodem->xmodem_send_data(&temp_buff,1);
    //     return XMODEM_OK;
    // }
    return XMODEM_OK;
}




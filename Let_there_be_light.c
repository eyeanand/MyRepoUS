/*
 * Copyright 2014, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *
 * Scan Application
 *
 * Features demonstrated
 *  - WICED scan API
 *
 * This application snippet regularly scans for nearby Wi-Fi access points
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Each time the application scans, a list of Wi-Fi access points in
 *   range is printed to the UART
 *
 */

#include <stdlib.h>
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define LIGHT_ENABLE_CIELAB
#define LIGHT_DIMMABLE
#define RX_BUFFER_SIZE    64
#define TEST_STR          "\r\nSend the brightness value you want to set (uint8_t) between 0 to 100...\r\n"
#define TCP_SERVER_LISTEN_PORT              (1234)
#define TCP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
#define TCP_SERVER_STACK_SIZE               (5500)
#define TCP_PACKET_ACK_DATA_LENGTH          (4)
#define LIGHT_ID                            13
#define LIGHT_BROADCAST_ID                  0
/******************************************************
 *                    Constants
 ******************************************************/
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
	*                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void link_up  ( void );
static void link_down( void );
void light_init();
void light_control(uint8_t);
static void tcp_server_thread(uint32_t);
/******************************************************
 *               Variable Definitions
 ******************************************************/
/* CIELAB1931
 * L* = 116 * (Y/Yn)1/3 - 16    for Y/Yn > 0.008856
 * L* = 903.3 * Y/Yn             otherwise
 */
#ifdef LIGHT_ENABLE_CIELAB
static float CIELAB[] = {0,
        0.110705192,
        0.221410384,
        0.332115576,
        0.442820768,
        0.55352596,
        0.664231152,
        0.774936345,
        0.885645168,
        1.001027615,
        1.126019927,
        1.261006499,
        1.406371725,
        1.5625,
        1.729775719,
        1.908583275,
        2.099307065,
        2.302331481,
        2.51804092,
        2.746819775,
        2.989052442,
        3.245123314,
        3.515416786,
        3.800317254,
        4.100209111,
        4.415476752,
        4.746504572,
        5.093676965,
        5.457378326,
        5.83799305,
        6.235905531,
        6.651500164,
        7.085161343,
        7.537273463,
        8.008220919,
        8.498388105,
        9.008159416,
        9.537919246,
        10.08805199,
        10.65894204,
        11.2509738,
        11.86453165,
        12.5,
        13.15776323,
        13.83820575,
        14.54171194,
        15.2686662,
        16.01945293,
        16.79445652,
        17.59406136,
        18.41865185,
        19.26861239,
        20.14432736,
        21.04618117,
        21.9745582,
        22.92984286,
        23.91241953,
        24.92267262,
        25.96098651,
        27.0277456,
        28.12333429,
        29.24813697,
        30.40253803,
        31.58692187,
        32.80167289,
        34.04717547,
        35.32381401,
        36.63197292,
        37.97203657,
        39.34438938,
        40.74941572,
        42.1875,
        43.65902661,
        45.16437995,
        46.7039444,
        48.27810437,
        49.88724425,
        51.53174843,
        53.21200131,
        54.92838729,
        56.68129075,
        58.47109609,
        60.29818771,
        62.16295,
        64.06576735,
        66.00702417,
        67.98710484,
        70.00639376,
        72.06527533,
        74.16413393,
        76.30335397,
        78.48331984,
        80.70441593,
        82.96702663,
        85.27153635,
        87.61832947,
        90.0077904,
        92.44030352,
        94.91625323,
        97.43602392,
        100
};
#endif
wiced_uart_config_t uart_config =
{
        .baud_rate    = 115200,
        .data_width   = DATA_WIDTH_8BIT,
        .parity       = NO_PARITY,
        .stop_bits    = STOP_BITS_1,
        .flow_control = FLOW_CONTROL_DISABLED,
};

wiced_ring_buffer_t rx_buffer;
uint8_t             rx_data[RX_BUFFER_SIZE];

uint8_t prev_value = 100;

wiced_tcp_socket_t socket;
static wiced_thread_t      tcp_thread;
wiced_bool_t quit = WICED_FALSE;

typedef struct {
    uint32_t id;
    uint32_t cmd;
    uint8_t on;
    uint8_t brightness;
    uint8_t group;
}light_payload_t;
/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( )
{
    //char c;
    /* Initialise the device */
    wiced_init();
    /* Initialise GPIO and PWM and turn ON the light */
    light_init();

    wiced_gpio_init(WICED_GPIO_2,OUTPUT_PUSH_PULL);

    /* Register callbacks */
    wiced_network_register_link_callback( link_up, link_down );

    /* Bring up the STA (client) interface ------------------------------------------------------- */
    while(wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS);
    link_up();

}

void light_init()
{
    wiced_gpio_init(WICED_GPIO_3,OUTPUT_PUSH_PULL);
    wiced_gpio_output_high(WICED_GPIO_3);
#ifdef LIGHT_DIMMABLE
    if(wiced_pwm_init(WICED_PWM_5,20000,100.0) == WICED_ERROR) {
        WPRINT_WICED_INFO(("error setting PWM...\n"));
    }
    wiced_pwm_start(WICED_PWM_5);
#endif
}

void light_control(uint8_t value)
{
    if(value > 100) {
        value = 100;
    }
    prev_value = value;
    if(value == 0) {
        wiced_gpio_output_low(WICED_GPIO_3);
#ifdef LIGHT_DIMMABLE
        wiced_pwm_stop(WICED_PWM_5);
#endif
    } else {
        wiced_gpio_output_high(WICED_GPIO_3);
#ifdef LIGHT_DIMMABLE
#ifdef LIGHT_ENABLE_CIELAB
        wiced_pwm_init(WICED_PWM_5,20000,CIELAB[value]);
#else
        wiced_pwm_init(WICED_PWM_5,20000,(float)value;
#endif
        wiced_pwm_start(WICED_PWM_5);
#endif
    }
}

static void link_up( void )
{
    WPRINT_APP_INFO( ("In link_up callback...\n") );
    wiced_gpio_output_high(WICED_GPIO_2);

    while(wiced_network_is_ip_up(WICED_STA_INTERFACE) == WICED_TRUE) {
        /* Create a TCP server socket */
        if(wiced_tcp_create_socket(&socket, WICED_STA_INTERFACE) == WICED_SUCCESS) {
            if(wiced_tcp_listen(&socket, TCP_SERVER_LISTEN_PORT ) == WICED_SUCCESS) {
                WPRINT_APP_INFO(("Creating tcp server thread \n"));
                quit = WICED_FALSE;
                wiced_rtos_create_thread(&tcp_thread, TCP_SERVER_THREAD_PRIORITY, "tcp_server", tcp_server_thread, TCP_SERVER_STACK_SIZE, &socket);
                break;
            }
        } else {
            wiced_tcp_delete_socket(&socket);
        }
        wiced_rtos_delay_milliseconds(100);
    }
}


static void link_down( void )
{
    WPRINT_APP_INFO( ("In link_down callback...\n") );
    wiced_gpio_output_low(WICED_GPIO_2);
    light_control(100);

    quit = WICED_TRUE;
    wiced_rtos_thread_join(&tcp_thread);
    wiced_tcp_delete_socket(&socket);

    while(wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS) {
        wiced_rtos_delay_milliseconds(100);
    }
}

static void tcp_server_thread(uint32_t arg)
{
    wiced_tcp_socket_t* sock = (wiced_tcp_socket_t *) arg;
    uint16_t        request_length;
    uint16_t        available_data_length;
    light_payload_t *payload;
    wiced_packet_t* tx_packet = NULL;
    wiced_packet_t* rx_packet = NULL;
    uint32_t *tx_data;
    while(quit == WICED_FALSE) {
        while (wiced_tcp_accept(sock) == WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Accepted a connection\n"));
            light_control(prev_value);
            while(quit == WICED_FALSE) {
                /* Receive the query from the TCP client */
                wiced_result_t ret;
                ret = wiced_tcp_receive( sock, &rx_packet, 1000 );
                if ( ret == WICED_SUCCESS)
                {
                    WPRINT_APP_INFO(("Received Packet\n"));
                    /* Process the client request */
                    wiced_packet_get_data(rx_packet, 0, (uint8_t**) &payload, &request_length, &available_data_length );

                    if(request_length == sizeof(light_payload_t)) {
                        //WPRINT_APP_INFO(("payload: id:%d, cmd:%d, on:%d\n, brightness:%d, group:%d\n",payload->id,payload->cmd,payload->on,payload->brightness,payload->group));
                        if((payload->id == LIGHT_ID)||(payload->id == LIGHT_BROADCAST_ID)) {
                            switch (payload->cmd) {
                                case 10:
                                    light_control(payload->brightness);
                                    /* Send Ack back */
                                    if (wiced_packet_create_tcp(sock, TCP_PACKET_ACK_DATA_LENGTH, &tx_packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
                                    {
                                        WPRINT_APP_INFO(("TCP packet creation failed\n"));
                                    }

                                    *tx_data = 0xffffffff;

                                    /* Set the end of the data portion */
                                    wiced_packet_set_data_end(tx_packet, (uint8_t*)tx_data + TCP_PACKET_ACK_DATA_LENGTH);

                                    /* Send the TCP packet */
                                    if (wiced_tcp_send_packet(sock, tx_packet) != WICED_SUCCESS)
                                    {
                                        WPRINT_APP_INFO(("TCP packet send failed\n"));

                                        /* Delete packet, since the send failed */
                                        wiced_packet_delete(tx_packet);
                                    }
                                    break;
                                case 11:
                                    if(payload->id == LIGHT_ID) {
                                        /* Send Ack back */
                                        if (wiced_packet_create_tcp(sock, TCP_PACKET_ACK_DATA_LENGTH, &tx_packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
                                        {
                                            WPRINT_APP_INFO(("TCP packet creation failed\n"));
                                        }

                                        *tx_data = (uint32_t) prev_value;

                                        /* Set the end of the data portion */
                                        wiced_packet_set_data_end(tx_packet, (uint8_t*)tx_data + TCP_PACKET_ACK_DATA_LENGTH);

                                        /* Send the TCP packet */
                                        if (wiced_tcp_send_packet(sock, tx_packet) != WICED_SUCCESS)
                                        {
                                            WPRINT_APP_INFO(("TCP packet send failed\n"));

                                            /* Delete packet, since the send failed */
                                            wiced_packet_delete(tx_packet);
                                        }
                                    }
                                    break;
                                default:
                                    /* Send Ack back */
                                    if (wiced_packet_create_tcp(sock, TCP_PACKET_ACK_DATA_LENGTH, &tx_packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
                                    {
                                        WPRINT_APP_INFO(("TCP packet creation failed\n"));
                                    }

                                    *tx_data = 0xfffffffe;

                                    /* Set the end of the data portion */
                                    wiced_packet_set_data_end(tx_packet, (uint8_t*)tx_data + TCP_PACKET_ACK_DATA_LENGTH);

                                    /* Send the TCP packet */
                                    if (wiced_tcp_send_packet(sock, tx_packet) != WICED_SUCCESS)
                                    {
                                        WPRINT_APP_INFO(("TCP packet send failed\n"));

                                        /* Delete packet, since the send failed */
                                        wiced_packet_delete(tx_packet);
                                    }
                                    break;
                            }
                            //light_control(payload->brightness);
                        }
                    } else {
                        WPRINT_APP_INFO(("Error in packet receive...\n"));
                    }
                    /* Delete the packet, we're done with it */
                    wiced_packet_delete( rx_packet );

                } else if(ret != 2){
                    break;
                }
            }
        }
    }
    WPRINT_APP_INFO(("Disconnect\n"));

    wiced_tcp_disconnect( sock );

    WICED_END_OF_CURRENT_THREAD();
}

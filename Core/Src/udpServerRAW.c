/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpClientRAW.c
  Author:     ControllersTech.com
  Updated:    Jul 23, 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "string.h"
#include <stdint.h>

#include "udpServerRAW.h"


void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

volatile float delays[16];
volatile uint8_t packet_is_delays;

struct udp_pcb *upcb;
char buffer[100];
int cnt = 0;


// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if(htim->Instance == TIM3)
//   {
//     udpClient_send();
//   }
// }


/* IMPLEMENTATION FOR UDP CLIENT :   source:https://www.geeksforgeeks.org/udp-server-client-implementation-c/

1. Create UDP socket.
2. Send message to server.
3. Wait until response from server is received.
4. Process reply and go back to step 2, if necessary.
5. Close socket descriptor and exit.
*/


int parse_buffer(const char *buffer, size_t buffer_len, volatile float out[16])
{
    if (buffer_len < 1 + 16 * sizeof(float))
        return -1;

    if (buffer[0] != 0x67)
        return -1;

    for (int i = 0; i < 16; i++) {
        memcpy(&out[i], &buffer[1 + i * sizeof(float)], sizeof(float));
    }

    return 0;
}

void udpClient_connect(void)
{
	err_t err;

	/* 1. Create a new UDP control block  */
	upcb = udp_new();

	/* Bind the block to module's IP and port */
	ip_addr_t myIPaddr;
	IP_ADDR4(&myIPaddr, 192, 168, 0, 123);
	udp_bind(upcb, &myIPaddr, 7777);


	/* configure destination IP address and port */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 0, 2);
	err= udp_connect(upcb, &DestIPaddr, 6666);

	if (err == ERR_OK)
	{
		/* 2. Send message to server */

		/* 3. Set a receive callback for the upcb */
		udp_recv(upcb, udp_receive_callback, NULL);
	}
}

 void udpClient_send(int16_t* data, uint16_t len)
{
  struct pbuf *txBuf;
  len = len * sizeof(uint16_t);


  /* allocate pbuf from pool*/
  txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

  if (txBuf != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(txBuf, data, len);

    /* send udp data */
    udp_send(upcb, txBuf);

    /* free pbuf */
    pbuf_free(txBuf);
  }
}


void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	/* Copy the data from the pbuf */
	strncpy(buffer, (char *)p->payload, p->len);

	/*increment message count */
	cnt++;
  if(buffer[0] == 0x67){
    packet_is_delays = 1;
  }
	/* Free receive pbuf */
	pbuf_free(p);
}

int8_t check_for_tx(float* delays){
  if(!packet_is_delays) return -1;
  packet_is_delays = 0;

  parse_buffer(buffer, sizeof(buffer), delays);
}


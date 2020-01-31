/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2017 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes
   * Angel Corona

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 3: Gateway
*/

// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h"			// Use LEDs.
#include "sys/clock.h"			// Use CLOCK_SECOND.
#include "net/rime/rime.h"		// Establish connections.
#include "dev/cc2538-rf.h"
#include "dev/adc-zoul.h"      	// ADC
#include "dev/zoul-sensors.h"  	// Sensor functions
// Standard C includes:
#include <stdio.h>				// For printf.
#include "sys/etimer.h"
#include "sys/ctimer.h"

#include "dev/cc2538-rf.h"
#include "lib/random.h"

#define MAX_NO_OF_MOTES	6
/*-----------------------------FUNCTION PROTOTYPES--------------------------------_*/
/*--------------------------------------------------------------------------------_*/

/*! Broadcast connection setup */
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn broadcastConn;
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

/*! Unicast connection setup */
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn unicast;
static const struct unicast_callbacks unicast_call = {unicast_recv};


/*--------------------------CTIMER DECLARATIONS-----------------------------------_*/
/*--------------------------------------------------------------------------------_*/

static void callback_off(void *ptr);							// Call when ALL LEDs are to be turned OFF
static void callback_vibration(void *ptr);						// Call each second to sense vibrations
static void callback_array_processing(void *ptr);				// Call every 60s to run detecting algorithm

static struct ctimer ctimer_array_processing;					// Used for 60s delay for detecting algorithm
static struct ctimer ctimer_vibration_sensing;					// Used for 1s delay for sensing vibrations
static struct ctimer ctimer_vibration_LED;						// Used for blinking LED for 1s when vibrations are detected on gateway
static struct ctimer ctimer_unicast_LED;						// Used for blinking LED for 1s when unicast packet is received


/*----------------------------DEFINITIONS OF VARIABLES----------------------------_*/
/*--------------------------------------------------------------------------------_*/

/*! Unicast Packet, Vibration value is not important but can be useful if more detailed algorithm is to be run on gateway */
typedef struct  {
	uint8_t source_id;
	uint16_t vibration_value;
}packet_t;

/*! Look-up table/ Broadcast packet */
typedef struct
{
	linkaddr_t 	next_hop;			/* Next hop in route to destination */
	uint16_t 	cost;
	uint16_t 	battery;
}l_table;

static l_table lut =
{
	.next_hop.u8[1] = 0x05, .cost= 0, .battery=80,
};

/* Stores the binary information if a certain mote has sensed the vibrations or not */
uint8_t vibration_array[MAX_NO_OF_MOTES] = {0};	/*TODO: Change into bool */


/*----------------------------PACKET RECEIVE FUNCTIONS----------------------------_*/
/*--------------------------------------------------------------------------------_*/

/* Nothing to be done in case of broadcast_recv */
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	 //printf("Broadcast message received from 0x%x%x: '%s' [RSSI %d]\n",from->u8[0], from->u8[1],(char *)packetbuf_dataptr(),(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));
}

/* Unicast packet is saved, Source ID which initiated the packet and vibration value are parsed and saved. */
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{
	packet_t rx_packet;
	packetbuf_copyto(&rx_packet);
	printf("Unicast message received from 0x%x%x, [RSSI: %d], Source ID: '%d',Vibration Value : %d\n",from->u8[0], from->u8[1],(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI), rx_packet.source_id,rx_packet.vibration_value);
	leds_on(LEDS_BLUE);
	ctimer_set(&ctimer_unicast_LED, CLOCK_SECOND, callback_off, NULL);
	rx_packet.source_id--;
	vibration_array[rx_packet.source_id] = 1;
}


/*---------------------------PROCESS CONTROL BLOCK--------------------------------_*/
/*--------------------------------------------------------------------------------_*/

PROCESS(gateway_main_process, "GATEWAY MOTE");
AUTOSTART_PROCESSES(&gateway_main_process);

PROCESS_THREAD(gateway_main_process, ev, data)
{
	static struct etimer etimer_broadcast;								/* For 10s delay in broadcasting the LUT */
	PROCESS_EXITHANDLER( broadcast_close(&broadcastConn); unicast_close(&unicast); )
	PROCESS_BEGIN();

	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,  16);			/* Group No: 6 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER, -24);			/* Setting minimum power to limit the range to emulate multi-hops */
	adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1);

	broadcast_open(&broadcastConn, 125, &broadcast_callbacks);
	unicast_open(&unicast, 129, &unicast_call);

	ctimer_set(&ctimer_array_processing, CLOCK_SECOND*60, callback_array_processing, NULL);		/* Detecting algorithm is done every 60 seconds */
	etimer_set(&etimer_broadcast, CLOCK_SECOND*10+ 0.1*random_rand()/RANDOM_RAND_MAX);			/* Broadcast is done every 10 seconds */
	ctimer_set(&ctimer_vibration_sensing, CLOCK_SECOND, callback_vibration, NULL);				/* Vibrations are sensed every second */

	while(1)
	{
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etimer_broadcast));

		packetbuf_copyfrom(&lut, sizeof(l_table));
		broadcast_send(&broadcastConn);
    	etimer_reset(&etimer_broadcast);
	}

	PROCESS_END();
}

/*-------------------------FUNCTION DEFINITIONS-----------------------------------_*/
/*--------------------------------------------------------------------------------_*/

/*-----------------------BREAKAGE DETECTION ALGORITHM-----------------------------_*/

static void callback_array_processing(void *ptr)
{
	uint8_t sum = 0;
	uint8_t health_status_array[MAX_NO_OF_MOTES - 2] = {0};				/* Binary array to store health status for each section of the track */
	printf("\nUpdating the status of the track. \nProcessing started:");

/*--------------------------------------------------------------------------------_*/

	printf("\nClearing Track ID Status");								/* For Qt Display */

/*--------------------------------------------------------------------------------_*/
	printf("\nPrinting Vibration Array:\n");
	for(int i = 0; i < MAX_NO_OF_MOTES; i++)
    {
    	printf("\nMoteID = %d   Vibration = %d",i+1, vibration_array[i]);
    	sum = sum + vibration_array[i];									/* If any mote senses vibration, sum > 0 and train arrival is detected */
    }

	printf("\n");

	if(sum > 0)
	{
		printf("\nTrain Arrival Detected = %d\n",1);					/* For Qt Display */
	}

	else
	{
		printf("\nTrain Arrival Detected = 0\n");						/* For Qt Display */
	}


/*--------------Till now, train arrival has been detected!------------------------_*/
/*--------------------------------------------------------------------------------_*/

	for(int i=0; i<MAX_NO_OF_MOTES - 2; i++)
	{
		if(vibration_array[i] - vibration_array[i+2] != 0)				/* Comparing vibrations of 2 consecutive motes to detect breakage */
		{
			printf("\nBreakage Detected!\n");
			printf("\nFaulted Track ID = %d\n", i+2);					/* For Qt Display */
			health_status_array[i] = 1;	/* Index 0 means Track ID 2 */
		}
	}

/*------------------Till Now the breakage has been detected----------------------_*/
/*-------------------------------------------------------------------------------_*/

	printf("\nUpdating the status of the sections:");

	for(int i = 0; i < MAX_NO_OF_MOTES - 2; i++)
	{
		printf("\nTrack ID = %d   Health Status = %s",i+2, (health_status_array[i] == 1) ? "FAULTY" : "HEALTHY");
	}

	printf("\n");
/*------Now we have identified exactly which section of the track is broken.-----_*/
/*-------------------------------------------------------------------------------_*/

	printf("\nProcessing Completed. \n");

	for(int j = 0; j<MAX_NO_OF_MOTES; j++)
	{
		vibration_array[j] = 0;			/* Clearing the vibration array */
	}

	printf("\nArray has been re-initialized.\n");
	ctimer_reset(&ctimer_array_processing);
}

/*--------------------------------------------------------------------------------_*/

static void callback_off(void *ptr)
{
	leds_off(LEDS_ALL);						/* A callback function to switch all LEDs OFF */
}

/*--------------------------------------------------------------------------------_*/

static void callback_vibration(void *ptr)
{
	uint16_t adc1_value = 1000;
	static uint16_t avg_adc1_value = 1000;

	adc1_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;
	avg_adc1_value = (avg_adc1_value + adc1_value) / 2;

	if(avg_adc1_value >1200 || avg_adc1_value< 900 )
	{
		printf("\nVibration Detected");
	    vibration_array[MAX_NO_OF_MOTES-1] = 1;						/* If gateway has sensed vibrations, vibration_value is set to true for gateway */
	    leds_on(LEDS_YELLOW);
	    ctimer_set(&ctimer_vibration_LED, CLOCK_SECOND, callback_off, NULL);
	}
	ctimer_reset(&ctimer_vibration_sensing);

}
/*--------------------------------------------------------------------------------_*/
/*--------------------------------------------------------------------------------_*/

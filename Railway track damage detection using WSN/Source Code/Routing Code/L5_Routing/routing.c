/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2018 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes
   * Angel Corona
   * Donjeta Elshani
   * Onur Ayan

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 5: Routing
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"     // Establish connections.
#include "net/netstack.h"      // Wireless-stack definitions
#include "dev/button-sensor.h" // User Button
#include "dev/leds.h"          // Use LEDs.
#include "core/net/linkaddr.h"
#include "dev/adc-zoul.h"      // ADC
#include "dev/zoul-sensors.h"  // Sensor functions
// Standard C includes:
#include <stdio.h>
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "dev/cc2538-rf.h"
#include <math.h>
#include "lib/random.h"
#include "sys/clock.h"
#include "dev/button-sensor.h"
#include <stdbool.h>
#include "project-conf.h"

/*---------------------------------------------------------------------------------*/

/*--------------------------TIMER DECLARATIONS-------------------------------------*/
/*---------------------------------------------------------------------------------*/

static void callback_broadcast(void *ptr);
static void callback_sensor(void *ptr);
static void callback_LUT_reset(void *ptr);
static void callback_off(void *ptr);

static struct ctimer timer_broadcast;				// To broadcast LUT every 10 s
static struct ctimer timer_sensor;					// To sense vibrations every 5 seconds
static struct ctimer timer_LUT_reset;				// To avoid faulty motes, LUT is reset every 2 minutes
static struct ctimer ctimer_unicast_LED, ctimer_vibration_detected_LED;						// For LED blinking

/*--------------------------------------------------------------------------------_*/
/*--------------------------------------------------------------------------------_*/

static uint8_t node_address;
int16_t	RSSI_array[MAX_NO_OF_MOTES] = {-50};			/*Initialize with average RSSI value */

/*--------------------------------------------------------------------------------_*/
typedef struct
{
	linkaddr_t 	next_hop;			/* Next hop in route to destination. */
	uint16_t 	cost;
	uint16_t 	battery;
}l_table;

/*--------------------------------------------------------------------------------_*/
static l_table lut =
{
	.next_hop.u8[1] = 0x08, .cost= 10000, .battery=100,
};

/*--------------------------------------------------------------------------------_*/
typedef struct
{
	uint8_t source_id;
	uint16_t vibration_value;
}packet_t;

l_table receive_message;
packet_t tx_packet;

/*---------------------PACKET RECEIVE FUNCTIONS DECLARATION-----------------------_*/
/*--------------------------------------------------------------------------------_*/

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn broadcastConn;
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn unicast;
static const struct unicast_callbacks unicast_call = {unicast_recv};

/*------------------PACKET RECEIVE FUMCTIONS DEFINITIONS--------------------------_*/
/*--------------------------------------------------------------------------------_*/

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{
	packet_t local_unicast_msg;
	printf("\nUnicast message received from 0x%x%x: [RSSI %d]\n",from->u8[0], from->u8[1],(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

	packetbuf_copyto(&local_unicast_msg);

	printf("\nPacket forwarding to 0x%x with source ID: %d and vibration value: %d", lut.next_hop, local_unicast_msg.source_id, local_unicast_msg.vibration_value);
	unicast_send(&unicast, &lut.next_hop);
	printf("\nPacket Forwarded");

	leds_on(LEDS_GREEN);
	ctimer_set(&ctimer_unicast_LED, CLOCK_SECOND, callback_off, NULL);
}

/*--------------------------------------------------------------------------------_*/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	int16_t local_cost = 0, total_cost 			= 0;
	int16_t received_RSSI 						= 0;
	bool LUT_update;

	int addr = from->u8[1];


	received_RSSI =(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);

	RSSI_array[(from->u8[1]) - 1] = (RSSI_array[(from->u8[1])-1] + received_RSSI) / 2;			/* Moving Average Filter for RSSI */

	if(((node_address % 2 == 1) && (addr < (node_address + 3))) || (node_address % 2 == 0 && addr <= (node_address + 3)))
	{
		LUT_update = 1;
	}

	else
	{
		LUT_update = 0;
	}

	if(LUT_update == 1)
	{
		printf("\nBroadcast message received from 0x%x%x: [RSSI %d]\n", from->u8[0], from->u8[1], (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

		packetbuf_copyto(&receive_message);
		printf("\nCost Received: %d\tBattery Value Received: %d", receive_message.cost, receive_message.battery);

		local_cost = (MAX_RSSI - (RSSI_array[(from->u8[1])-1])) + (100 - receive_message.battery);		/* Formula to calculate the cost, minimum cost means better route */

		printf("\n\nCalculated Local cost: %d", local_cost);
		printf("\nCost before updating: %d\tNext hop before updating: 0x%x%x", lut.cost, lut.next_hop.u8[0], lut.next_hop.u8[1]);

		total_cost = receive_message.cost + local_cost;			/* Calculating cost for the complete path to the destination  */
		printf("\n\nTotal cost: %d", total_cost);

		if(total_cost <= lut.cost)								/* Update next hop only when the calculated cost < current saved cost in LUT */
		{
			lut.next_hop.u8[1] = from->u8[1];					/* Updating next hop */
			lut.cost = total_cost;								/* Updating cost */
			printf("\n\n\nCost updated to: %d,\tNext hop updated to: 0x%x%x", lut.cost, lut.next_hop.u8[0], lut.next_hop.u8[1]);
			//leds_on(LEDS_YELLOW);
			//ctimer_set(&ctimer_LUT_update_LED, CLOCK_SECOND, callback_off, NULL);
		}

		else
		{
			if(lut.next_hop.u8[1] == from->u8[1])				/* If received from the mote which was the next hop, cost must still be updated */
			{
				lut.cost = total_cost;
				printf("\nCost Updated.\n");
			}
			printf("\nNext Hop not updated.\n");
		}
	}
}
/*--------------------------------------------------------------------------------_*/


static bool flag = false, flag1 = false;		/* Just for programming logic */

/*----------------------------PROCESS CONTROL BLOCK-------------------------------_*/
/*--------------------------------------------------------------------------------_*/

PROCESS(code_for_field_motes, "CODE FOR FIELD MOTES");
AUTOSTART_PROCESSES(&code_for_field_motes);

PROCESS_THREAD(code_for_field_motes, ev, data) {

	PROCESS_EXITHANDLER(broadcast_close(&broadcastConn); unicast_close(&unicast))
	PROCESS_BEGIN();

	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,  CHANNEL);
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER, TX_POWER);

	button_sensor.configure(BUTTON_SENSOR_CONFIG_TYPE_INTERVAL, CLOCK_SECOND/2);
	adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1);

	unicast_open(&unicast, 129, &unicast_call);
	broadcast_open(&broadcastConn, 125, &broadcast_callbacks);

	ctimer_set(&timer_broadcast, CLOCK_SECOND*10+ 0.1*random_rand()/RANDOM_RAND_MAX, callback_broadcast, NULL);
	ctimer_set(&timer_sensor, CLOCK_SECOND*5+ 0.1*random_rand()/RANDOM_RAND_MAX, callback_sensor, NULL);
	ctimer_set(&timer_LUT_reset, CLOCK_SECOND*120, callback_LUT_reset, NULL);

	node_address=(linkaddr_node_addr.u8[1] & 0xFF);

	while(1)
	{
		PROCESS_WAIT_EVENT();

		if(ev == sensors_event)		/* Only to simluate to show change in route due to low battery */
	    {
	    	if(data == &button_sensor)	 /* Event from the User button */
	    	{
	    		if(button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) == BUTTON_SENSOR_PRESSED_LEVEL)	/* Button was pressed */
	    		{
	    			if(flag == false)
	    			{
	    				printf("\nButton pressed, batterey is set to zero!");
	    				flag = true;
	    				leds_on(LEDS_RED);
	    				flag1 = true;
	    			}
	    			else
	    			{
		    			printf("\nButton pressed again, batterey is set to original value!");
		    			flag = false;
		    			flag1 = false;
	    				leds_off(LEDS_RED);
	    			}
	    		}
	    	}
	    }
	}

	PROCESS_END();
}

/*------------------------------CALLBACK FUNCTIONS--------------------------------_*/
/*--------------------------------------------------------------------------------_*/
static void callback_broadcast(void *ptr)	/* Periodic broadcasting LUT */
{
	if(flag1 == false)			/* Programming logic to simulate change in route due to low battery */
	{
		lut.battery= (vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED))/40;
	}

	else if (flag1 == true && flag == true)
	{
		lut.battery = 0;
	}

	else if (flag1 == true && flag == false)
	{
		lut.battery= (vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED))/40;
	}

	packetbuf_copyfrom(&lut, sizeof(l_table));
	broadcast_send(&broadcastConn);

	printf("\n\nLUT broadcasted: \nNext Hop: 0x%x%x\nCost: %d\nBattery: %d.\n",lut.next_hop.u8[0],lut.next_hop.u8[1],lut.cost, lut.battery);

	ctimer_reset(&timer_broadcast);
}

/*--------------------------------------------------------------------------------_*/
static void callback_sensor(void *ptr)			/* Periodic sensing of vibrations */
{
	uint16_t adc1_value;
	adc1_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;

	if(adc1_value >1800 || adc1_value< 500 )
	{
		tx_packet.source_id = (linkaddr_node_addr.u8[1] & 0xFF);
		tx_packet.vibration_value  = adc1_value;

		printf("\nVibration detected, value: %d.\n",tx_packet.vibration_value);

		packetbuf_copyfrom(&tx_packet, sizeof(packet_t));
		unicast_send(&unicast, &lut.next_hop);
		leds_on(LEDS_BLUE);
		ctimer_set(&ctimer_vibration_detected_LED, CLOCK_SECOND*tx_packet.source_id, callback_off, NULL);
	}

	ctimer_reset(&timer_sensor);
}

/*--------------------------------------------------------------------------------_*/
static void callback_LUT_reset(void *ptr)		/* Re-initialize LUT periodically to avoid faulty motes */
{
	lut.cost = 10000;
	printf("\n\n\nLUT Re-initialized.\n");
	ctimer_reset(&timer_LUT_reset);
}

/*--------------------------------------------------------------------------------_*/
static void callback_off(void *ptr)
{
	leds_off(LEDS_ALL);
}

/*--------------------------------------------------------------------------------_*/
/*--------------------------------------------------------------------------------_*/

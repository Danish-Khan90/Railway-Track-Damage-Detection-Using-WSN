#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


// PHY LAYER PARAMETERS
#define CHANNEL 16
#define TX_POWER -24

#define MAX_RSSI -35
#define MAX_NO_OF_MOTES	6

// MAC LAYER PARAMETERS
//#define NETSTACK_CONF_MAC nullmac_driver
#define NETSTACK_CONF_MAC csma_driver

//#define NETSTACK_CONF_RDC nullrdc_driver
#define NETSTACK_CONF_RDC contikimac_driver

//#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 4



#endif /* PROJECT_CONF_H_ */

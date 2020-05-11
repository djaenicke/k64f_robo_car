#ifndef CONFIG_H_
#define CONFIG_H_

#include "io_abstraction.h"

#define OPEN_LOOP            (0)
#define CYCLE_TIME           (0.025f)
#define TUNE                 (0)

#define ROS_ENABLED          (0)

#define USE_BLUETOOTH        (0)

#define USE_XBEE             (1)

#if USE_BLUETOOTH && USE_XBEE
  #error "Bluetooth and XBee cannot be used at the same time."
#endif

/** XBee Library configuration options */
#define ENABLE_LOGGING
#define ENABLE_ASSERTIONS
#define FRAME_BUFFER_SIZE           4
#define MAX_FRAME_PAYLOAD_LEN       128
 
#define SYNC_OPS_TIMEOUT_MS         2000
 
#define RADIO_TX                XBEE_TX
#define RADIO_RX                XBEE_RX
#define RADIO_RTS               NC
#define RADIO_CTS               NC
#define RADIO_RESET             NC
#define RADIO_SLEEP_REQ         NC
#define RADIO_ON_SLEEP          NC
#define DEBUG_TX                NC
#define DEBUG_RX                NC
 
#if !defined(RADIO_TX)
    #error "Please define RADIO_TX pin"
#endif
 
#if !defined(RADIO_RX)
    #error "Please define RADIO_RX pin"
#endif
 
#if !defined(RADIO_RESET)
    #define RADIO_RESET             NC
    #warning "RADIO_RESET not defined, defaulted to 'NC'"
#endif
 
#if defined(ENABLE_LOGGING)
    #if !defined(DEBUG_TX)
        #error "Please define DEBUG_TX"
    #endif
    #if !defined(DEBUG_RX)
        #define DEBUG_RX                NC
        #warning "DEBUG_RX not defined, defaulted to 'NC'"
    #endif
#endif

#endif /* CONFIG_H_ */

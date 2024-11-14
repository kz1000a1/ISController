#ifndef __SUBARU_LEVORG_VNX_H__
#define __SUBARU_LEVORG_VNX_H__

/* #define for DEBUG_MODE */
#define no_printf_(fmt, ...)                 \
({                                           \
	if (0)                               \
		printf_(fmt, ##__VA_ARGS__); \
	0;                                   \
})

#ifdef DEBUG_MODE
#define dprintf_(fmt, ...) \
	DebugMode==DEBUG?printf_(fmt, ##__VA_ARGS__):no_printf_(fmt, ##__VA_ARGS__)
#else
#define dprintf_(fmt, ...) \
	no_printf_(fmt, ##__VA_ARGS__)
#endif

// Receive Only Two CAN Ids
#define CAN_ID_TCU         0x174
#define CAN_ID_SPEED       0x139
#define CAN_ID_AVH_STATUS  0x32B
#define CAN_ID_CCU         0x390

// Brake Pressure to Enable AVH
#define BRAKE_HIGH 60

// CCU STATUS
enum ccu_status {
    ENGINE_STOP,
    PAUSE,
    READY
};

// TCU STATUS
enum tcu_status {
    NOT_READY,
    IDLING_STOP_ON,
    IDLING_STOP_OFF
};

// STATUS
enum prog_status {
    PROCESSING,
    CANCELLED,
    FAILED,
    SUCCEEDED
};

// MODE
enum debug_mode {
    NORMAL,
    DEBUG,
    CANDUMP
};

// AVH STATUS
#define AVH_UNHOLD 0b00
#define AVH_HOLD   0b01

extern enum debug_mode DebugMode;

// for Calculate Check Sum
#define SUM_CHECK_DIVIDER 365


#define MAX_RETRY 2

#endif /* __SUBARU_LEVORG_VNX_H_ */

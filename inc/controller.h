#ifndef _CONTROLLER_H
#define _CONTROLLER_H

int8_t eliminator_parse_str(uint8_t *buf, uint8_t len);

// maximum rx buffer len: command length from USB CDC port
#define CONTROLLER_MTU 2 // (sizeof("M0"))

#endif // _CONTROLLER_H

//
// Engine auto start-stop system eliminator firmware for SUBARU Levorg VN5
//

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "led.h"
#include "system.h"
#include "error.h"
#include "printf.h"
#include "subaru_levorg_vnx.h"

#ifdef DEBUG_MODE
    enum debug_mode DebugMode = DEBUG;
#else
    enum debug_mode DebugMode = NORMAL;
#endif


void print_rx_frame(CAN_RxHeaderTypeDef* rx_msg_header, uint8_t* rx_msg_data){
    uint32_t CurrentTime;

    CurrentTime = HAL_GetTick();

    // Output all received message(s) to CDC port as candump -L
    if(rx_msg_header->RTR == CAN_RTR_DATA){ // Data Frame
        printf_("(%d.%03d000) can0 %03X#", CurrentTime / 1000,
                                           CurrentTime % 1000,
                                           rx_msg_header->StdId);
        for (uint8_t i=0; i < rx_msg_header->DLC; i++) {
            printf_("%02X", rx_msg_data[i]);
        }
        printf_("\n");
    } else { // Remote Frame
        printf_("(%d.%03d000) can0 %03X#R%d\n", CurrentTime / 1000,
                                                CurrentTime % 1000,
                                                rx_msg_header->StdId,
                                                rx_msg_header->DLC);
    }
}


void print_tx_frame(CAN_TxHeaderTypeDef* tx_msg_header, uint8_t* tx_msg_data){
    uint32_t CurrentTime;

    CurrentTime = HAL_GetTick();

    // Output all received message(s) to CDC port as candump -L
    printf_("(%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                                CurrentTime / 1000,
                                CurrentTime % 1000,
                                tx_msg_header->StdId,
                                tx_msg_data[0],
                                tx_msg_data[1],
                                tx_msg_data[2],
                                tx_msg_data[3],
                                tx_msg_data[4],
                                tx_msg_data[5],
                                tx_msg_data[6],
                                tx_msg_data[7]);
}


void send_frame(uint8_t* rx_msg_data){
    // Storage for transmit message buffer
    CAN_TxHeaderTypeDef tx_msg_header;
    tx_msg_header.IDE = CAN_ID_STD;
    tx_msg_header.StdId = CAN_ID_CCU;
    tx_msg_header.ExtId = 0;
    tx_msg_header.RTR = CAN_RTR_DATA;
    tx_msg_header.DLC = 8;
    uint8_t tx_msg_data[8] = {0};

    if ((rx_msg_data[1] & 0x0f) == 0x0f) {
        tx_msg_data[1] = rx_msg_data[1] & 0xf0;
    } else {
        tx_msg_data[1] = rx_msg_data[1] + 0x01;
    }
    tx_msg_data[2] = rx_msg_data[2];
    tx_msg_data[3] = rx_msg_data[3];
    tx_msg_data[4] = rx_msg_data[4];
    tx_msg_data[5] = rx_msg_data[5];
    tx_msg_data[6] = rx_msg_data[6] | 0x40; // Introduce / Eliminate engine auto stop bit on
    tx_msg_data[7] = rx_msg_data[7];
    // Calculate checksum
    tx_msg_data[0] = (tx_msg_data[1] +
                      tx_msg_data[2] +
                      tx_msg_data[3] +
                      tx_msg_data[4] +
                      tx_msg_data[5] +
                      tx_msg_data[6] +
                      tx_msg_data[7]) % SUM_CHECK_DIVIDER;
    can_tx(&tx_msg_header, tx_msg_data); // Queueing message
    can_process(); // Transmit message
    if(DebugMode == DEBUG){
        printf_("# ");
        print_tx_frame(&tx_msg_header, tx_msg_data);
    }
}


void led_blink(uint8_t Status){
    if(Status & 1){
        led_orange_on();
    } else {
        led_orange_off();
    }
    if(Status & 2){
        led_green_on();
    } else {
        led_green_off();
    }
}


int main(void)
{
    // Storage for status and received message buffer
    CAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    static enum tcu_status TcuStatus = NOT_READY;
    static enum tcu_status TcuControl = IDLING_STOP_OFF;
    static enum ccu_status CcuStatus = ENGINE_STOP;
    static enum prog_status ProgStatus = PROCESSING;
    static uint16_t PreviousCanId = CAN_ID_CCU;
    static uint8_t Retry = 0;
    static uint8_t AvhStatus = AVH_UNHOLD;
    // static uint8_t PrevAvhStatus = AVH_UNHOLD;
    static float Brake = 0.0;
    static float PrevBrake = 0.0;

    // Initialize peripherals
    system_init();
    can_init();
    led_init();
	
    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));

#ifdef DEBUG_MODE
    usb_init();
#endif

    can_enable();

    while(1){
#ifdef DEBUG_MODE
	cdc_process();
#endif

        // If CAN message receive is pending, process the message
        if(is_can_msg_pending(CAN_RX_FIFO0)){
            can_rx(&rx_msg_header, rx_msg_data);

            if(DebugMode != NORMAL){
                print_rx_frame(&rx_msg_header, rx_msg_data);
            }

            if(rx_msg_header.RTR != CAN_RTR_DATA || rx_msg_header.DLC != 8){
                continue;
            }

            if(DebugMode != CANDUMP){
                switch (rx_msg_header.StdId) {
                    case CAN_ID_SPEED:
                        // PrevSpeed = Speed;
                        PrevBrake = Brake;
                        // Speed = (rx_msg_data[2] + ((rx_msg_data[3] & 0x1f) << 8)) * 0.05625;
                        Brake = rx_msg_data[5] / 0.8;
                        if(100 < Brake){
                            Brake = 100;
                        }

                        if(ProgStatus == SUCCEEDED){
                            if(TcuStatus == IDLING_STOP_OFF && TcuControl == IDLING_STOP_OFF){
			        if(AvhStatus == AVH_HOLD && PrevBrake < BRAKE_HIGH && BRAKE_HIGH <= Brake){
                                    TcuControl = IDLING_STOP_ON;
				    ProgStatus = PROCESSING;
                                    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
                                    dprintf_("# INFO: Request IDLING STOP OFF => ON.\n");
                                }
                            }
                        }

                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_AVH_STATUS:
                        // PrevAvhStatus = AvhStatus;
                        AvhStatus = ((rx_msg_data[5] & 0x22) == 0x22);

                        if(ProgStatus == SUCCEEDED){
                            if(TcuStatus == IDLING_STOP_ON && TcuControl == IDLING_STOP_ON){
                                if(AvhStatus == AVH_UNHOLD){
                                    TcuControl = IDLING_STOP_OFF;
				    ProgStatus = PROCESSING;
                                    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
                                    dprintf_("# INFO: Request IDLING STOP ON => OFF.\n");
				}
                            }
                        }

                        // PreviousCanId = rx_msg_header.StdId;
                        break;
			
                    case CAN_ID_TCU:
			switch(TcuStatus){
			    case NOT_READY:
                                if ((rx_msg_data[2] & 0x08) == 0x08) {
			            if (rx_msg_data[4] == 0xc0) {
                                        TcuStatus = IDLING_STOP_OFF;
                                    } else {
                                        TcuStatus = IDLING_STOP_ON;
			            }
                                    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
                                }
				break;
				
			    case IDLING_STOP_ON:
				if ((rx_msg_data[2] & 0x08) != 0x08) {
                                    TcuStatus = NOT_READY;
                                    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
				} else {
			            if (rx_msg_data[4] == 0xc0) {
                                        TcuStatus = IDLING_STOP_OFF;
                                        led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
			            }
                                }
				break;
				
			    case IDLING_STOP_OFF:
				if ((rx_msg_data[2] & 0x08) != 0x08) {
                                    TcuStatus = NOT_READY;
                                    led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
				} else {
			            if (rx_msg_data[4] != 0xc0) {
                                        TcuStatus = IDLING_STOP_ON;
                                        led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
			            }
                                }
				break;
			}
                        if(ProgStatus == PROCESSING){
                            if(TcuStatus == TcuControl){
				if(TcuStatus == IDLING_STOP_ON){
                                    dprintf_("# INFO: IDLING STOP ON succeeded.\n");
			        } else {
                                    dprintf_("# INFO: IDLING STOP OFF succeeded.\n");
			        }
                                ProgStatus = SUCCEEDED;
                                Retry = 0;
                            }
                        }
                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_CCU:
                        if (PreviousCanId == CAN_ID_CCU) { // TCU don't transmit message
			    if(CcuStatus != ENGINE_STOP){
                                TcuStatus = NOT_READY;
                                TcuControl = IDLING_STOP_OFF;
                                CcuStatus = ENGINE_STOP;
                                ProgStatus = PROCESSING;
                                Retry = 0;
                                AvhStatus = AVH_UNHOLD;
                                // PrevAvhStatus = AVH_UNHOLD;
                                Brake = 0.0;
                                PrevBrake = 0.0;
                                dprintf_("# INFO: ENGINE stop.\n");
                                led_blink(((TcuStatus == IDLING_STOP_ON) << 1) + (TcuControl == IDLING_STOP_ON));
			    }
                        } else {
			    if (rx_msg_data[6] & 0x40) {
                                dprintf_("# INFO: IDLING STOP CONTROL cancelled.\n");
                                ProgStatus = CANCELLED;
                            }
			    switch(ProgStatus){
				case PROCESSING:
                                    switch(CcuStatus){
					case READY:
					    if(TcuStatus != NOT_READY){
					        if (TcuStatus != TcuControl) { // Transmit message for eliminate engine auto stop
                                                    if (MAX_RETRY <= Retry) { // Previous eliminate engine auto stop message failed
                                                        dprintf_("# ERROR: IDLING STOP CONTROL failed. Retry:%d.\n", Retry);
                                                        ProgStatus = FAILED;
                                                    } else {
                                                        Retry++;
                                                        HAL_Delay(50 / 2);
                                                        send_frame(rx_msg_data); // Transmit message
                                                        // Discard message(s) that received during HAL_delay()
                                                        while(is_can_msg_pending(CAN_RX_FIFO0)){
                                                            can_rx(&rx_msg_header, rx_msg_data);
                                                        }
				                        rx_msg_header.StdId = CAN_ID_TCU;
                                                        CcuStatus = PAUSE;
						    }
                                                }
					    }
					    break;

				        case ENGINE_STOP:
                                            dprintf_("# INFO: ENGINE start.\n");
				        case PAUSE:
                                            CcuStatus = READY;
				            break;
				    }

				default: // SUCCEEDED or FAILED or CANCELLED
				    break;
			    }
                        }
                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    default: // Unexpected can id
                        dprintf_("# Warning: Unexpected can id (0x%03x).\n", rx_msg_header.StdId);
                        break;
                }
            }
        }
    }
}


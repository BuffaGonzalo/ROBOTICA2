#ifndef FLEXIROVER_LINK_H_
#define FLEXIROVER_LINK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Adaptador de comunicación. El firmware del FlexiRover aporta UART y CH_PD. */
void FlexiLink_Init(void (*set_ch_pd)(uint8_t), int (*uart_write)(uint8_t),
                   void (*log_message)(const char *),
                   const char *ssid, const char *password, const char *pc_ip,
                   uint16_t pc_port, uint16_t local_port);
void FlexiLink_UartRx(uint8_t byte);
void FlexiLink_Tick10ms(void);
void FlexiLink_Task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif

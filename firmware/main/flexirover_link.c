#include "flexirover_link.h"
#include "esp01.h"
#include "unerPrtcl.h"
#include "util.h"
#include <string.h>

static _sESP01Handle wifi;
static _sComm rx, tx;
static volatile uint8_t rx_buffer[RXBUFSIZE], tx_buffer[TXBUFSIZE];
static char target_ip[16];
static uint16_t target_port, udp_local_port;
static uint8_t connect_pending, reply_pending, connected, pc_seen;
static uint8_t alive_awaiting_send_ok;
static uint32_t next_alive_ms, last_pc_alive_ms;
static void (*log_message)(const char *);

static void log_wifi(const char *message) {
    if (log_message) log_message(message);
}

static void received_byte(uint8_t byte) {
    uint8_t next = (uint8_t)((rx.indexW + 1u) & rx.mask);
    if (next != rx.indexR) {
        rx.buff[rx.indexW] = byte;
        rx.indexW = next;
    }
}

static void changed_state(_eESP01STATUS state) {
    if (state == ESP01_WIFI_CONNECTING_WIFI)
        log_wifi("[WIFI] ESP-01 responde AT; conectando a la red");
    if (state == ESP01_WIFI_DISCONNECTED)
        log_wifi("[WIFI] No se asocio a la red; reintentando");
    if (state == ESP01_WIFI_NEW_IP) {
        log_wifi("[WIFI] ESP-01 obtuvo IP");
        if (ESP01_GetLocalIP()) log_wifi(ESP01_GetLocalIP());
        connect_pending = 1;
    }
    if (state == ESP01_UDPTCP_CONNECTED) {
        if (!connected) log_wifi("[UDP] Socket del ESP-01 listo");
        connected = 1;
        pc_seen = 0;
        reply_pending = 0;
        next_alive_ms = 0;
    }
    if (state == ESP01_UDPTCP_DISCONNECTED || state == ESP01_WIFI_DISCONNECTED) {
        if (connected) log_wifi("[UDP] Socket cerrado");
        else if (state == ESP01_UDPTCP_DISCONNECTED)
            log_wifi("[UDP] No se pudo abrir el socket; reintentando");
        connected = 0;
        alive_awaiting_send_ok = 0;
        pc_seen = 0;
        reply_pending = 0;
    }
    if (state == ESP01_SEND_OK && alive_awaiting_send_ok) {
        alive_awaiting_send_ok = 0;
        log_wifi("[UDP] ALIVE enviado por Wi-Fi: SEND OK");
    }
}

void FlexiLink_Init(void (*set_ch_pd)(uint8_t), int (*uart_write)(uint8_t),
                   void (*log)(const char *),
                   const char *ssid, const char *password, const char *pc_ip,
                   uint16_t pc_port, uint16_t local_port) {
    strncpy(target_ip, pc_ip, sizeof(target_ip) - 1);
    target_ip[sizeof(target_ip) - 1] = '\0';
    target_port = pc_port;
    udp_local_port = local_port;
    log_message = log;
    connect_pending = reply_pending = connected = pc_seen = 0;
    alive_awaiting_send_ok = 0;
    unerPrtcl_Init(&rx, &tx, rx_buffer, tx_buffer);
    wifi.DoCHPD = set_ch_pd;
    wifi.WriteUSARTByte = uart_write;
    wifi.WriteByteToBufRX = received_byte;
    ESP01_Init(&wifi);
    ESP01_AttachChangeState(changed_state);
    ESP01_SetWIFI(ssid, password);
}

void FlexiLink_UartRx(uint8_t byte) { ESP01_WriteRX(byte); }
void FlexiLink_Tick10ms(void) { ESP01_Timeout10ms(); }

void FlexiLink_Task(uint32_t now_ms) {
    ESP01_Task();
    if (connect_pending) {
        connect_pending = 0;
        log_wifi("[UDP] Abriendo socket hacia PC");
        ESP01_StartUDP(target_ip, target_port, udp_local_port);
    }
    while (unerPrtcl_DecodeHeader(&rx)) {
        if (rx.buff[rx.indexData] == ALIVE) {
            reply_pending = 1;
            pc_seen = 1;
            last_pc_alive_ms = now_ms;
        }
    }
    if (pc_seen && (uint32_t)(now_ms - last_pc_alive_ms) >= 6000u) {
        pc_seen = 0;
        next_alive_ms = 0;
    }
    if (!connected || (!reply_pending &&
        (pc_seen || (int32_t)(now_ms - next_alive_ms) < 0)))
        return;

    /* Equivale a UNER 03 : A0 0D 98, la trama de referencia. */
    tx.indexW = tx.indexR = 0;
    unerPrtcl_PutHeaderOnTx(&tx, ALIVE, 2);
    unerPrtcl_PutByteOnTx(&tx, ACK);
    unerPrtcl_PutByteOnTx(&tx, tx.chk);
    if (ESP01_Send(0, tx.buff, 0, tx.nBytes, TXBUFSIZE) == ESP01_SEND_READY) {
        alive_awaiting_send_ok = 1;
        reply_pending = 0;
        if (!pc_seen) next_alive_ms = now_ms + 100u;
    }
}

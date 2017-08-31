/*
This is the firmware for the BLE module.
Intended function:
x    When a bluetooth device connects or disconnects, notify the processor over UART
    When the iOS app sends data, clean it, parse it, and send it over UART
    Respond to UART pings with connection status
    
    Sleep mode
    Put radio to sleep on UART command
    Wake up on UART command
 */
 
#include "mbed.h"
#include "ble/BLE.h"


#define BLE_UUID_TXRX_SERVICE            0x0000 /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_TX_CHARACTERISTIC       0x0002 /**< The UUID of the TX Characteristic. */
#define BLE_UUIDS_RX_CHARACTERISTIC      0x0003 /**< The UUID of the RX Characteristic. */

#define TXRX_BUF_LEN                     40 // Was originally 20

BLE  ble;

Serial uart(USBTX, USBRX);


// The Nordic UART Service
static const uint8_t uart_base_uuid[] = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_tx_uuid[]   = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_rx_uuid[]   = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[] = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};


uint8_t txPayload[TXRX_BUF_LEN] = {0,};
uint8_t rxPayload[TXRX_BUF_LEN] = {0,};

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_len=0;


GattCharacteristic  txCharacteristic (uart_tx_uuid, txPayload, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);
                                      
GattCharacteristic  rxCharacteristic (uart_rx_uuid, rxPayload, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
                                      
GattCharacteristic *uartChars[] = {&txCharacteristic, &rxCharacteristic};

GattService         uartService(uart_base_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));


// On connect, notify over UART
void connectionCallback(const Gap::ConnectionEventCallback_t *params) {
    uart.printf("Connected\r\n");
}

// On disconnect, restart advertising
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params) {
    uart.printf("Disconnected\r\n");
    ble.startAdvertising();
}

// If message is received over the air, clean it, parse it, send it to the main processor
void WrittenHandler(const GattWriteCallbackParams *Handler) {   
    uint8_t buf[TXRX_BUF_LEN];
    uint16_t bytesRead, index;
    
    if (Handler->handle == txCharacteristic.getValueAttribute().getHandle()) {
        ble.readCharacteristicValue(txCharacteristic.getValueAttribute().getHandle(), buf, &bytesRead);
        memset(txPayload, 0, TXRX_BUF_LEN);
        memcpy(txPayload, buf, TXRX_BUF_LEN);

        for(index=0; index<bytesRead; index++)
            uart.putc(txPayload[index]);

        uart.printf("\r\n");
    }
}

// When message is received on UART, parse it and respond
void uartCB() {   
    while(uart.readable()) {
        rx_buf[rx_len++] = uart.getc();

        if(rx_len>=TXRX_BUF_LEN || rx_buf[rx_len-1]=='\0' || rx_buf[rx_len-1]=='\n') {
            ble.updateCharacteristicValue(rxCharacteristic.getValueAttribute().getHandle(), rx_buf, rx_len); 
            // uart.printf("RecHandler \r\n");
            // uart.printf("Length: ");
            // uart.putc(rx_len);
            // uart.printf("\r\n");
            rx_len = 0;
            break;
        }
    }
}

int main(void) {
    ble.init();
    ble.onConnection(connectionCallback);
    ble.onDisconnection(disconnectionCallback);
    ble.onDataWritten(WrittenHandler);  // Received over the air
    
    uart.baud(9600);
    uart.attach(uartCB, uart.RxIrq);

    // setup advertising 
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                    (const uint8_t *)"SwForge", sizeof("SwForge") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                    (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid));
    
    // 100ms; in multiples of 0.625ms. 
    ble.setAdvertisingInterval(160);

    ble.addService(uartService);
    
    ble.startAdvertising(); 
    
    while(1) ble.waitForEvent();
}
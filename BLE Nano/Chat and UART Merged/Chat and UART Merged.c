/* Tyler's merged simple chat + UART console
Starting from simple chat
*/

/*
 *    The application works with the BLEController iOS/Android App.
 *    Type something from the Terminal to send
 *    to the BLEController App or vice verse.
 *    Characteristics received from App will print on Terminal.
 */
 
#include <string.h> // added from uart
#include "mbed.h"
#include "ble/BLE.h"

#include "UARTService.h"  // added from uart

 // added from uart
#define NEED_CONSOLE_OUTPUT 1 /* Set this if you need debug messages on the console;
* it will have an impact on code-size and power consumption. */

#if NEED_CONSOLE_OUTPUT
#define DEBUG(STR) { if (uart) uart->write(STR, strlen(STR)); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */
// end added from uart


#define BLE_UUID_TXRX_SERVICE            0x0000 /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_TX_CHARACTERISTIC       0x0002 /**< The UUID of the TX Characteristic. */
#define BLE_UUIDS_RX_CHARACTERISTIC      0x0003 /**< The UUID of the RX Characteristic. */

#define TXRX_BUF_LEN                     20

BLE  ble;
DigitalOut led1(P0_19); // added from uart
UARTService *uart; // added from uart

Serial pc(USBTX, USBRX);


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



void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    DEBUG("Disconnected!\n\r");
    DEBUG("Restarting the advertising process\n\r");
    // pc.printf("Disconnected \r\n");
    // pc.printf("Restart advertising \r\n");
    ble.startAdvertising();
}

void WrittenHandler(const GattWriteCallbackParams *Handler)
{   
    uint8_t buf[TXRX_BUF_LEN];
    uint16_t bytesRead, index;
    
    if (Handler->handle == txCharacteristic.getValueAttribute().getHandle()) 
    {
        ble.readCharacteristicValue(txCharacteristic.getValueAttribute().getHandle(), buf, &bytesRead);
        memset(txPayload, 0, TXRX_BUF_LEN);
        memcpy(txPayload, buf, TXRX_BUF_LEN);       
        pc.printf("WriteHandler \r\n");
        pc.printf("Length: ");
        pc.putc(bytesRead);
        pc.printf("\r\n");
        pc.printf("Data: ");
        for(index=0; index<bytesRead; index++)
        {
            pc.putc(txPayload[index]);        
        }
        pc.printf("\r\n");
    }
}

void uartCB(void)
{   
    while(pc.readable())    
    {
        rx_buf[rx_len++] = pc.getc();    
        if(rx_len>=20 || rx_buf[rx_len-1]=='\0' || rx_buf[rx_len-1]=='\n')
        {
            ble.updateCharacteristicValue(rxCharacteristic.getValueAttribute().getHandle(), rx_buf, rx_len); 
            pc.printf("RecHandler \r\n");
            pc.printf("Length: ");
            pc.putc(rx_len);
            pc.printf("\r\n");
            rx_len = 0;
            break;
        }
    }
}

void periodicCallback(void)
{
    led1 = !led1;
    DEBUG("ping\r\n");
}   


int main(void)
{
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback, 1);

    DEBUG("Initialising the nRF51822\n\r");
    ble.init();
    ble.onDisconnection(disconnectionCallback);
    

    // PC simple chat stuff
    // ble.onDataWritten(WrittenHandler);  
    // pc.baud(9600);
    // pc.printf("SimpleChat Init \r\n");
    
    // pc.attach( uartCB , pc.RxIrq);

    // New UART stuff
    uart = new UARTService(ble);


   // setup advertising 
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                    (const uint8_t *)"SwingForge", sizeof("SwingForge") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                    (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid));
    // 100ms; in multiples of 0.625ms. 
    ble.setAdvertisingInterval(160);

    // ble.addService(uartService);
    
    DEBUG("Starting Advertising\n\r");
    ble.startAdvertising(); 
    // pc.printf("Advertising Start \r\n");
    
    while(1)
    {
        ble.waitForEvent(); 
    }
}

















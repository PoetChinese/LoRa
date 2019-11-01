
#include <Arduino.h>

#include <SX126x-Arduino.h>
#include <SPI.h>
#define ESP32  1
hw_config hwConfig;

#ifdef ESP32
// ESP32 - SX126x pin configuration
int PIN_LORA_RESET = 27;  // LORA RESET
int PIN_LORA_NSS = 15;	// LORA SPI CS
int PIN_LORA_SCLK = 14;  // LORA SPI CLK
int PIN_LORA_MISO = 13;  // LORA SPI MISO
int PIN_LORA_DIO_1 = 26; // LORA DIO_1
int PIN_LORA_BUSY = 39;  // LORA SPI BUSY
int PIN_LORA_MOSI = 12;  // LORA SPI MOSI
int RADIO_TXEN = 5;	 // LORA ANTENNA TX ENABLE
int RADIO_RXEN = 4;	 // LORA ANTENNA RX ENABLE
#endif


// Function declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);


// Check if the board has an LED port defined
#ifdef ESP32
#define LED_BUILTIN 2
#endif



// Define LoRa parameters
#define RF_FREQUENCY 			434000000  // Hz
#define TX_OUTPUT_POWER 		14		// dBm
#define LORA_BANDWIDTH 			0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 	7      // [SF7..SF12]
#define LORA_CODINGRATE 		1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 	8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 	100   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON 	false
#define RX_TIMEOUT_VALUE 		4000
#define TX_TIMEOUT_VALUE 		5000

#define BUFFER_SIZE 64 // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t RcvBuffer[BUFFER_SIZE];
static uint8_t TxdBuffer[BUFFER_SIZE];
static bool isMaster = true;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

time_t timeToSend;

time_t cadTime;

uint8_t pingCnt = 0;
uint8_t pongCnt = 0;

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Define the HW configuration between MCU and SX126x
	hwConfig.CHIP_TYPE = SX1261_CHIP;		  // Example uses an eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	 // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
	hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	 // Example uses an eByte E22 module which uses RXEN and TXEN pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an eByte E22 module which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	 // Only Insight ISP4520 module uses DIO3 as antenna control

	// Initialize Serial for debug output
	Serial.begin(115200);

	Serial.println("=====================================");
	Serial.println("SX126x PingPong test");
	Serial.println("=====================================");


	#ifdef ESP32
		Serial.println("MCU Espressif ESP32");
	#endif


	uint8_t deviceId[8];

	BoardGetUniqueId(deviceId);
	Serial.printf("BoardId: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
				  deviceId[7],
				  deviceId[6],
				  deviceId[5],
				  deviceId[4],
				  deviceId[3],
				  deviceId[2],
				  deviceId[1],
				  deviceId[0]);

	// Initialize the LoRa chip
	Serial.println("Starting lora_hardware_init");
	lora_hardware_init(hwConfig);

	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	// Set Radio RX configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
	Radio.SetPublicNetwork(true);
	// Start LoRa
	
	Serial.println("Starting Send"); 
	// Radio.Rx(RX_TIMEOUT_VALUE);
    //Radio.SX126xSendPayload("123485");
    //Radio.SetTxContinuousWave(915,15,30);
	Radio.Send((uint8_t *)"123456789abcdefghijklmn",24);
}

void loop()
{
	// Handle Radio events
	Radio.IrqProcess();
	// We are on FreeRTOS, give other tasks a chance to run
	delay(100);
	yield();
	Radio.Send((uint8_t *)"123456789abcdefghijklmn",24);
	delay(5000);
}

/**@brief Function to be executed on Radio Tx Done event
 */
uint16_t i=0;
void OnTxDone(void)
{
	Serial.print("OnTxDone=");
	i++;
	Serial.println(i);
	//Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Serial.println("OnRxDone");
	delay(10);
	BufferSize = size;
	memcpy(RcvBuffer, payload, BufferSize);

	Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);

	for (int idx = 0; idx < size; idx++)
	{
		Serial.printf("%c ", RcvBuffer[idx]);
	}
	Serial.println("");

	// digitalWrite(LED_BUILTIN, HIGH);

	Radio.Rx(RX_TIMEOUT_VALUE);

}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	// Radio.Sleep();
	Serial.println("OnTxTimeout");
	digitalWrite(LED_BUILTIN, LOW);

	Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
	Serial.println("OnRxTimeout");
	digitalWrite(LED_BUILTIN, LOW);

	Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
	Serial.println("OnRxError");
	digitalWrite(LED_BUILTIN, LOW);

	Radio.Rx(RX_TIMEOUT_VALUE);

}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnCadDone(bool cadResult)
{
	time_t duration = millis() - cadTime;

	Radio.Rx(RX_TIMEOUT_VALUE);
}

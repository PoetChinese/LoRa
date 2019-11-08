#include <Arduino.h>

#include <SX126x-Arduino.h>
#include <SPI.h>

hw_config hwConfig;

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

// Function declarations
/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);
/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);
/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);
/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);
/*!
 * \brief Function executed on Radio CAD Done event
 */
void OnCadDone(bool cadResult);
/*!
 * \brief Function configuring CAD parameters
 * \param [in]  cadSymbolNum   The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param [in]  cadDetPeak     Limit for detection of SNR peak used in the CAD
 * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
 * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity
 */
void SX126xConfigureCad(RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint32_t cadTimeout);
void LoRaParameter(uint8_t t);
void Machinestate(void);
// Check if the board has an LED port defined
#define LED_BUILTIN 33

// Define LoRa parameters
#define RF_FREQUENCY 422000000  // Hz
#define TX_OUTPUT_POWER 		14		// dBm
#define LORA_BANDWIDTH 			0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR	7// [SF7..SF12]
#define LORA_CODINGRATE 		1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 	8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT  	8   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON 	false
#define RX_TIMEOUT_VALUE		4000
#define TX_TIMEOUT_VALUE 		5000

#define BUFFER_SIZE 64 // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t RcvBuffer[BUFFER_SIZE];


//CAD parameters
#define CAD_SYMBOL_NUM LORA_CAD_02_SYMBOL
#define CAD_DET_PEAK 22
#define CAD_DET_MIN 10
#define CAD_TIMEOUT_MS 2000

uint8_t _sf=7;
uint8_t _StateCad=0;
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
	hwConfig.RADIO_TXEN = RADIO_TXEN;				  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = RADIO_RXEN;				  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	 // Only Insight ISP4520 module uses DIO3 as antenna control

	// Initialize Serial for debug output
	Serial.begin(115200);

	Serial.println("=====================================");
	Serial.println("SX126x CAD test");
	Serial.println("=====================================");

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
	// Radio.Standby();
	// LoRaParameter(8);
	Radio.Standby();
	Serial.println("Starting CAD");
	SX126xConfigureCad(LORA_CAD_02_SYMBOL, LORA_SPREADING_FACTOR + 15, 10, 2000);
	Radio.StartCad();
	// SX126xSetLoRaSymbNumTimeout(0);	
	// Radio.Rx(0);
}

void loop()
{
	// Handle Radio events
	Radio.IrqProcess();

	yield();

}
uint32_t cad_flag=0;
/**@brief Function to be executed on Radio Rx Error event
 */
void OnCadDone(bool cadResult)
{
	Radio.Standby();
	if (cadResult)
	{
		// cad_flag++;
		// Serial.println("");
		// Serial.print("cad_flag=");
		// Serial.println(cad_flag);
		// Serial.println("\nWaiting for package");	
		Radio.Rx(0);
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else
	{
		// Serial.print(".");

		Radio.Standby();
		SX126xConfigureCad(LORA_CAD_02_SYMBOL, LORA_SPREADING_FACTOR + 15, 10, 2000);
		Radio.StartCad();	
	}
	// return;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
uint8_t temp=0;
void OnRxTimeout(void)
{
	// Serial.println("OnRxTimeout Restart ... ");
	digitalWrite(LED_BUILTIN, LOW);
	if(temp<=100)
	{
		Radio.Standby();
		LoRaParameter(8);
		temp++;
		
	}
	else
	{
		Radio.Standby();
		LoRaParameter(7);
		temp=0;
	}
	
	Radio.Sleep();
	Radio.Standby();
	SX126xConfigureCad(LORA_CAD_02_SYMBOL, LORA_SPREADING_FACTOR + 15, 10, 2000);
	Radio.StartCad();
	delay(5);
	// return;
}
/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	Radio.Standby();
	Serial.println("OnTxDone");
	Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
uint16_t rxflag=0;
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Radio.Standby();
	Serial.println("OnRxDone");
	delay(10);
	BufferSize = size;
	memcpy(RcvBuffer, payload, BufferSize);

	Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);

	for (int idx = 0; idx < size; idx++)
	{
		Serial.printf("%02X ", RcvBuffer[idx]);
	}
	rxflag++;
	Serial.println("");
	Serial.print("rxflag=");
	Serial.println(rxflag);
	Serial.println("Restarting CAD");
	Radio.Standby();
	// _sf=7;
	// LoRaParameter(_sf);
	// _StateCad=0;
	SX126xConfigureCad(LORA_CAD_02_SYMBOL, LORA_SPREADING_FACTOR + 15, 10, 2000);
	Radio.StartCad();
}
/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	Radio.Standby();
	Serial.println("OnTxTimeout");
	digitalWrite(LED_BUILTIN, LOW);

	Serial.println("Restart listening");
	Radio.Rx(0);
	return;

	Radio.Rx(RX_TIMEOUT_VALUE);
}
/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
	Serial.println("OnRxError");
	Radio.Standby();
	SX126xConfigureCad(LORA_CAD_02_SYMBOL, LORA_SPREADING_FACTOR + 15, 10, 2000);
	Radio.StartCad();
	digitalWrite(LED_BUILTIN, LOW);
}

// LORA_CAD_ONLY		LORA_CAD_RX
void SX126xConfigureCad(RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint32_t cadTimeout)
{
	SX126xSetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
						  IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
						  IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SX126xSetCadParams(cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, ((cadTimeout * 1000) / 15.625));
}

void LoRaParameter(uint8_t t)
{
		uint8_t parameter[]={t,0x04,0x01,0x00};
		// Radio.Standby();
		
		SX126xWriteCommand(RADIO_SET_MODULATIONPARAMS, parameter, 4);
}

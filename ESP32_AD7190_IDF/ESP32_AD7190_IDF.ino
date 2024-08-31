// This is just a proof of concept to check SPI MODE 3
// this is based on espressif spi master example:
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/spi_master/lcd/main/spi_master_example_main.c


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Define SPI and board constraints
#define ADC_HOST    SPI2_HOST

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   GPIO_NUM_15
#define PIN_NUM_MEASURE GPIO_NUM_21

esp_err_t ret;
spi_device_handle_t spi;

spi_bus_config_t spiBusConfig;
spi_device_interface_config_t spiDeviceConfig;

// This function is called (in irq context!) just before a transmission starts. 
// It will set the CS/SS (chip select / slave select) line to the value indicated in the user field.
void spi_pre_transfer_callback(spi_transaction_t *t) {
    //int chipSelectLevel = (int)t->user;
    //Serial.printf("CS set to: %d\n", chipSelectLevel);
    gpio_set_level(PIN_NUM_CS, LOW);
}

// Aquesta funcio la he creat jo:
// Per algun putu motiu no esta al putu exemple pero sino no puja el CS
// El AD7190 no li cal pujar el CS pero crec que no esta del tot be
// si no es puja el analitzador logic falla
void spi_post_transfer_callback(spi_transaction_t *t) {
  
  if (!(t->flags & SPI_TRANS_CS_KEEP_ACTIVE)){
    gpio_set_level(PIN_NUM_CS, HIGH);
  }

}

/* Send a command to the ADC. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void adc_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active) {

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       // Zero out the transaction
    t.length = 8;                   // Total data length, in bits. All command are 8 bits
    t.tx_buffer = &cmd;             // The data is the cmd itself
    t.user = (void*)0;              // CS needs to be set to 0
                                    // PERO QUE CULLONS
                                    // La documetacio diu: 
                                    // void *user; User-defined variable. Can be used to store eg transaction ID.
                                    // Aquest cabron ho fa servir per baixar el CS abans de transmetre. Perque cullons no ho fa sol!
    if (keep_cs_active) {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE;       // Keep CS active after data transfer
                                                  // AQUEST PUTO FLAG no funciona!
                                                  // He tingut que crear spi_post_transfer_callback per pujar el flag o no
    }
    ret = spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret == ESP_OK);                        //Should have had no issues.
}

void adc_send_cmd(spi_device_handle_t spi, const uint8_t* cmd, const uint8_t size, bool keep_cs_active) {
    
    Serial.print(F("adc_send_cmd() Register 0x"));
    Serial.print(cmd[0], HEX);
    Serial.print(" ");
    Serial.print(getAddressDebugString(cmd[0]));
    Serial.print(F(", Payload:"));
    for (uint8_t i = 1; i < size; i ++) {
        Serial.printf(" [%d|0x%02x] ", i, cmd[i]);
    }
    Serial.println();

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));             //Zero out the transaction
    t.length = 8 * size;                  //Command is 8 bits
    t.tx_buffer = cmd;                   //The data is the cmd itself
    t.user = (void*)0;                    //D/C needs to be set to 0
    if (keep_cs_active) {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    
    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);
    digitalWrite(PIN_NUM_CS, LOW); 

    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);          //Should have had no issues.

    spi_device_release_bus(spi);

    if (!keep_cs_active) {
        digitalWrite(PIN_NUM_CS, HIGH); 
    }
    
}


bool waitRdyGoLow(void) {
  
  //  RDY only goes low when a valid conversion is available
  
  //  The DOUT/ RDY pin functions as a data ready signal also, the
  //  line going low when a new data-word is available in the output
  //  register. It is reset high when a read operation from the data
  //  register is complete. It also goes high prior to the updating of the
  //  data register to indicate when not to read from the device, to
  //  ensure that a data read is not attempted while the register is being
  //  updated. 

  Serial.println(F("AD7190.waitRdyGoLow()"));

  gpio_set_level(PIN_NUM_MEASURE, LOW);

  pinMode(PIN_NUM_MISO, INPUT);
  uint32_t t = millis();  
  while (digitalRead(PIN_NUM_MISO) == HIGH) {
  }

  gpio_set_level(PIN_NUM_MEASURE, HIGH);

  t = millis()-t;
  Serial.printf("RDY in %dms\n", t);

  return true;
}

void setup() {
  
  Serial.begin(115200);
  delay(2000);
  Serial.println("HOLA!");

  // 1-Initialize an SPI bus by calling the function spi_bus_initialize(). 
  // Make sure to set the correct I/O pins in the struct spi_bus_config_t. 
  // Set the signals that are not needed to -1.

  // 2-Register a Device connected to the bus with the driver by calling the function spi_bus_add_device(). 
  // Make sure to configure any timing requirements the Device might need with the parameter dev_config. 
  // You should now have obtained the Device's handle which will be used when sending a transaction to it.

  // 3-To interact with the Device, fill one or more spi_transaction_t structs with any transaction parameters required. 
  // Then send the structs either using a polling transaction or an interrupt transaction.

  // When the transaction data size is equal to or less than 32 bits, it will be sub-optimal to allocate a buffer for the data. 
  // The data can be directly stored in the transaction struct instead. 
  // For transmitted data, it can be achieved by using the spi_transaction_t::tx_data member and setting the SPI_TRANS_USE_TXDATA flag on the transmission. 
  // For received data, use spi_transaction_t::rx_data and set SPI_TRANS_USE_RXDATA
  // In both cases, do not touch the spi_transaction_t::tx_buffer or spi_transaction_t::rx_buffer 
  
  // Init GPIOS for SPI
  // Relealy required?

  pinMode(PIN_NUM_CLK, OUTPUT);
  pinMode(PIN_NUM_CS, OUTPUT);
  pinMode(PIN_NUM_MEASURE, OUTPUT);
  digitalWrite(PIN_NUM_CLK, HIGH);
  digitalWrite(PIN_NUM_CS, HIGH);
  digitalWrite(PIN_NUM_MEASURE, HIGH);

  spiBusConfig.miso_io_num = PIN_NUM_MISO;
  spiBusConfig.mosi_io_num = PIN_NUM_MOSI;
  spiBusConfig.sclk_io_num = PIN_NUM_CLK;
  spiBusConfig.quadwp_io_num = -1;
  spiBusConfig.quadhd_io_num = -1;

  spiBusConfig.data4_io_num = -1;
  spiBusConfig.data5_io_num = -1;
  spiBusConfig.data6_io_num = -1;
  spiBusConfig.data7_io_num = -1;
  spiBusConfig.max_transfer_sz = 4;                       // Maximum transfer size, in bytes. 
                                                          // Defaults to 4092 if 0 when DMA enabled, or to `SOC_SPI_MAXIMUM_BUFFER_SIZE` if DMA is disabled.

  // spiBusConfig.isr_cpu_id = 0;                         // Select cpu core to register SPI ISR.
  // spiBusConfig.flags = SPICOMMON_BUSFLAG_MASTER        // Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.

  spiDeviceConfig.clock_speed_hz = 5 * 1000 * 1000,       // Clock out at 5 MHz
  spiDeviceConfig.mode = 3,                               // SPI mode 3. representing a pair of (CPOL = 1, CPHA = 1) configuration
  spiDeviceConfig.spics_io_num = PIN_NUM_CS,              // CS pin
  spiDeviceConfig.queue_size = 7,                         // We want to be able to queue 7 transactions at a time
  spiDeviceConfig.pre_cb = spi_pre_transfer_callback,     // Specify pre-transfer callback to handle CS line
  spiDeviceConfig.post_cb = spi_post_transfer_callback,   // Specify pre-transfer callback to handle CS line
  
  spiDeviceConfig.cs_ena_pretrans = 0;                    // Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
  spiDeviceConfig.cs_ena_posttrans = 0;                   // Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
  // spiDeviceConfig.input_delay_ns = XX;                 // Maximum data valid time of slave. The time required between SCLK and MISO
                                                          // valid, including the possible clock delay from slave to master. The driver uses this value to give an extra
                                                          // delay before the MISO is ready on the line. Leave at 0 unless you know you need a delay. For better timing
                                                          // performance at high frequency (over 8MHz), it's suggest to have the right value.

  //Initialize the SPI bus
  //ret = spi_bus_initialize(LCD_HOST, &spiBusConfig, SPI_DMA_CH_AUTO);
  ret = spi_bus_initialize(ADC_HOST, &spiBusConfig, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  
  //Attach the LCD to the SPI bus
  ret = spi_bus_add_device(ADC_HOST, &spiDeviceConfig, &spi);
  ESP_ERROR_CHECK(ret);

  //Initialize non-SPI GPIOs
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
  spi_device_acquire_bus(spi, portMAX_DELAY);

  //  The serial interface can be reset by writing a series of 1s to the
  //  DIN input. If a Logic 1 is written to the AD7190 DIN line for at
  //  least 40 serial clock cycles, the serial interface is reset. This ensures
  //  that the interface can be reset to a known state if the interface gets
  //  lost due to a software error or some glitch in the system. Reset
  //  returns the interface to the state in which it is expecting a write to
  //  the communications register. This operation resets the contents of
  //  all registers to their power-on values. Following a reset, the user
  //  should allow a period of 500 µs before addressing the serial
  //  interface.
  adc_cmd(spi, 0xFF, true);       //  1
  adc_cmd(spi, 0xFF, true);       //  2
  adc_cmd(spi, 0xFF, true);       //  3
  adc_cmd(spi, 0xFF, true);       //  4
  adc_cmd(spi, 0xFF, true);      //  5          // At least means 40 just 40? => 5 bytes * 8 bits
  // adc_cmd(spi, 0xFF, true);        //  6
  // adc_cmd(spi, 0xFF, false);       //  7

  // From datasheet it should be 500 µs.
  // But RDY/MISO goes low after ~80 ms
  // Is it better to wait RDY/MISO to go low?
  // In the past I saw some issues. May be related with MODE3 bug
  waitRdyGoLow();
  
  //delay(1);
  Serial.println("AD7190 reset done");


  // Request to read REG_ID (0x60)
  adc_cmd(spi, 0x60, true);

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8 * 1;                                           // Total data length, in bits.
  t.flags = SPI_TRANS_USE_RXDATA;                             // Receive into rx_data member of spi_transaction_t instead into memory at rx_buffer.
  
  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  assert(ret == ESP_OK);

  Serial.printf("AD7190_REG_ID response: %X\n", t.rx_data[0]);  // THIS IS AD7190 MUST BE 0x8x mine always reply 0x84 :)
  
  // The identification number for the AD7190 is stored in the ID register. This is a read-only register
  // Power-On/Reset = 0xX4
  if ((t.rx_data[0] & 0xF) == 0x4){
    Serial.println(F("AD7190 Identification OK"));
  }else{
    Serial.println(F("AD7190 Identification FAILED!"));
  }

  // Configure AD7190 to read internal temperature sensor (channel: 4)

  // Release bus
  spi_device_release_bus(spi);

  // Write AD7190_CONF_CHAN register
  // Select Temperature sensor channel
  // No Chop + BUF + Gain 0
  const uint8_t cmdArray[4] = {0x10, 0x00, 0x04, 0x10};
  adc_send_cmd(spi, cmdArray, 4, false);

  // Write AD7190_CONF_MODE register
  // Mode 1: Single conversion mode
  // No data status
  // Internal 4.92 MHz clock. Pin MCLK2 is tristated     --> Aixo es pot canviar a external clock tambe (esta montat a la PCB)
  const uint8_t cmdArray2[4] = {0x08, 0x28, 0x00, 0x60};
  adc_send_cmd(spi, cmdArray2, 4, true);

  memset(&t, 0, sizeof(t));
  t.length = 8 * 3;                                           // Total data length, in bits.
  t.flags = SPI_TRANS_USE_RXDATA;                             // Receive into rx_data member of spi_transaction_t instead into memory at rx_buffer.
  
  spi_device_acquire_bus(spi, portMAX_DELAY);

  // Wait AD7190 conversion (MISO/RDY pin to go low)
  waitRdyGoLow();
  
  // READ AD7190_DATA_REG register
  adc_cmd(spi, 0x58, true);

  ret = spi_device_polling_transmit(spi, &t);
  assert(ret == ESP_OK);

  uint32_t rawData = (t.rx_data[0] << 16) + (t.rx_data[1] << 8) + (t.rx_data[2]);

  //  Temp (K) = (Conversion – 0x800000)/2815 K
  //  Temp (°C) = Temp (K) – 273
  float temperature = 0x0;
  temperature = rawData - 0x800000;
  temperature /= 2815;            // Kelvin Temperature
  temperature -= 273;             //Celsius Temperature

  Serial.printf("AD7190 Temperature: %.1f°C RAW: %X\n", temperature, rawData);

  // Release bus
  spi_device_release_bus(spi);
  
  // -------------------------------------------------------------
  // Configure Ad7190 Continuous Read
  // -------------------------------------------------------------

  // Write GPIO_CON_Register
  // BPDS: Bridge power-down switch (Switch closed)
  // P2 and P3 enabled
  // P3 ON! (White LED)
  const uint8_t cmdArray3[2] = {0x28, 0x68};
  adc_send_cmd(spi, cmdArray3, 2, false);

  // Write CONF_CHAN register
  // CHOP ON
  // Channel 1 (AIN1/AIN2) and Channel 2 (AIN3/AIN4)
  // Gain 128
  const uint8_t cmdArray4[4] = {0x10, 0x80, 0x03, 0x07};
  adc_send_cmd(spi, cmdArray4, 4, false);

  // Write CONF_MODE register
  // Mode 0: Continuous conversion mode
  // DAT_STA active: transmission of status register contents after each data register read
  // Internal 4.92 MHz clock. Pin MCLK2 is tristated
  // ENPAR: parity checking on the data register
  const uint8_t cmdArray5[4] = {0x08, 0x18, 0x20, 0x50};
  adc_send_cmd(spi, cmdArray5, 4, true);                        // Important: Keep CS active for MISO/RDY to report readyness
  
  // READ 10 SAMPLES IN CONTINOUS MODE!
  // TWO CHANNELS!
  // Configuration done.
  // Wait conversion to finish
  // waitRdyGoLow();

  // Set Chip select to low
  // gpio_set_level(PIN_NUM_CS, LOW);
  
  spi_device_acquire_bus(spi, portMAX_DELAY);
  // Read Data Register
  // CREAD ON: 
  //  Continuous read of the data register. When this bit is set to 1 (and the data register is selected), the serial
  //  interface is configured so that the data register can be continuously read; that is, the contents of the data
  //  register are automatically placed on the DOUT pin when the SCLK pulses are applied after the RDY pin
  //  goes low to indicate that a conversion is complete. The communications register does not have to be
  //  written to for subsequent data reads. To enable continuous read, the instruction 01011100 must be written
  //  to the communications register. To disable continuous read, the instruction 01011000 must be written to
  //  the communications register while the RDY pin is low. 
  //  IMPORTANT:
  /// While continuous read is enabled, the ADC monitors activity on the DIN line so that it can receive the instruction 
  //  to disable continuous read. Additionally, a reset occurs if 40 consecutive 1s are seen on DIN. 
  //  Therefore, DIN should be held low until an instruction is to be written to the device.
  
  gpio_set_level(PIN_NUM_CS, LOW);

  adc_cmd(spi, 0x5C, true);
  spi_device_release_bus(spi);

  Serial.printf("Size of t: %d\n", sizeof(t));

  const uint8_t cmdArrayRead[4] = {0x00, 0x00, 0x00, 0x00};

  uint8_t counter = 1;
  while (counter <= 10){

    memset(&t, 0, sizeof(t));
    t.length = 8 * 4;                                             // Total data length, in bits.
    //t.tx_data[0] = 0;
    //t.tx_data[1] = 0;
    //t.tx_data[2] = 0;
    //t.tx_data[3] = 0;
    t.tx_buffer = cmdArrayRead;
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE | SPI_TRANS_USE_RXDATA;    // Receive into rx_data member of spi_transaction_t instead into memory at rx_buffer.
    //t.user = (void*)1;                                          // User-defined variable. Can be used to store eg transaction ID.
    
    waitRdyGoLow();                                               //  Wait AD7190 conversion to finish        
    
    spi_device_acquire_bus(spi, portMAX_DELAY);
    ret = spi_device_polling_transmit(spi, &t);
    spi_device_release_bus(spi);

    uint32_t result = (t.rx_data[0] << 16) | (t.rx_data[1] << 8) | t.rx_data[2];
    Serial.println(result, HEX);

    uint8_t status = t.rx_data[3];
    Serial.printf("Sample %d: 0x%X - Status: 0x%X - ", counter, result, status);
    printStatusInfo(status);

    counter = counter + 1;
  }

}

// Do nothing
void loop() {
  delay(1);
  manageSerial();
}


void manageSerial(void){

  // Check if there is any UART command:
  if (!Serial.available()) {
    return;
  }
  uint8_t c = Serial.read();
  if(c == 0xA)                                      // if new line on UART print menu
  {
    Serial.println(F("\n\nUART MENU:"));
    Serial.println(F("r: ESP RESET"));
    Serial.println();
    return;
  }else if (c =='r'){
    Serial.print(F("\nSW RESET!"));
    ESP.restart();
  }
}


void printStatusInfo(uint8_t status){
  bool rdy =        !((status & 0x80) >> 7);
  bool err =         (status & 0x40) >> 6;
  bool noref =       (status & 0x20) >> 5;
  bool parity =      (status & 0x10) >> 4;
  uint8_t channel =  (status & 0x07); 

  if (rdy){
    Serial.print(F(" [RDY] "));
  }else{
    Serial.print(F(" [NOT RDY] "));
  }
  
  if (err){
    Serial.print(F(" [ERR] "));
  }else{
    Serial.print(F(" [E_OK] "));
  }

  if (noref){
    Serial.print(F(" [NO_REF] "));
  }else{
    Serial.print(F(" [R_OK] "));
  }

  if (parity){
    Serial.print(F(" [Od] "));          //  Odd number of 1
  }else{
    Serial.print(F(" [Ev] "));          //  Even number of 1
  }

  Serial.print(F("Channel: "));
  Serial.println(channel);
}


char* getAddressDebugString(uint8_t a){

  bool writeEnable =    (a & 0x80) >> 7;
  bool writeOperation = !((a & 0x40) >> 6);
  uint8_t regAdd =      (a & 0x38) >> 3;
  bool creadRead =      (a & 0x04) >> 2;

  char we[3];
  if (writeEnable){
    strcpy(we,"WE");
  }else{
    strcpy(we,"  ");
  }
  char w[2];
  if (writeOperation){
    strcpy(w,"W");
  }else{
    strcpy(w,"R");
  }
  char c[3];
  if (creadRead){
    strcpy(c,"CR");
  }else{
    strcpy(c,"  ");
  }

  char regAdd_char[20];
  strcpy(regAdd_char,"AD7190_UNKNOWN");
    
  switch(regAdd) {
    case 0:
      if (writeOperation){
        strcpy(regAdd_char,"AD7190_REG_COMM");
      }else{
        strcpy(regAdd_char,"AD7190_REG_STAT");
      }
      break;
      
    case 1:
      strcpy(regAdd_char,"AD7190_REG_MODE");
      break;
      
    case 2:
      strcpy(regAdd_char,"AD7190_REG_CONF");
      break;
      
    case 3:
      if (!writeOperation){
        strcpy(regAdd_char,"AD7190_REG_DATA");
      }
      break;
      
    case 4:
      if (!writeOperation){
        strcpy(regAdd_char,"AD7190_REG_ID");
      }
      break;
      
    case 5:
      strcpy(regAdd_char,"AD7190_REG_GPOCON");
      break;
      
    case 6:
      strcpy(regAdd_char,"AD7190_REG_OFFSET");
      break;
      
    case 7:
      strcpy(regAdd_char,"AD7190_REG_FULLSCALE");
      break;
      
    default:
      strcpy(regAdd_char,"AD7190_UNKNOWN");
    } 
  
  static char bufferDebugAddress[40];
  char* formato="[%s][%s][%s] (%d: %s)";
  sprintf(bufferDebugAddress, formato, we, w, c, regAdd, regAdd_char);

  return bufferDebugAddress;  
}
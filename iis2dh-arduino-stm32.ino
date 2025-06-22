// STWINKT1B
// STM32L4R9ZIJ6
// Using Generic stm32duino stm32l4 
// Board generic
// Sensor IIS2DH Basic Example
// Libraries at https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/iis2dh_STdC
// https://github.com/STMicroelectronics/iis2dh-pid.git
//
#include "SPI.h"
#include "iis2dh_reg.h"
#include <LibPrintf.h>

#define LED_BUILTIN PE1
#define LED2 PD0

// SerialPort is on SWD
HardwareSerial Serial2(PD_6,PD_5);
#define SerialPort  Serial2

// SPI Port on
#define SPI_MOSI PB5
#define SPI_MISO PB4
#define SPI_SCLK PB3
//Define Software Chip Select pin to use (default: SS)
#define IIS2DH_SPI_CS   PD15

SPIClass dev_interface(SPI_MOSI, SPI_MISO, SPI_SCLK);
uint32_t spi_speed = 10000000;


stmdev_ctx_t dev_ctx_iis2dh;

void setup() {
  // Led
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Initialize serial for output
  printf_init(SerialPort);
  SerialPort.begin(230400);

  // Initialize SPI bus interface
  dev_interface.begin();
  dev_interface.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
  delay(1000);

  // Initialize iis2dh sensor
  iis2dh_init();
  delay(1000);
}

void loop() {
// Get data from iis2dh 
iis2dh_get_accel_data(); 
delay(1000);
}

// Write SPI data
static int32_t platform_write_iis2dh(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  digitalWrite(IIS2DH_SPI_CS, LOW);
  /* Write Reg Address */
  reg |= 0x40;
  dev_interface.transfer(reg);
  /* Write the data */
  for (uint16_t i = 0; i < len; i++) {
    dev_interface.transfer(bufp[i]);
  }
  digitalWrite(IIS2DH_SPI_CS, HIGH);
  return 0;
}

// Read SPI data
static int32_t platform_read_iis2dh(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  digitalWrite(IIS2DH_SPI_CS, LOW);
  /* Write Reg Address */
  reg |= 0xC0;
  dev_interface.transfer(reg | 0x80);
  /* Read the data */
  for (uint16_t i = 0; i < len; i++) {
    *(bufp + i) = dev_interface.transfer(0x00);
    }
  digitalWrite(IIS2DH_SPI_CS, HIGH);
  return 0;
}

static void platform_delay(uint32_t ms)
{

}

static void platform_init(void)
{

}

// Initialize settings
void iis2dh_init(void)
{
  static uint8_t whoamI;
  // Initialize CS/SS Pin
  pinMode(IIS2DH_SPI_CS, OUTPUT);
  digitalWrite(IIS2DH_SPI_CS, HIGH);

    /* Initialize mems driver interface */
  dev_ctx_iis2dh.write_reg = platform_write_iis2dh;
  dev_ctx_iis2dh.read_reg = platform_read_iis2dh;
  dev_ctx_iis2dh.handle = (void*) (NULL);
  /* Check device ID */
  iis2dh_device_id_get(&dev_ctx_iis2dh, &whoamI);


  if (whoamI != IIS2DH_ID) {
    while (1) {
      /* manage here device not found */
      // LED_BUILTIN Blinks
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
    }
  }

  printf("Device Found with Id %#x\n",whoamI);
  int8_t count = 3;
  while(count-- > 0) {
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED2, LOW);
    delay(100);
  }

  /* Enable Block Data Update */
  iis2dh_block_data_update_set(&dev_ctx_iis2dh, PROPERTY_ENABLE);
  /* Set Output Data Rate to 1Hz */
  iis2dh_data_rate_set(&dev_ctx_iis2dh, IIS2DH_ODR_10Hz);
  /* Set full scale to 2g */
  iis2dh_full_scale_set(&dev_ctx_iis2dh, IIS2DH_2g);
  /* Enable temperature sensor */
  iis2dh_temperature_meas_set(&dev_ctx_iis2dh, IIS2DH_TEMP_ENABLE);
  /* Set device in continuous mode with 12 bit resol. */
  iis2dh_operating_mode_set(&dev_ctx_iis2dh, IIS2DH_HR_12bit);
}

void iis2dh_get_accel_data(void) {
  iis2dh_reg_t reg;
  static int16_t data_raw_acceleration[3];
  static float_t acceleration_mg[3];
  static uint8_t tx_buffer[1000];

    /* Read output only if new value available */
    iis2dh_xl_data_ready_get(&dev_ctx_iis2dh, &reg.byte);

    if (reg.byte) {
      /* Read accelerometer data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      iis2dh_acceleration_raw_get(&dev_ctx_iis2dh, data_raw_acceleration);
      acceleration_mg[0] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "IIS2DH Acceleration [mg]: %4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      printf("%s",tx_buffer);
    }
}
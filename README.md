================================================================================
Overall pin assignments (summary, kept in sync with `main/hello_world_main.c`)
================================================================================

- I2C:
	- SDA: GPIO 41 (I2C_MASTER_SDA_IO)
	- SCL: GPIO 42 (I2C_MASTER_SCL_IO)
	- I2C port: I2C_NUM_0
	- Default I2C test frequency used in code: 400000 Hz (configurable)

- SPI:
  - MOSI: GPIO 34
  - MISO: GPIO 33
  - SCLK: GPIO 48
  - MT25Q CS: GPIO 21 (MT25Q_SPI_CS)
  - E22 CS: GPIO 35 (LORA_SPI_CS)

- E22 LoRa control pins:
  - RX_EN: GPIO 4
  - TX_EN: GPIO 5
  - BUSY:  GPIO 26
  - NRST:  GPIO 47
  - DIO1:  not configured by default (set LORA_DIO1_PIN in code)

- GPS (UART): //ATGM336H-6N-74
  - UART port: UART_NUM_1
  - TX: GPIO 9 (GPS_UART_TX_PIN)
  - RX: GPIO 10 (GPS_UART_RX_PIN)
  - Baud rate: 9600

- LEDs / UI:
  - LED_R: GPIO 8
  - LED_G: GPIO 7
  - LED_B: GPIO 6
  - Buzzer: GPIO 11 (tested at startup with 2 Hz soft beep)

- Sensor interrupts (as defined in code):
	- BMP388 INT: GPIO 36
	- ICM-42688 INT1: GPIO 39
	- ICM-42688 INT2: GPIO 40
	- ADXL interrupts: GPIOs 38, 37

Device identification (WHO_AM_I / device-id values used by the sensor tests):

- BMP388
  - WHO_AM_I register: 0x00
  - Expected value: 0x50
  - Common I2C addresses tried by test: 0x76, 0x77

- ICM-42688 (ICM-42688-P)
  - WHO_AM_I register: 0x75
  - Expected value: 0x47
  - Common I2C addresses tried by test: 0x68, 0x69

- LIS2MDL
  - WHO_AM_I register: 0x4F
  - Expected value: 0x40
  - Common I2C addresses tried by test: 0x1E, 0x1C, 0x1D

- E22-900M30S LoRa module
  - Communication: SPI (not I2C)
  - SPI bus: SPI2_HOST
  - SPI CS pin: GPIO 35 (LORA_SPI_CS)
  - Control pins: RX_EN (GPIO 4), TX_EN (GPIO 5), BUSY (GPIO 26), NRST (GPIO 47)
  - **Power-efficient operation**: Initializes once, sends a single test packet ("BEACON"), then immediately enters sleep mode.
  - Sleep mode reduces current draw to ~1-10 µA (vs. ~20-50 mA in active mode).
  - Status is probed first via SPI GetStatus (0xC0) to verify connectivity.
  - Not used for RX operations during main application (stays in sleep).

- ATGM336H-6N-74 GPS module
  - Communication: UART (not I2C or SPI)
  - UART port: UART_NUM_1
  - TX pin: GPIO 9, RX pin: GPIO 10
  - Baud rate: 9600
  - Data: NMEA sentences (GPRMC for position, GPGGA for altitude/satellite count)
  - The test initializes the UART driver and attempts to read NMEA data with GPRMC/GPGGA parsing.
  - Provides latitude, longitude, altitude, satellite count, and HDOP (Horizontal Dilution of Precision).

How the test works
------------------
- **Startup beep**: A soft 2 Hz tone (500ms on, 500ms off) is sounded on GPIO 11 as an audible system-ready indicator.
- **I2C scanner**: Lists all responding I2C devices on the bus (GPIO 41 SDA, GPIO 42 SCL @ 400 kHz).
- **I2C sensor probes**: Reads WHO_AM_I / device-id registers for BMP388, ICM-42688, and LIS2MDL at common addresses.
  - Logs "OK" if the value matches the expected device ID, or "UNEXPECTED" if it responds but doesn't match.
  - Silent if no response (no log entry for unreachable address).
- **E22 LoRa SPI probe & transmit (power-efficient)**: 
  - First probes via SPI GetStatus (0xC0) to verify connectivity.
  - Initializes E22 with default configuration (9600 baud, 2400 air data rate, channel 23, 30 dBm TX power).
  - Sends a single test packet with payload "BEACON" (~6 bytes).
  - Waits 100ms for transmission to complete.
  - Puts module into **sleep mode** immediately after sending (current draw reduces from ~30 mA to ~2 µA).
  - Module remains in sleep for the duration of the main application to minimize power consumption.
  - Logs "E22 test packet sent: BEACON (size=6 bytes)" and "E22 entered sleep mode (power consumption minimized)".
- **GPS UART probe**: Initializes the GPS module on UART_NUM_1 and attempts to read NMEA sentences.
  - Waits 500ms for GPS data to arrive.
  - Logs "GPS module responding (OK)" if data is received.
  - Attempts to parse GPRMC and GPGGA sentences.
  - Logs position, altitude, satellite count, and HDOP if a valid fix is available.
  - Logs "GPS no fix yet" if module is responding but hasn't acquired a fix (e.g., cold start).
  - GPS module is deinitialized after the test (UART driver released).
- The I2C driver remains installed throughout so all subsequent I2C operations work without re-initialization.
- The main application then initializes E22 radio, RGB LED fade loop, and other subsystems.

Notes
-----
- If a sensor is on non-standard address or uses SPI instead of I2C, update the code in
	`main/hello_world_main.c` to point at the correct bus/pins and addresses.
- GPS UART is configured on UART_NUM_1 with TX=GPIO 9 and RX=GPIO 10 @ 9600 baud.
- GPS library source: `main/drivers/gps/atgm336h.h` and `main/drivers/gps/atgm336h.c`
  - Provides UART initialization, NMEA sentence parsing (GPRMC/GPGGA), and data structures for position/altitude/satellites.
  - Note: GPS cold start may take 30-60 seconds to acquire first fix. Test timeout is 2 seconds.

Power Consumption Notes
-----------------------
- **E22 LoRa**: Typically draws 30-50 mA during active transmission. Sleep mode reduces to ~1-10 µA.
  - Current design: transmits a single 6-byte beacon, then sleeps indefinitely.
  - Wake-up time from sleep: ~10-30 ms (not used in current test flow).
- **GPS (ATGM336H)**: Typically draws 60-80 mA during active tracking.
  - Current design: UART driver is deinitialized after test (GPIO driver remains but UART released).
- **Sensors (I2C)**: BMP388, ICM-42688, LIS2MDL all draw < 10 mA each during probing.
  - I2C bus remains active but sensors stay in default low-power state.
- **Buzzer**: Single 1-second activation (500ms on, 500ms off) at startup only.
- **RGB LEDs**: LEDC PWM loop is the dominant power consumer in main app (can be disabled or reduced).

- The source constants are defined in `main/hello_world_main.c` (pins) and the sensor headers
	under `main/sensors/` and `main/drivers/gps/` (WHO_AM_I register/address definitions).

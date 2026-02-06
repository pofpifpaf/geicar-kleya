# This file describes the main steps to use the imu/ecompass

# Contents
1. **Detailed manual**
*  1. How to use the card 
*  2. Frames format
*  3. Basic application structure

# 1. Detailed manual

## i. How to use the card 

1. Plug the card to an USB port on raspberry PI
2. A new device should spawn (/dev/ttyACM0) 
3. Open this device either in a ROS Node (using Posix API) of with screen/minicom/gtkterm
4. The device is seen as a serial connection: 115200 8/N/1 

**Card is ready and already send Motion / environmental / Ecompass measures**

## ii. Frames format
Frames sent by the card use the following format:

<SOH><Msg id><Frame length><Data><Checksum>

with

* SOH (Start of heading): 1 byte : 0x01
* Msg id: 1 byte : See following table for values
* Frame length : 2 bytes : length of the whole framen header included
* data: n bytes: Data of the frame
* checksum: 1 byte: see corresponding point for crc calculation

### Msg ID values:

| Name          | Value | Usage                                       |
| -----         | ----- | -----                                       |
| GET_VERSION   | 0x10  | Request from PC to get firmware version     |
| VERSION       | 0x11  | Answer from IMU board with firmware version |
| RESET_HEADING | 0x20  | Request from PC to reset heading            |
| CALIBRATE_MAG | 0x21  | Request from PC to Launch mag calibration   |
| MOTION        | 0x30  | Periodic frame (100ms) with motion sensors data     |
| ECOMPASS      | 0x31  | Periodic frame (100ms) with ecompass data   |
| ENV           | 0x32  | Periodic frame (1s) with environmental data |

### Checksum calculation

Checksum is a simple sum of all byte frome SOH to last byte of data (whole frame except checksum byte). Only the 8 bits of LSB are meaningful. Algorithm is as follow:

      uint8_t checksum

      for (int i =0; i< length; i++) {
            checksum += frame[i];
      }

      if (checksum == frame[length]) {
            // checksum correct
      }

### Data formats 
#### MOTION

* dps : Degree per second
* g : hearth acceleration

| Byte offset | Length | Type | Meaning |
| -------- | ----- | ----- | ------- |
| 0  | 4 | float_32 | Accelerometer X (in g) |
| 4  | 4 | float_32 | Accelerometer Y (in g) |
| 8  | 4 | float_32 | Accelerometer Z (in g) |
| 12 | 4 | float_32 | Gyroscope X (in dps) |
| 16 | 4 | float_32 | Gyroscope Y (in dps) |
| 20 | 4 | float_32 | Gyroscope Z (in dps) |
| 24 | 4 | float_32 | Magnetometer X (in gauss) |
| 28 | 4 | float_32 | Magnetometer Y (in gauss) |
| 32 | 4 | float_32 | Magnetometer Z (in gauss) |

#### ECompass

| Byte offset | Length | Type | Meaning |
| -------- | ----- | ----- | ------- |
| 0  | 4 | float_32 | Yaw (in deg)|
| 4  | 4 | float_32 | Pitch (in deg) |
| 8  | 4 | float_32 | Roll (in deg) |
| 12 | 4 | float_32 | Heading (in deg) |
| 16 | 1 | uint8_t | Heading valid (Boolean)|

#### Environmental sensor

* hPa : Hecto Pascal

| Byte offset | Length | Type | Meaning |
| -------- | ----- | ----- | ------- |
| 0  | 4 | float_32 | Temperature (in °C) |
| 4  | 4 | float_32 | Pressure (in hPa) |
| 8  | 4 | float_32 | Humidity (in %) |

## iii. Basic application structure

### Files structure

* [carte-imu-gps-freertos.ioc](./carte-imu-gps-freertos.ioc) CubeMX file description
 * [Application](./Application) Main Application directory
   * [app.(c/h)]() Main application code
   * [com_mgr.(c/h)]() Communication over USB manager
   * [configuration.h]() Application configuration file
   * [debug.(c/h)]() Debug facilities
   * [ecompass.(c/h)]() ECompass manager
   * [sensors.(c/h)]() Sensors Manager
   * [tasks.(c/h)]() Tasks creation and definition
 * [Core](./Core) Automatically gene
 * [Drivers](./Drivers) HAL and peripherals drivers
 * [Middlewares](./Middlewares) Freertos and others runtimes
 * [X-CUBE-MEMS1]() ECompass runtime and sensors drivers

### Application structure

Application is event-driven based. Each event task use a mailbox (message Queue) for receiving messages from others task. Others task include periodic tasks.

Application is built around main application task used as orchestrator : it receive message from all other part of software, make basic treatment and then trigger new event by posting requests to other mailbox. Basically, it is build around a switch ... case statement using message ID as selector.

Tasks are defined in **tasks.c** file. Each tasks then call a processing function in other application modules (file) that effectively do processing (use).

Event tasks include :
| Task name (in tasks.c) | Processing function | File | Use |
| ----------------------- | -------------------| ---- | --- |
| TASKS_AppLoop | APP_Run | app.c | Orchestrator and central point for messages processing |
| TASKS_ComTXLoop | COM_RunTX | com_mgr.c | Receive data to embbed in frame and send over USB |
| TASKS_EcompassLoop | ECOMPASS_ProcessMessage | ecompass.c | Receive motion data and process them to compute ecompass data (yaw/pitch/roll/heading) |


Periodic tasks include :
| Task name (in tasks.c) | Processing function | File | Period | Use |
| ----------------------- | -------------------| ---- | --- | --- |
| TASKS_ComRXLoop | COM_RunRX | com_mgr.c | 10 ms | Check circular buffer for incoming data |
| TASKS_DebugLoop | DEBUG_PrintPeriodicInfo | debug.c | 1s | Print debug information about tasks status and heap usage |
| TASKS_SensorsLoop | SENSORS_TriggerMeasurements | sensors.c | 10 ms | Start sensors acquisition |
| MotionTimerCallback | SENSORS_SendMotionMesures | sensors.c | 100 ms | Send a request for sending motion data over USB |
| EnvTimerCallback | SENSORS_SendEnvMesures | sensors.c | 1 s | Send a request for sending environemental data over USB |
| EcompassTimerCallback | ECOMPASS_SendMesures | ecompass.c | 100 ms | Send a request for sending ecompass data over USB |
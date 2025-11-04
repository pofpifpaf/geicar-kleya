/**
 * @file ultrasound.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Functions to control the ultrasonic sensors of the car.
 * This file contains the functions to trigger the ultrasonic sensors and measure the distance.
 * It uses TIM3 for microsecond timing and GPIO for triggering the sensors.
 */

#include <sensors.h>
#include "main.h"

#include "configuration.h"

#include "iks4a1_motion_sensors.h"
#include "iks4a1_env_sensors.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include "tasks.h" // for xAppLoopQueue
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#define SENSOR_OK 0
#define SENSOR_ERROR (!SENSOR_OK)

/* Accelerometer LSM6DSV16X */
LSM6DSV16X_Object_t lsm6dsv16x_acc_handler;
LSM6DSV16X_AxesRaw_t acceleration_raw = {0};
float lsm6dsv16x_acc_sensitivity = 0.0f;
uint32_t SENSORS_LSM6DSV16X_InitAcc(LSM6DSV16X_Object_t *handler);

/* Gyroscope LSM6DSV16X */
LSM6DSV16X_Object_t lsm6dsv16x_gyro_handler;
LSM6DSV16X_AxesRaw_t angular_velocity_raw = {0};
#define NUM_CALIBRATION_SAMPLES 64
LSM6DSV16X_AxesRaw_t gyro_bias = {0};
float lsm6dsv16x_gyro_sensitivity = 0.0f;
uint32_t SENSORS_LSM6DSV16X_InitGyro(LSM6DSV16X_Object_t *handler);
void SENSORS_CalibrateGyroBias(LSM6DSV16X_Object_t *pObj);

/* Magnetometer LIS2MDL */
LIS2MDL_Object_t lis2mdl_mag_handler;
LIS2MDL_AxesRaw_t magnetic_field_raw = {0};
float lis2mdl_mag_sensitivity = 0.0f;
uint32_t SENSORS_LIS2MDL_InitMag(LIS2MDL_Object_t *handler);

/* Pressure sensor LPS22DF */
LPS22DF_Object_t lps22df_handler;
uint32_t SENSORS_LPS22DF_Init(LPS22DF_Object_t *handler);

/* Humidity sensor SHT40AD1B */
SHT40AD1B_Object_t sht40ad1b_handler;
uint32_t SENSORS_SHT40AD1B_Init(SHT40AD1B_Object_t *handler);

/* Temperature sensor STTS22H */
STTS22H_Object_t stts22h_handler;
uint32_t SENSORS_STTS22H_InitTemp(STTS22H_Object_t *handler);

/**
 * @brief Initialize the ultrasonic sensors.
 * This function configures the necessary peripherals for the ultrasonic sensors,
 * including GPIO and Timer 3 for microsecond timing.
 */
void SENSORS_Init(void) {
	IKS4A1_I2C_INIT(); // Initialize I2C bus for sensors

	/* Environment sensors init */
	/*IKS4A1_ENV_SENSOR_Init(IKS4A1_LPS22DF_0, ENV_PRESSURE);
	IKS4A1_ENV_SENSOR_Init(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY);
	IKS4A1_ENV_SENSOR_Init(IKS4A1_STTS22H_0, ENV_TEMPERATURE);*/

	// LPS22DF pressure sensor
	if (SENSORS_LPS22DF_Init(&lps22df_handler) != SENSOR_OK) {
		assert_param(0);
	}

	// SHT40AD1B humidity sensor
	if (SENSORS_SHT40AD1B_Init(&sht40ad1b_handler) != SENSOR_OK) {
		assert_param(0);
	}

	// STTS22H temperature sensor
	if (SENSORS_STTS22H_InitTemp(&stts22h_handler) != SENSOR_OK) {
		assert_param(0);
	}

	/*
	 * Motion sensors init
	 */

	// Accelerometer LSM6DSV16X
	if (SENSORS_LSM6DSV16X_InitAcc(&lsm6dsv16x_acc_handler) != SENSOR_OK) {
		assert_param(0);
	}

	// Gyroscope LSM6DSV16X
	if (SENSORS_LSM6DSV16X_InitGyro(&lsm6dsv16x_gyro_handler) != SENSOR_OK) {
		assert_param(0);
	}

	// Magnetometer LIS2MDL
	if (SENSORS_LIS2MDL_InitMag(&lis2mdl_mag_handler) != SENSOR_OK) {
		assert_param(0);
	}
}

void SENSORS_TriggerMeasurements(void) {
	static uint32_t environement_counter = 0;

	HAL_GPIO_WritePin(EVT_1_GPIO_Port, EVT_1_Pin, GPIO_PIN_SET);

	SENSORS_MotionMesures_t *motion_msg = pvPortMalloc(sizeof(SENSORS_MotionMesures_t));

	if (motion_msg != NULL) {
		memset(motion_msg, 0, sizeof(SENSORS_MotionMesures_t));  // ras des valeurs
		motion_msg->header.id = SENSORS_MOTION_MEASURES_ID;

		// Get accelerometer raw data
		if (LSM6DSV16X_ACC_GetAxesRaw(&lsm6dsv16x_acc_handler,
				&(motion_msg->acc)) == LSM6DSV16X_OK) {
			// No bias removal for accelerometer
		}

		// Get gyroscope raw data
		if (LSM6DSV16X_GYRO_GetAxesRaw(&lsm6dsv16x_gyro_handler,
				&(motion_msg->gyro)) == LSM6DSV16X_OK) {

			// Remove bias
			motion_msg->gyro.x -= gyro_bias.x;
			motion_msg->gyro.y -= gyro_bias.y;
			motion_msg->gyro.z -= gyro_bias.z;
		}

		// Get magnetometer raw data
		if (LIS2MDL_MAG_GetAxesRaw(&lis2mdl_mag_handler,
				&(motion_msg->mag)) == LIS2MDL_OK) {
			// No bias removal for magnetometer
		}

		// Send mesures to APP task
		if (xQueueSend(xAppLoopQueue, &motion_msg, portMAX_DELAY) != pdPASS) {
			// Queue full, drop the message
			vPortFree(motion_msg);
			assert_param(0);
		}
	} else {
		// Memory allocation failed, drop the message and assert
		assert_param(0);
	}

	HAL_GPIO_WritePin(EVT_1_GPIO_Port, EVT_1_Pin, GPIO_PIN_RESET);

	// Environment measurements every N calls
	environement_counter++;

	if (environement_counter >= (SENSORS_ENVIRONMENTAL_PERIOD)) {
		HAL_GPIO_WritePin(EVT_1_GPIO_Port, EVT_2_Pin, GPIO_PIN_SET);

		environement_counter = 0;
		SENSORS_EnvironementMesures_t *env_msg = pvPortMalloc(sizeof(SENSORS_EnvironementMesures_t));

		if (env_msg != NULL) {
			memset(env_msg, 0, sizeof(SENSORS_EnvironementMesures_t)); // ras des valeurs
			env_msg->header.id = SENSORS_ENV_MEASURES_ID;

			// Get pressure data
			if (LPS22DF_PRESS_GetPressure(&lps22df_handler,
					&(env_msg->pressure)) != LPS22DF_OK) {
				// Error reading pressure
				env_msg->pressure = 0.0f;
			}

			// Get temperature data
			if (STTS22H_TEMP_GetTemperature(&stts22h_handler,
					&(env_msg->temperature)) != STTS22H_OK) {
				// Error reading temperature
				env_msg->temperature = 0.0f;
			}

			// Get humidity data from previous task activation
			if (SHT40AD1B_HUM_GetHumidity(&sht40ad1b_handler,
					&(env_msg->humidity)) != SHT40AD1B_OK) {
				// Error reading humidity
				env_msg->humidity = 0.0f;
			}

			// Start humidity measurement for next task activation
			if (SHT40AD1B_HUM_StartMeasurement(&sht40ad1b_handler) != SHT40AD1B_OK) {
				// Error starting humidity measurement
			}

			// Send mesures to APP task
			if (xQueueSend(xAppLoopQueue, &env_msg, portMAX_DELAY) != pdPASS) {
				// Queue full, drop the message
				vPortFree(env_msg);
				assert_param(0);
			}
		} else {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		HAL_GPIO_WritePin(EVT_1_GPIO_Port, EVT_2_Pin, GPIO_PIN_RESET);
	}


}

void SENSORS_SendMesures(void) {
	AppMessage_typeDef *msg = pvPortMalloc(sizeof(AppMessage_typeDef));

	if (msg != NULL) {
		msg->id = SENSORS_SEND_MEASURES_ID;

		// Send to APP task, no wait
		if (xQueueSend(xAppLoopQueue, &msg, 0) != pdPASS)  {
			// Queue full, drop the message
			vPortFree(msg);
		}
	} else {
		// Memory allocation failed, drop the message
		assert_param(0);
	}
}

uint32_t SENSORS_LIS2MDL_InitMag(LIS2MDL_Object_t *handler)
{
	LIS2MDL_IO_t io_ctx;
	uint8_t id;
	int32_t ret;

	// Magnetometer LIS2MDL
	ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType     = LIS2MDL_I2C_BUS; /* I2C */
	io_ctx.Address     = LIS2MDL_I2C_ADD;
	io_ctx.Init        = IKS4A1_I2C_INIT;
	io_ctx.DeInit      = IKS4A1_I2C_DEINIT;
	io_ctx.ReadReg     = IKS4A1_I2C_READ_REG;
	io_ctx.WriteReg    = IKS4A1_I2C_WRITE_REG;
	io_ctx.GetTick     = IKS4A1_GET_TICK;
	io_ctx.Delay       = IKS4A1_DELAY;

	if (LIS2MDL_RegisterBusIO(handler, &io_ctx) != LIS2MDL_OK) {
		ret = SENSOR_ERROR;
	} else if (LIS2MDL_ReadID(handler, &id) != LIS2MDL_OK) {
		ret = SENSOR_ERROR;
	} else if (id != LIS2MDL_ID) {
		ret = SENSOR_ERROR;
	}

	if (ret == SENSOR_OK) {
		/* Enable the magnetometer */
		if (LIS2MDL_MAG_Enable(handler) != LIS2MDL_OK) {
			ret = SENSOR_ERROR;
		} else if (LIS2MDL_MAG_SetOutputDataRate(handler,
				SENSORS_MAG_ODR_HZ) != LIS2MDL_OK) { /* set mag output data rate */
			ret = SENSOR_ERROR;
		} else if (LIS2MDL_MAG_SetFullScale(handler,
				SENSORS_MAG_FS_GAUSS) != LIS2MDL_OK) { /* Set mag full scale to 50 gauss */
			ret = SENSOR_ERROR;
		} else if (LIS2MDL_MAG_GetSensitivity(handler,
				&lis2mdl_mag_sensitivity) != LIS2MDL_OK) { /* Get LIS2MDL magnetometer actual sensitivity */
			ret = SENSOR_ERROR;
		}
	}

	return ret;
}

uint32_t SENSORS_LSM6DSV16X_InitAcc(LSM6DSV16X_Object_t *handler)
{
	LSM6DSV16X_IO_t io_ctx;
	uint8_t id;
	int32_t ret;

	// Accelerometer LSM6DSV16X
	ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType     = LSM6DSV16X_I2C_BUS; /* I2C */
	io_ctx.Address     = LSM6DSV16X_I2C_ADD_H;
	io_ctx.Init        = IKS4A1_I2C_INIT;
	io_ctx.DeInit      = IKS4A1_I2C_DEINIT;
	io_ctx.ReadReg     = IKS4A1_I2C_READ_REG;
	io_ctx.WriteReg    = IKS4A1_I2C_WRITE_REG;
	io_ctx.GetTick     = IKS4A1_GET_TICK;
	io_ctx.Delay       = IKS4A1_DELAY;

	if (LSM6DSV16X_RegisterBusIO(handler, &io_ctx) != LSM6DSV16X_OK) {
		ret = SENSOR_ERROR;
	} else if (LSM6DSV16X_ReadID(handler, &id) != LSM6DSV16X_OK) {
		ret = SENSOR_ERROR;
	} else if (id != LSM6DSV16X_ID) {
		ret = SENSOR_ERROR;
	}

	if (ret == SENSOR_OK) {
		/* Enable the accelerometer */
		if (LSM6DSV16X_ACC_Enable(handler) != LSM6DSV16X_OK) {
			ret = SENSOR_ERROR;
		} else if (LSM6DSV16X_ACC_SetOutputDataRate(handler, SENSORS_ACC_ODR_HZ)
				!= LSM6DSV16X_OK) { /* set acc output data rate */
			ret = SENSOR_ERROR;
		} else if (LSM6DSV16X_ACC_SetFullScale(handler, SENSORS_ACC_FS_G) != LSM6DSV16X_OK) { /* Set acc full scale to 4 g */
			ret = SENSOR_ERROR;
		} else if (LSM6DSV16X_ACC_GetSensitivity(handler,
				&lsm6dsv16x_acc_sensitivity) != LSM6DSV16X_OK) { /* Get LSM6DSV16X accelerometer actual sensitivity */
			ret = SENSOR_ERROR;
		}
	}

	return ret;
}

uint32_t SENSORS_LSM6DSV16X_InitGyro(LSM6DSV16X_Object_t *handler) {
	LSM6DSV16X_IO_t            io_ctx;
	uint8_t                    id;
	int32_t                    ret;

	// Gyroscope LSM6DSV16X
	ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType     = LSM6DSV16X_I2C_BUS; /* I2C */
	io_ctx.Address     = LSM6DSV16X_I2C_ADD_H;
	io_ctx.Init        = IKS4A1_I2C_INIT;
	io_ctx.DeInit      = IKS4A1_I2C_DEINIT;
	io_ctx.ReadReg     = IKS4A1_I2C_READ_REG;
	io_ctx.WriteReg    = IKS4A1_I2C_WRITE_REG;
	io_ctx.GetTick     = IKS4A1_GET_TICK;
	io_ctx.Delay       = IKS4A1_DELAY;

	if (LSM6DSV16X_RegisterBusIO(handler, &io_ctx) != LSM6DSV16X_OK) {
		ret = SENSOR_ERROR;
	} else if (LSM6DSV16X_ReadID(handler, &id) != LSM6DSV16X_OK) {
		ret = SENSOR_ERROR;
	} else if (id != LSM6DSV16X_ID) {
		ret = SENSOR_ERROR;
	}

	// Inutile de le faire, déjà fait par l'accelero
	//	if (ret == SENSOR_OK) {
	//		if (LSM6DSV16X_Init(handler) != LSM6DSV16X_OK) {
	//			ret = SENSOR_ERROR;
	//		}
	//	}

	if (ret == SENSOR_OK) {
		/* Enable the gyroscope */
		if (LSM6DSV16X_GYRO_Enable(handler) != LSM6DSV16X_OK) {
			ret = SENSOR_ERROR;
		} else 	if (LSM6DSV16X_GYRO_SetOutputDataRate_With_Mode(handler,
				SENSORS_GYRO_ODR_HZ, LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE) != LSM6DSV16X_OK) { /* set gyro output data rate */
			ret = SENSOR_ERROR;
		} else if (LSM6DSV16X_GYRO_SetFullScale(handler, SENSORS_GYRO_FS_DPS) != LSM6DSV16X_OK) { /* Set gyro full scale to 125 dps */
			ret = SENSOR_ERROR;
		}
	}

	if (ret == SENSOR_OK) {
		/* Calibrate gyro bias */
		SENSORS_CalibrateGyroBias(handler);

		/* Get LSM6DSV16X actual sensitivity */
		lsm6dsv16x_gyro_sensitivity = 0.0f;
		if (LSM6DSV16X_GYRO_GetSensitivity(handler, &lsm6dsv16x_gyro_sensitivity) != LSM6DSV16X_OK) {
			ret = SENSOR_ERROR;
		}
	}

	return ret;
}

uint32_t SENSORS_LPS22DF_Init(LPS22DF_Object_t *handler) {
	LPS22DF_IO_t            io_ctx;
	uint8_t                 id;
	int32_t                 ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType     = LPS22DF_I2C_BUS; /* I2C */
	io_ctx.Address     = LPS22DF_I2C_ADD_H;
	io_ctx.Init        = IKS4A1_I2C_INIT;
	io_ctx.DeInit      = IKS4A1_I2C_DEINIT;
	io_ctx.ReadReg     = IKS4A1_I2C_READ_REG;
	io_ctx.WriteReg    = IKS4A1_I2C_WRITE_REG;
	io_ctx.GetTick     = IKS4A1_GET_TICK;
	io_ctx.Delay       = IKS4A1_DELAY;

	if (LPS22DF_RegisterBusIO(handler, &io_ctx) != LPS22DF_OK) {
		ret = SENSOR_ERROR;
	} else if (LPS22DF_ReadID(handler, &id) != LPS22DF_OK) {
		ret = SENSOR_ERROR;
	} else if (id != LPS22DF_ID) {
		ret = SENSOR_ERROR;
	}

	if (ret == SENSOR_OK) {
		/* Enable the pressure sensor */
		if (LPS22DF_PRESS_Enable(handler) != LPS22DF_OK) {
			ret = SENSOR_ERROR;
		} else if (LPS22DF_PRESS_SetOutputDataRate(handler,
				SENSORS_PRESS_ODR_HZ) != LPS22DF_OK) { /* set pressure output data rate */
			ret = SENSOR_ERROR;
		}
	}

	return SENSOR_OK;
}

uint32_t SENSORS_SHT40AD1B_Init(SHT40AD1B_Object_t *handler) {
	SHT40AD1B_IO_t            io_ctx;
	uint8_t                   id;
	int32_t                   ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType     = SHT40AD1B_I2C_BUS; /* I2C */
	io_ctx.Address     = SHT40AD1B_I2C_ADDRESS;
	io_ctx.Init        = IKS4A1_I2C_INIT;
	io_ctx.DeInit      = IKS4A1_I2C_DEINIT;
	io_ctx.Read        = IKS4A1_I2C_READ;
	io_ctx.Write       = IKS4A1_I2C_WRITE;
	io_ctx.GetTick     = IKS4A1_GET_TICK;
	io_ctx.Delay       = IKS4A1_DELAY;

	if (SHT40AD1B_RegisterBusIO(handler, &io_ctx) != SHT40AD1B_OK) {
		ret = SENSOR_ERROR;
	} else if (SHT40AD1B_ReadID(handler, &id) != SHT40AD1B_OK) {
		ret = SENSOR_ERROR;
	} else if (id != SHT40AD1B_ID) {
		ret = SENSOR_ERROR;
	}

	if (ret == SENSOR_OK) {
		/* Enable the humidity sensor */
		if (SHT40AD1B_HUM_Enable(handler) != SHT40AD1B_OK) {
			ret = SENSOR_ERROR;
		} else if (SHT40AD1B_HUM_SetOutputDataRate(handler,
				SENSORS_HUM_ODR_HZ) != SHT40AD1B_OK) { /* set humidity output data rate */
			ret = SENSOR_ERROR;
		}
	}

	return SENSOR_OK;
}

uint32_t SENSORS_STTS22H_InitTemp(STTS22H_Object_t *handler) {
	STTS22H_IO_t io_ctx;
	uint8_t id;
	int32_t ret = SENSOR_OK;

	/* Configure the driver */
	io_ctx.BusType = STTS22H_I2C_BUS; /* I2C */
	io_ctx.Address = STTS22H_I2C_ADD_H;
	io_ctx.Init = IKS4A1_I2C_INIT;
	io_ctx.DeInit = IKS4A1_I2C_DEINIT;
	io_ctx.ReadReg = IKS4A1_I2C_READ_REG;
	io_ctx.WriteReg = IKS4A1_I2C_WRITE_REG;
	io_ctx.GetTick = IKS4A1_GET_TICK;
	io_ctx.Delay = IKS4A1_DELAY;

	if (STTS22H_RegisterBusIO(handler, &io_ctx) != STTS22H_OK) {
		ret = SENSOR_ERROR;
	} else if (STTS22H_ReadID(handler, &id) != STTS22H_OK) {
		ret = SENSOR_ERROR;
	} else if (id != STTS22H_ID) {
		ret = SENSOR_ERROR;
	}

	if (ret == SENSOR_OK) {
		/* Enable the temperature sensor */
		if (STTS22H_TEMP_Enable(handler) != STTS22H_OK) {
			ret = SENSOR_ERROR;
		} else if (STTS22H_TEMP_SetOutputDataRate(handler,
				SENSORS_TEMP_ODR_HZ) != STTS22H_OK) { /* set temperature output data rate */
			ret = SENSOR_ERROR;
		}
	}

	return SENSOR_OK;
}

void SENSORS_CalibrateGyroBias(LSM6DSV16X_Object_t *pObj)
{
	LSM6DSV16X_AxesRaw_t raw_angular_rate; // Pour stocker les valeurs brutes (dps * 1000)
	int32_t sum_x = 0;
	int32_t sum_y = 0;
	int32_t sum_z = 0;
	uint32_t i;

	// Le capteur doit être absolument immobile pendant cette étape

	// Lecture d'un grand nombre d'échantillons
	for (i = 0; i < NUM_CALIBRATION_SAMPLES; i++)
	{
		// 1. Lire les données
		// La fonction LSM6DSV16X_GYRO_GetAxes(pObj, raw_angular_rate) est supposée lire les données
		// brutes du gyroscope et les mettre à l'échelle en milli-dps (mdps) ou dps*1000 selon la BSP
		if (LSM6DSV16X_GYRO_GetAxesRaw(pObj, &raw_angular_rate) == LSM6DSV16X_OK)
		{
			sum_x += raw_angular_rate.x;
			sum_y += raw_angular_rate.y;
			sum_z += raw_angular_rate.z;
		}

		// Délai minimal pour s'assurer que l'ODR est respecté.
		// Par exemple, si ODR = 104 Hz, l'échantillon arrive toutes les 9.6ms.
		IKS4A1_DELAY(10); // Attendre un peu moins que la période d'échantillonnage pour être sûr
	}

	// 2. Calculer la moyenne (le biais)
	gyro_bias.x = sum_x / NUM_CALIBRATION_SAMPLES;
	gyro_bias.y = sum_y / NUM_CALIBRATION_SAMPLES;
	gyro_bias.z = sum_z / NUM_CALIBRATION_SAMPLES;
}

SENSORS_TriaxeValues_t SENSORS_ACC_RawtoG(LSM6DSV16X_AxesRaw_t *raw) {
	SENSORS_TriaxeValues_t acc_g;
	// Convertir les valeurs brutes en g (acceleration de la gravité)
	// Pour info, la sensibilité pour 4g est de 0.012207 mg/LSB

	acc_g.x = ((float)raw->x) * lsm6dsv16x_acc_sensitivity / 1000.0f;
	acc_g.y = ((float)raw->y) * lsm6dsv16x_acc_sensitivity / 1000.0f;;
	acc_g.z = ((float)raw->z) * lsm6dsv16x_acc_sensitivity / 1000.0f;;

	return acc_g;
}

SENSORS_TriaxeValues_t SENSORS_GYRO_RawtoDPS(LSM6DSV16X_AxesRaw_t *raw) {
	SENSORS_TriaxeValues_t gyro_dps;
	// Convertir les valeurs brutes en dps (degrees per second)
	// Pour info, la sensibilité pour 500 dps est de 17.5 mdps/LSB

	gyro_dps.x = ((float)raw->x) * lsm6dsv16x_gyro_sensitivity / 1000.0f;
	gyro_dps.y = ((float)raw->y) * lsm6dsv16x_gyro_sensitivity / 1000.0f;;
	gyro_dps.z = ((float)raw->z) * lsm6dsv16x_gyro_sensitivity / 1000.0f;;

	return gyro_dps;
}

SENSORS_TriaxeValues_t SENSORS_MAG_RawtoMilliGauss(LIS2MDL_AxesRaw_t *raw) {
	SENSORS_TriaxeValues_t mag_gauss;
	// Convertir les valeurs brutes en gauss
	// Pour info, la sensibilité pour 50 gauss est de 1.5 mgauss/LSB

	mag_gauss.x = ((float) raw->x) * lis2mdl_mag_sensitivity;
	mag_gauss.y = ((float) raw->y) * lis2mdl_mag_sensitivity;
	mag_gauss.z = ((float) raw->z) * lis2mdl_mag_sensitivity;

	return mag_gauss;
}

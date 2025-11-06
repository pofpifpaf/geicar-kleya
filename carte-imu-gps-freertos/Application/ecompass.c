/*
 * ecompass.c
 *
 *  Created on: Oct 23, 2025
 *      Author: dimercur
 */

#include "ecompass.h"
#include "configuration.h"

#include "iks4a1_motion_sensors.h"
#include "motion_ec.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include "tasks.h" // for xAppLoopQueue

#define MOTION_SENSOR_Axes_t IKS4A1_MOTION_SENSOR_Axes_t

char acc_orientation[4];
char gyr_orientation[4];
char mag_orientation[4];

static float AccMatrix[3][3];
static float GyrMatrix[3][3];
static float MagMatrix[3][3];

/* Private function prototypes -----------------------------------------------*/
static int32_t calc_heading(float *heading, float v_head[]);
static void calc_matrix(char orientation[], float matrix[][3]);
static void q_conjug(float q_conj[], float q_src[]);
static void q_multiply(float q_res[], float q_a[], float q_b[]);
static void transform_orientation(const SENSORS_TriaxeValues_t *input, float output[], float matrix[][3]);
static void v_rotate(float v_new[], float q_rot[], float v_old[]);

#define DWT_CONTROL (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)
#define DWT_CR_CYCCNTENA_Msk (1UL << 0)
#define DEMCR_TRCENA_Msk     (1UL << 24)

#define CYCLES_PER_MS 80000.0 // Utilisez un flottant pour le calcul en double

double duration_ms;
/**
 * @brief Active le compteur de cycles DWT
 */
void DWT_Init(void)
{
    // 1. Autoriser l'accès au DWT (Trace Enable)
    DEMCR |= DEMCR_TRCENA_Msk;

    // 2. Réinitialiser le compteur de cycles
    DWT_CYCCNT = 0;

    // 3. Démarrer le compteur
    DWT_CONTROL |= DWT_CR_CYCCNTENA_Msk;
}

/**
 * @brief  Get accelerometer sensor orientation
 * @param  Orientation Pointer to sensor orientation
 * @retval None
 */
void BSP_SENSOR_ACC_GetOrientation(char *Orientation)
{
	Orientation[0] = 's';
	Orientation[1] = 'e';
	Orientation[2] = 'u';
}

/**
 * @brief  Get gyroscope sensor orientation
 * @param  Orientation Pointer to sensor orientation
 * @retval None
 */
void BSP_SENSOR_GYR_GetOrientation(char *Orientation)
{
	Orientation[0] = 's';
	Orientation[1] = 'e';
	Orientation[2] = 'u';
}

/**
 * @brief  Get magnetometer sensor orientation
 * @param  Orientation Pointer to sensor orientation
 * @retval None
 */
void BSP_SENSOR_MAG_GetOrientation(char *Orientation)
{
	Orientation[0] = 'n';
	Orientation[1] = 'e';
	Orientation[2] = 'u';
}

/**
 * @brief  Calculate heading
 * @param  heading Device heading in range <0, 360) degrees
 * @param  v_head  Device orientation vector for heading
 * @retval 1 in case of success, 0 otherwise
 */
static int32_t calc_heading(float *heading, float v_head[])
{
	const float tol_deg = 5.0f; /* Tolerance [deg] */
	float tolerance = sinf(tol_deg * M_PI / 180.0f);

	if (v_head[0] > (-1.0f) * tolerance && v_head[0] < tolerance
			&& v_head[1] > (-1.0f) * tolerance && v_head[1] < tolerance)
	{
		*heading = 0.0f;
		return 0; /* Device is pointing up or down - it is impossible to evaluate heading */
	}

	else
	{
		*heading = atan2f(v_head[0], v_head[1]) * 180.0f / M_PI;
		*heading = floorf(*heading * 100.0f + 0.5f) / 100.0f;          /* Rounds number to two decimal digits */
		*heading = (*heading < 0.0f) ? (*heading + 360.0f) : *heading; /* Change negative value to be in range <0,360) */
		return 1;
	}
}

/**
 * @brief  Calculate orientation transformation matrix
 * @param  orientation  Sensor orientation
 * @param  matrix  Sensor frame to device frame transformation matrix
 * @retval None
 */
static void calc_matrix(char orientation[], float matrix[][3])
{
	matrix[0][0] = (orientation[0] == 'e') ?  1
			: (orientation[0] == 'w') ? -1
					:                            0;

	matrix[0][1] = (orientation[1] == 'e') ?  1
			: (orientation[1] == 'w') ? -1
					:                            0;

	matrix[0][2] = (orientation[2] == 'e') ?  1
			: (orientation[2] == 'w') ? -1
					:                            0;

	matrix[1][0] = (orientation[0] == 'n') ?  1
			: (orientation[0] == 's') ? -1
					:                            0;

	matrix[1][1] = (orientation[1] == 'n') ?  1
			: (orientation[1] == 's') ? -1
					:                            0;

	matrix[1][2] = (orientation[2] == 'n') ?  1
			: (orientation[2] == 's') ? -1
					:                            0;

	matrix[2][0] = (orientation[0] == 'u') ?  1
			: (orientation[0] == 'd') ? -1
					:                            0;

	matrix[2][1] = (orientation[1] == 'u') ?  1
			: (orientation[1] == 'd') ? -1
					:                            0;

	matrix[2][2] = (orientation[2] == 'u') ?  1
			: (orientation[2] == 'd') ? -1
					:                            0;
}

/**
 * @brief  Create conjugated quaternion
 * @param  q_conj  Conjugated quaternion
 * @param  q_src   Source quaternion
 * @retval None
 */
static void q_conjug(float q_conj[], float q_src[])
{
	q_conj[0] = (-1.0f) * q_src[0];
	q_conj[1] = (-1.0f) * q_src[1];
	q_conj[2] = (-1.0f) * q_src[2];
	q_conj[3] =           q_src[3];

	return;
}

/**
 * @brief  Do quaternion multiplication
 * @param  q_res  Quaternion multiplication result: q_res = q_a q_b
 * @param  q_a    Quaternion A
 * @param  q_b    Quaternion B
 * @retval None
 */
static void q_multiply(float q_res[], float q_a[], float q_b[])
{
	q_res[0] =
			q_a[3] * q_b[0]
						 + q_a[0] * q_b[3]
										+ q_a[1] * q_b[2]
													   - q_a[2] * q_b[1]
																	  ;

	q_res[1] =
			q_a[3] * q_b[1]
						 + q_a[1] * q_b[3]
										+ q_a[2] * q_b[0]
													   - q_a[0] * q_b[2]
																	  ;

	q_res[2] =
			q_a[3] * q_b[2]
						 + q_a[2] * q_b[3]
										+ q_a[0] * q_b[1]
													   - q_a[1] * q_b[0]
																	  ;

	q_res[3] =
			q_a[3] * q_b[3]
						 - q_a[0] * q_b[0]
										- q_a[1] * q_b[1]
													   - q_a[2] * q_b[2]
																	  ;

	return;
}

/**
 * @brief  Transform orientation from sensor frame to device frame
 * @param  input  Input data (sensor frame)
 * @param  output  Output data (device frame)
 * @param  matrix  Sensor frame to device frame transformation matrix
 * @retval None
 */
static void transform_orientation(const SENSORS_TriaxeValues_t *input, float output[], float matrix[][3])
{
	output[0] = matrix[0][0] * (float)input->x  +  matrix[0][1] * (float)input->y  +  matrix[0][2] * (float)input->z;
	output[1] = matrix[1][0] * (float)input->x  +  matrix[1][1] * (float)input->y  +  matrix[1][2] * (float)input->z;
	output[2] = matrix[2][0] * (float)input->x  +  matrix[2][1] * (float)input->y  +  matrix[2][2] * (float)input->z;

	return;
}

/**
 * @brief  Rotate vector using quaternion. Uses following equation:
 *           v_new = q_rot v_old q_rot_inv
 *
 * @param  v_new  Vector after rotation
 * @param  q_rot  Rotation quaternion
 * @param  v_old  Vector before rotation
 * @retval None
 */
static void v_rotate(float v_new[], float q_rot[], float v_old[])
{
	float q_old[4];
	float q_new[4];
	float q_rot_inv[4];
	float q_temp[4];

	/* Create quaternion from old position vector */
	q_old[0] = v_old[0];
	q_old[1] = v_old[1];
	q_old[2] = v_old[2];
	q_old[3] = 0.0f;

	q_conjug(q_rot_inv, q_rot);
	q_multiply(q_temp, q_old, q_rot_inv);
	q_multiply(q_new, q_rot, q_temp);

	v_new[0] = q_new[0];
	v_new[1] = q_new[1];
	v_new[2] = q_new[2];

	return;
}

/**
 * @brief  Transform the accelerometer and magnetometer data orientation
 * @param  acc_in  accelerometer data (sensor frame)
 * @param  mag_in  magnetometer data (sensor frame)
 * @param  acc_out  accelerometer data (device frame)
 * @param  mag_out  magnetometer data (device frame)
 * @retval None
 */
//void MotionEC_manager_transform_orientation(MOTION_SENSOR_Axes_t *acc_in, MOTION_SENSOR_Axes_t *mag_in, float acc_out[],
//		float mag_out[])
//{
//	transform_orientation(acc_in, acc_out, AccMatrix);
//	transform_orientation(mag_in, mag_out, MagMatrix);
//}

/**
 * @brief  Run E-Compass algorithm (accelerometer and magnetometer data fusion)
 * @param  data_in  Structure containing input data
 * @param  data_out  Structure containing output data
 * @retval None
 */
void MotionEC_manager_run(MEC_input_t *data_in, MEC_output_t *data_out)
{
	MotionEC_Run(data_in, data_out);
}

/**
 * @brief  Calculate E-Compass device heading
 * @param  quaternion  E-Compass device orientation quaternion
 * @param  heading  E-Compass device heading
 * @param  heading_valid  E-Compass device heading is valid
 * @retval None
 */
void MotionEC_manager_calc_heading(float quaternion[], float *heading, int32_t *heading_valid)
{
	float v_base[3] = {0.0, 1.0, 0.0};
	float v_head[3];

	v_rotate(v_head, quaternion, v_base);
	*heading_valid = calc_heading(heading, v_head);
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionEC_manager_get_version(char *version, int32_t *length)
{
	*length = (int)MotionEC_GetLibVersion(version);
}

/**
 * @brief  Initialize and reset the MotionEC engine
 * @param  freq  Sensors sampling frequency [Hz]
 * @retval None
 */
uint32_t ECOMPASS_Init(float freq) {
	// Initialization code for the eCompass module

	BSP_SENSOR_ACC_GetOrientation(acc_orientation);
	BSP_SENSOR_GYR_GetOrientation(gyr_orientation);
	BSP_SENSOR_MAG_GetOrientation(mag_orientation);

	calc_matrix(acc_orientation, AccMatrix);
	calc_matrix(gyr_orientation, GyrMatrix);
	calc_matrix(mag_orientation, MagMatrix);

	/* Je ne sais pas ce que c'est */
	MotionEC_Initialize(MEC_MCU_STM32, &freq);
	MotionEC_SetOrientationEnable(MEC_ENABLE);
	MotionEC_SetVirtualGyroEnable(MEC_ENABLE);
	MotionEC_SetGravityEnable(MEC_ENABLE);
	MotionEC_SetLinearAccEnable(MEC_ENABLE);

	return 0; // Return 0 on success
}

void ECOMPASS_ProcessMessage(const ECOMPASS_SensorsValues_t *msg) {
	// Process the sensor values to compute heading, pitch, and roll

	uint32_t start_cycles;
	uint32_t end_cycles;
	uint32_t duration_cycles;

	MEC_input_t  data_in;
	MEC_output_t data_out;

	float heading;
	int32_t heading_valid;

	HAL_GPIO_WritePin(EVT_4_GPIO_Port, EVT_4_Pin, GPIO_PIN_SET);

	/* Do sensor orientation transformation */
	transform_orientation(&(msg->acc), data_in.acc, AccMatrix);
	//transform_orientation(&(msg->gyro), data_in.gyr, GyrMatrix);
	transform_orientation(&(msg->mag), data_in.mag, MagMatrix);

	vPortFree((void*)msg); // Free the message after extracting data)

	/* Convert raw accelerometer data from [mg] to [g] */
//	data_in.acc[0] = data_in.acc[0] / 1000.0f; /* East */
//	data_in.acc[1] = data_in.acc[1] / 1000.0f; /* North */
//	data_in.acc[2] = data_in.acc[2] / 1000.0f; /* Up */

	/* Convert compensated magnetometer data from [mGauss] to [uT / 50], [mGauss / 5] */
	data_in.mag[0] = data_in.mag[0] / 5.0f; /* East */
	data_in.mag[1] = data_in.mag[1] / 5.0f; /* North */
	data_in.mag[2] = data_in.mag[2] / 5.0f; /* Up */

	/* Delta time [s] */
	data_in.deltatime_s = (float)msg->elapsedTimeMs / 1000.0f;

	/* Run E-Compass algorithm */
	//BSP_LED_On(LED2);
	start_cycles = DWT_CYCCNT;
	MotionEC_manager_run(&data_in, &data_out);
	end_cycles = DWT_CYCCNT;
	duration_cycles = end_cycles - start_cycles;
	duration_ms = (double)duration_cycles / CYCLES_PER_MS;
	/* Write data to output stream */

	MotionEC_manager_calc_heading(data_out.quaternion, &heading, &heading_valid);

	ECOMPASS_Attitude_t *attitude_msg = pvPortMalloc(sizeof(ECOMPASS_Attitude_t));
	if (attitude_msg != NULL) {
		memset(attitude_msg, 0, sizeof(ECOMPASS_Attitude_t)); // ras des valeurs
		attitude_msg->header.id = ECOMPASS_ATTITUDE_DATA_ID;
		attitude_msg->values.heading = heading;
		attitude_msg->values.heading_valid = heading_valid;
		attitude_msg->values.yaw = data_out.euler[0];
		attitude_msg->values.pitch = data_out.euler[1];
		attitude_msg->values.roll = data_out.euler[2];

		if (xQueueSend(xAppLoopQueue, &attitude_msg, portMAX_DELAY) != pdPASS) {
			// Queue full, drop the message
			vPortFree(attitude_msg);
			assert_param(0);
		}
	} else {
		// Memory allocation failed, drop the message and assert
		assert_param(0);
	}

	HAL_GPIO_WritePin(EVT_4_GPIO_Port, EVT_4_Pin, GPIO_PIN_RESET);
}

void ECOMPASS_SendMesures(void) {
// This function can be used to trigger sending eCompass measures if needed
	AppMessage_typeDef *msg = pvPortMalloc(sizeof(AppMessage_typeDef));

	if (msg != NULL) {
		msg->id = ECOMPASS_SEND_MEASURES_ID;

		// Send to APP task, no wait
		if (xQueueSend(xAppLoopQueue, &msg, portMAX_DELAY) != pdPASS) {
			// Queue full, drop the message
			vPortFree(msg);
		}
	} else {
		// Memory allocation failed, drop the message
		assert_param(0);
	}
}

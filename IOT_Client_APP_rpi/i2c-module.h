/**
  ******************************************************************************
  * @file   i2c_Module.h
  * @author Mario López Parejo (mario.lopezp@alumnos.upm.es) & Juan Morales Sáez (j.msaez@alumnos.upm.es)
  * @brief  I2C Module header.
  *
  * @note   Embedded System Design with Raspberry Pi (ESD-RPI)
  *         This module manages the i2c processing in the IOT Client.
  ******************************************************************************
*/

#ifndef I2C_MODULE_H_
#define I2C_MODULE_H_

	/* Includes ------------------------------------------------------------------*/
	/* Exported types ------------------------------------------------------------*/
	/* Exported constants --------------------------------------------------------*/
	/* Exported macro ------------------------------------------------------------*/
	/* Exported Functions --------------------------------------------------------*/

	extern int init_i2c_driver (void);

	extern int get_accelerometer_data (float *acceleration_data);

	extern int get_color_sensor_data (float *color_sensor_data);

	extern int deactivate_i2c_driver (void);

#endif /* I2C_MODULE_H_ */

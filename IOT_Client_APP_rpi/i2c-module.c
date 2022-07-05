/**
  ******************************************************************************
  * @file   i2c_Module.c
  * @author Mario López Parejo (mario.lopezp@alumnos.upm.es) & Juan Morales Sáez (j.msaez@alumnos.upm.es)
  * @brief  I2C Module.
  *
  * @note   Embedded System Design with Raspberry Pi (ESD-RPI)
  *         This module manages the i2c processing in the IOT Client.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "i2c-module.h" // Module header

#include <stdint.h> // Import uint8_t ...
#include <stdio.h>  // Import prinf ...

#include <linux/i2c.h> // Import i2c dependencies
#include <linux/i2c-dev.h> // Import i2c dependencies

#include <sys/ioctl.h>
#include <sys/stat.h> // Import open function

#include <fcntl.h> // Import O_RDWR.


/* Private define ------------------------------------------------------------*/

// I2C Definition

#define write_op_length 2

#define i2c_driver_path "/dev/i2c-1"

// Accelerometer Defines

#define AC_SLAVE_ADDR 0x68

#define AC_POWER_MANAGMENT_REGISTER 0x6B
#define AC_INT_ENA_REGISTER         0X38
#define AC_RATE_DIV_REGISTER	    0x19
#define AC_CONF_REGISTER            0x1A
#define AC_FIFO_DISABLE_REGISTER	0x23
#define AC_SET_SCALE_REGISTER       0x1C

#define AC_POWER_MANAGMENT_REGISTER_VALUE 0x01
#define AC_INT_ENA_REGISTER_VALUE         0x01
#define AC_RATE_DIV_REGISTER_VALUE        0x07
#define AC_CONF_REGISTER_VALUE            0x00
#define AC_FIFO_DISABLE_REGISTER_VALUE    0x00
#define AC_SET_SCALE_REGISTER_VALUE       0x08

#define AC_POWER_MANAGMENT_REGISTER_RESET    0x80
#define AC_POWER_MANAGMENT_REGISTER_POWEROFF 0x40

#define AC_MEAS_REGISTER 0x3B
#define AC_MEAS_REGISTER_LENGTH 0x06

// Color Sensor Defines

#define CS_SLAVE_ADDR 0x29

#define CS_ENABLE_REGISTER      (0x80 |0x00)
#define CS_RGBC_TIMING_REGISTER  (0x80 |0x01)
#define CS_WAIT_TIME_REGISTER	 (0x80 |0x03)
#define CS_CONTROL_REGISTER  (0x80 |0x0F)

#define CS_ENABLE_REGISTER_VALUE   0x03
#define CS_RGBC_TIMING_REGISTER_VALUE 0x00
#define CS_WAIT_TIME_REGISTER_VALUE   0xFF
#define CS_CONTROL_REGISTER_VALUE	  0x00

#define CS_MEAS_REGISTER (0x80 | 0x14)
#define CS_MEAS_REGISTER_LENGTH 0x08


/* Private variables----------------------------------------------------------*/

static int fd;


/* Function prototypes -------------------------------------------------------*/

int init_i2c_driver (void);

int get_accelerometer_data (float *acceleration_data);

int get_color_sensor_data (float *color_sensor_data);

int deactivate_i2c_driver (void);

float accelerometer_data_conversion (char MSB, char LSB);

float color_sensor_data_conversion(char MSB, char LSB);

int configure_accelerometer (int fd);

int configure_color_sensor (int fd);

void send_I2C_message (int fd, uint8_t slave_Addr, uint8_t register_addr, uint8_t register_value);

int read_I2C_message(int fd, int length, uint8_t slave_Addr, uint8_t register_addr, uint8_t *read_bytes);


/* Functions definition ------------------------------------------------------*/

int init_i2c_driver (void)
{
	char i2cPath[15];
	sprintf(i2cPath, i2c_driver_path);

	fd = open(i2cPath, 	O_RDWR);

	if (fd == -1) { perror("\n I2C_Module > Error while i2c resources allocation."); return -1; }

	if (configure_accelerometer (fd) == -1) {
		perror("\n I2C_Module > Error while accelerometer configuration.");
		return -1;
	}else{
		printf("\n I2C_Module >Se ha configurado el acelerómetro ");
	}

	if (configure_color_sensor(fd) == -1) {
		perror("\n I2C_Module > Error while color sensor configuration.");
		return -1;
	}else{
		printf("\n I2C_Module >Se ha configurado el sensor de color ");
	}

	return 0;
}

int get_accelerometer_data (float *acceleration_data)
{
	uint8_t accelerometer_read_bytes[AC_MEAS_REGISTER_LENGTH];

	if ( read_I2C_message(fd, AC_MEAS_REGISTER_LENGTH, AC_SLAVE_ADDR, AC_MEAS_REGISTER, accelerometer_read_bytes) == -1 ) return -1;

	acceleration_data[0] = accelerometer_data_conversion(accelerometer_read_bytes[0], accelerometer_read_bytes[1]);
	acceleration_data[1] = accelerometer_data_conversion(accelerometer_read_bytes[2], accelerometer_read_bytes[3]);
	acceleration_data[2] = accelerometer_data_conversion(accelerometer_read_bytes[4], accelerometer_read_bytes[5]);

	return 0;
}

int get_color_sensor_data (float *color_sensor_data)
{
	uint8_t color_sensor_read_bytes[CS_MEAS_REGISTER_LENGTH];
	if ( read_I2C_message(fd, CS_MEAS_REGISTER_LENGTH, CS_SLAVE_ADDR, CS_MEAS_REGISTER, color_sensor_read_bytes) == -1) return -1;

	color_sensor_data[0] = color_sensor_data_conversion(color_sensor_read_bytes[0], color_sensor_read_bytes[1]);
	color_sensor_data[1] = color_sensor_data_conversion(color_sensor_read_bytes[2], color_sensor_read_bytes[3]);
	color_sensor_data[2] = color_sensor_data_conversion(color_sensor_read_bytes[4], color_sensor_read_bytes[5]);
	color_sensor_data[3] = color_sensor_data_conversion(color_sensor_read_bytes[6], color_sensor_read_bytes[7]);

	return 0;
}

int deactivate_i2c_driver (void)
{
	printf("\n I2C_Module > Setting the accelerometer to sleep mode");

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_POWER_MANAGMENT_REGISTER, AC_POWER_MANAGMENT_REGISTER_RESET);    //device reset
	send_I2C_message(fd, AC_SLAVE_ADDR, AC_POWER_MANAGMENT_REGISTER, AC_POWER_MANAGMENT_REGISTER_POWEROFF); //device sleep

	printf("\n I2C_Module > Freeing up allocated resources.");
	return close(fd);
}


float accelerometer_data_conversion (char MSB, char LSB)
{
	//El acelerómetro devuelve los datos con signo.
	int16_t acceleration_measure = ((MSB << 8) | LSB);

	return (float) acceleration_measure / (8192); // Desplazar bits ?

}

float color_sensor_data_conversion(char MSB, char LSB)
{
	//El sensor de color devuelve los datos sin signo.
	uint16_t color_sensor_measure = ((MSB << 8) | LSB);

	return (float) (color_sensor_measure * 100) / (65536); // Desplazar bits ?
}


int configure_color_sensor (int fd)
{
	send_I2C_message(fd, CS_SLAVE_ADDR, CS_ENABLE_REGISTER, CS_ENABLE_REGISTER_VALUE); 	// Enable AEN and PON so the device starts working.

	send_I2C_message(fd, CS_SLAVE_ADDR, CS_RGBC_TIMING_REGISTER, CS_RGBC_TIMING_REGISTER_VALUE); // Internal integration time of 700 ms.

	send_I2C_message(fd, CS_SLAVE_ADDR, CS_WAIT_TIME_REGISTER, CS_WAIT_TIME_REGISTER_VALUE); // Wait time of 2.4 ms

	send_I2C_message(fd, CS_SLAVE_ADDR, CS_CONTROL_REGISTER, CS_CONTROL_REGISTER_VALUE); // Set gain to 1

	return 0;
}

int configure_accelerometer (int fd)
{
	send_I2C_message(fd, AC_SLAVE_ADDR, AC_POWER_MANAGMENT_REGISTER, AC_POWER_MANAGMENT_REGISTER_VALUE); // Power management register.

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_INT_ENA_REGISTER, AC_INT_ENA_REGISTER_VALUE); // Interrupt enable register.

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_RATE_DIV_REGISTER, AC_RATE_DIV_REGISTER_VALUE); // Sample rate divider register.

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_CONF_REGISTER, AC_CONF_REGISTER_VALUE); // Configuration register

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_FIFO_DISABLE_REGISTER, AC_FIFO_DISABLE_REGISTER_VALUE); // Disable the FIFO -> real time measurements

	send_I2C_message(fd, AC_SLAVE_ADDR, AC_SET_SCALE_REGISTER, AC_SET_SCALE_REGISTER_VALUE); // Set the scale to +/- 4g

	return 0;
}

void send_I2C_message (int fd, uint8_t slave_Addr, uint8_t register_addr, uint8_t register_value)
{
	struct i2c_msg messages[2];
	struct i2c_rdwr_ioctl_data packet;

	uint8_t write_bytes[2];

	write_bytes[0] = register_addr;
	write_bytes[1] = register_value;

	messages[0].addr  = slave_Addr;
	messages[0].flags = 0;
	messages[0].len   = write_op_length;
	messages[0].buf   = write_bytes; // Pointer to the data bytes to be written.

	packet.msgs = messages;
	packet.nmsgs = 1;

	if ( ioctl(fd, I2C_RDWR, &packet) == -1 ) perror("\n I2C_Module > Error during i2c write operation."); // Execute write operation
}

int read_I2C_message(int fd, int length, uint8_t slave_Addr, uint8_t register_addr, uint8_t *read_bytes)
{
	int status = 0;
	struct i2c_msg messages[2];
	struct i2c_rdwr_ioctl_data packet;

	uint8_t write_bytes;

	write_bytes       = register_addr;
	messages[0].addr  = slave_Addr;
	messages[0].flags = 0;
	messages[0].len   = 1;            // We define the read operation
	messages[0].buf   = &write_bytes; // Pointer to the data bytes to be written.

	messages[1].addr  = slave_Addr;
	messages[1].flags = I2C_M_RD;    // We define the read operation
	messages[1].len   = length;
	messages[1].buf   = read_bytes;  // Pointer to the data bytes to be written.

	packet.msgs = messages;
	packet.nmsgs = 2;

	if ( ioctl(fd, I2C_RDWR, &packet) == -1 ){
		perror("\n I2C_Module > Error during i2c write operation."); // Execute read operation
		status = -1;
	}
	return status;
}

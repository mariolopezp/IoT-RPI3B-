/**
  ******************************************************************************
  * @file   IOTClient.c
  * @author Mario López Parejo (mario.lopezp@alumnos.upm.es) & Juan Morales Sáez (j.msaez@alumnos.upm.es)
  * @brief  IOTClient Module.
  *
  * @note   Embedded System Design with Raspberry Pi (ESD-RPI)
  *         This module manages the IOT client using I2C_Module and UDP socket.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "i2c-module.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <netinet/in.h>

/* Private define ------------------------------------------------------------*/

#define PORT        5784
#define MAXLINE     1024
#define SAMPLE_TIME 1       //1 sample per second
#define SEND_TIME   10      //RPI sends data every 10 s.

/* Private variables----------------------------------------------------------*/

int end_of_program;
int sockfd;

/* Function prototypes -------------------------------------------------------*/

void SIGINT_handler (int signal);

/* Functions definition ------------------------------------------------------*/

// Driver code
int main(int argc, char *argv[]) {

	uint measure_rate, send_data_to_server_period;

	float accelerometer_data[3], color_sensor_data[4];
	float data_to_send[7];



    int n, len, op_code;
    char buffer[MAXLINE];
    struct sockaddr_in servaddr;

//	if ( argc < 2 ) {
//        perror("\n IOTClient@main #> Insufficient arguments number. ");
//        perror(" Remember: \n\t -0- Measure rate in seconds (s) \n\t -1- Time interval for sending information to server in seconds (s)");
//        exit(EXIT_FAILURE);
//	}
//
//	measure_rate = strtol(argv[0], NULL, 10);
//
//	send_data_to_server_period = strtol(argv[1], NULL, 10);

    signal(SIGINT, SIGINT_handler);
    end_of_program = 0;

    uint8_t time_cnt = 1;

    printf("\n IOTClient@main > Initializing socket.");

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("\n IOTClient@main #> Error while socket creation.");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr("192.168.0.148"); //IP address of the server.

    printf("\n IOTClient@main > Initializing i2c module.");

    // Initializing i2c module
    if ( init_i2c_driver() < 0 ) {
        perror("\n IOTClient@main #> Error while i2c initialization.");
        exit(EXIT_FAILURE);
    }

    printf("\n IOTClient@main > Initialization finished. Start getting data from sensor and sending it to server");

    while ( end_of_program != -1 ) {

        printf("\n IOTClient@main > Getting data from the accelerometer.");
        get_accelerometer_data(accelerometer_data);

        printf("\n IOTClient@main > Getting data from the sensor color.");
        get_color_sensor_data(color_sensor_data);

    	// Data output to be displayed on the screen.

    	printf("\n Accelerometer: ");
    	printf("\n\t - X axis [g]: %.2f ", accelerometer_data[0]);
    	printf("\n\t - Y axis [g]: %.2f ", accelerometer_data[1]);
    	printf("\n\t - Z axis [g]: %.2f ", accelerometer_data[2]);
    	printf("\n");
    	printf("\n Color Sensor: ");
    	printf("\n\t - Brightness level: %.2f ", color_sensor_data[0]);
    	printf("\n\t - Red level       : %.2f ", color_sensor_data[1]);
    	printf("\n\t - Green level     : %.2f ", color_sensor_data[2]);
    	printf("\n\t - Blue level      : %.2f ", color_sensor_data[3]);
    	printf("\n\n");

//    	if (time_cnt < send_data_to_server_period) ++time_cnt;

    	if (time_cnt < SEND_TIME){

    		++time_cnt;
    	}

    	else {

    		printf("\n IOTClient@main > Sending data to server.");

    		time_cnt = 1;

    		data_to_send[0] = accelerometer_data[0];
    		data_to_send[1] = accelerometer_data[1];
    		data_to_send[2] = accelerometer_data[2];

    		data_to_send[3] = color_sensor_data[0];
    		data_to_send[4] = color_sensor_data[1];
    		data_to_send[5] = color_sensor_data[2];
    		data_to_send[6] = color_sensor_data[3];



            op_code = sendto(sockfd, (const float *)data_to_send, sizeof(data_to_send), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));

//            op_code = sendto(sockfd, (const float *)data_to_send, sizeof(data_to_send), MSG_DONTWAIT, (const struct sockaddr *) &servaddr, sizeof(servaddr));

            if ( op_code == -1 ) {
            	perror("\n IOTClient@main #> Error while sending data.");
            	exit(EXIT_FAILURE);
            }

            n = recvfrom(sockfd, (char *)buffer, MAXLINE,  MSG_DONTWAIT, (struct sockaddr *) &servaddr, &len);


            if ( n == -1 ) {
            	perror("#> Error while receiving data.");
            	//exit(EXIT_FAILURE);
            }

            buffer[n] = '\0';
            printf("\nServer response : %s\n", buffer);
    	}

    	//sleep(measure_rate);
		sleep(SAMPLE_TIME);
    }

    printf("\n IOTClient@main > Freeing up i2c module allocated resources.");

    // Closing i2c module allocated resources.
    if ( deactivate_i2c_driver() < 0 ) {
        perror("\n IOTClient@main #> Closing i2c module allocated resources.");
        exit(EXIT_FAILURE);
    }

    printf("\n IOTClient@main > Freeing up UDP socket allocated resources.");

    // Closing socket file descriptor
    if ( close(sockfd) < 0 ) {
        perror("\n IOTClient@main #> Closing socket file descriptor.");
        exit(EXIT_FAILURE);
    }

    printf("\n IOTClient@main > Successful freeing up of allocated resources. See you soon");

    printf("\n\n");

    printf("\n ________________________________________");
    printf("\n/  Thanks for using this IOT application \"");
    printf("\n|                                        |");
    printf("\n\    If something doesn't work, reset    /");
	printf("\n ----------------------------------------");
    printf("\n   \"");
    printf("\n    \"");
    printf("\n        .--.");
    printf("\n       |o_o |");
	printf("\n       |:_/ |");
	printf("\n      //   \ \"");
    printf("\n     (|     | )");
    printf("\n    /'\_   _/`\"");
    printf("\n    \___)=(___/");

    sleep(5);

    exit(EXIT_SUCCESS);
}

// CTRL + C -> Sent SIGINT signal to the process.
void SIGINT_handler (int signal)
{
	end_of_program = -1;
	printf("\n IOTClient@main > SIGINT detected resources. Finishing the process and freeing up allocated resources.");
    if ( close(sockfd) < 0 ) {
        perror("\n IOTClient@main #> Closing socket file descriptor.");
        exit(EXIT_FAILURE);
    }
}

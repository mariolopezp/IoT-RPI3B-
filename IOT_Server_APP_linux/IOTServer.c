/**
  ******************************************************************************
  * @file   IOTServer.c
  * @author Mario López Parejo (mario.lopezp@alumnos.upm.es) & Juan Morales Sáez (j.msaez@alumnos.upm.es)
  * @brief  IOTServer Module.
  *
  * @note   Embedded System Design with Raspberry Pi (ESD-RPI)
  *         This module manages the IOT server using UDP socket.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <math.h> //Para calcular las estadísticas.

#include <sys/types.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <netinet/in.h>


/* Private define ------------------------------------------------------------*/

#define PORT    5784
#define MAXLINE 1024
#define TIEMPO_ESTADISTICAS 6


/* Private variables----------------------------------------------------------*/

int end_of_program;
int sockfd;

/* Function prototypes -------------------------------------------------------*/

void SIGINT_handler (int signal);
float calculo_media(int contador, float datos, float prev_media);
float calculo_min(float datos, float minimo);
float calculo_max(float datos, float maximo);
float calculo_std(float media, float *valor, int contador);

/* Functions definition ------------------------------------------------------*/

// Driver code
int main() {

    char *ack_msg = "ACK";
    int primera_tx = 0;
    /**
     * Las posiciones de estos arrays van a ser las siguientes:
     * [0] -> x_accel
     * [1] -> y_accel
     * [2] -> z_accel
     * [3] -> lum
     * [4] -> red
     * [5] -> green
     * [6] -> blue
     *
     */
    float recv[7];
    float medias[7];
    float valores_max[7];
    float valores_min[7];
    float stdeviation[7];

    float almacenados_x[TIEMPO_ESTADISTICAS];
    float almacenados_y[TIEMPO_ESTADISTICAS];
    float almacenados_z[TIEMPO_ESTADISTICAS];
    float almacenados_lux[TIEMPO_ESTADISTICAS];
    float almacenados_red[TIEMPO_ESTADISTICAS];
    float almacenados_green[TIEMPO_ESTADISTICAS];
    float almacenados_blue[TIEMPO_ESTADISTICAS];

    int number_message_received = 0;

    struct sockaddr_in servaddr, cliaddr;
    int len, n, i;

    signal(SIGINT, SIGINT_handler);
    end_of_program = 0;

    printf("\n IOTServer@main > Initializing UDP socket.");

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("\n IOTServer@main #> Error while socket creation.");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 )
    {
        perror("IOTServer@main #> Error: bind failed.");
        exit(EXIT_FAILURE);
    }

    printf("\n IOTServer@main > Initialization finished. Start receiving data.\n");

    while (end_of_program != -1) {

        len = sizeof(cliaddr);  //len is value/result

        n = recvfrom(sockfd, (float *)recv, sizeof(recv), MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);



        if(n!=0){  //The value of n is 0 when the peer has performed a succesful and orderly shutdown.

          printf("\n Aceleracion eje x [g]: %.2f",recv[0]);
          printf("\n Aceleracion eje y [g]: %.2f",recv[1]);
          printf("\n Aceleracion eje z [g]: %.2f",recv[2]);
          printf("\n Luminosity level  [%]: %.2f",recv[3]);
          printf("\n Red level         [%]: %.2f",recv[4]);
          printf("\n Green level       [%]: %.2f",recv[5]);
          printf("\n Blue level        [%]: %.2f",recv[6]);

          /**
           *
           * Inicialización de los valores de las estadísticas
           * Los tengo que inicializar con el primer valor que reciben.
           * Si lo inicializo a 0 el valor max y el minimo, no se puede calcular
           * bien. Hay que meter mucha lógica innecesaria
           *
           * */
          if (primera_tx==0){
			  for(i = 0;i<7;i++){
				medias[i] = 0;
				stdeviation[i] = 0;
				valores_max[i] = recv[i];
				valores_min[i] = recv[i];
			  }
			  primera_tx = 1;
          }

          almacenados_x[number_message_received] = recv[0];
          almacenados_y[number_message_received] = recv[1];
          almacenados_z[number_message_received] = recv[2];
          almacenados_lux[number_message_received] = recv[3];
          almacenados_red[number_message_received] = recv[4];
          almacenados_green[number_message_received] = recv[5];
          almacenados_blue[number_message_received] = recv[6];

          number_message_received++;

          //Esto lo acabo de mover aqui, a ver si asi va.
          sendto(sockfd, (const char *)ack_msg, strlen(ack_msg), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);

          //Hay que calcular las estadisticas.
          //sizeof te debuelve el tamaño en numero de bytes
          for(i = 0;i<7;i++){
        	  medias[i] = calculo_media(number_message_received, recv[i], medias[i]);
          	  valores_max[i] = calculo_max(recv[i], valores_max[i]);
          	  valores_min[i] = calculo_min(recv[i], valores_min[i]);
          	  if(number_message_received % TIEMPO_ESTADISTICAS == 0){
				  switch(i){

				  case 0:
					  printf("\n\n\n Aceleración máxima (x)[g]: %.2f",valores_max[i]);
					  printf("\n Aceleración minima (x)[g]: %.2f",valores_min[i]);
					  printf("\n Aceleración media  (x)[g]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_x, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					break;

				  case 1:
					  printf("\n Aceleración máxima (y)[g]: %.2f",valores_max[i]);
					  printf("\n Aceleración minima (y)[g]: %.2f",valores_min[i]);
					  printf("\n Aceleración media  (y)[g]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_y, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					break;

				  case 2:
					  printf("\n Aceleración máxima (z)[g]: %.2f",valores_max[i]);
					  printf("\n Aceleración minima (z)[g]: %.2f",valores_min[i]);
					  printf("\n Aceleración media  (z)[g]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_z, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					break;

				  case 3:
					  printf("\n Luminosidad máxima        [%]: %.2f",valores_max[i]);
					  printf("\n Luminosidad minima        [%]: %.2f",valores_min[i]);
					  printf("\n Luminosidad media         [%]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_lux, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					  break;

				  case 4:
					  printf("\n Intensidad rojo máxima    [%]: %.2f",valores_max[i]);
					  printf("\n Intensidad rojo minima    [%]: %.2f",valores_min[i]);
					  printf("\n Intensidad rojo media     [%]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_red, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					  break;

				  case 5:
					  printf("\n Intensidad verde máxima   [%]: %.2f",valores_max[i]);
					  printf("\n Intensidad verde minima   [%]: %.2f",valores_min[i]);
					  printf("\n Intensidad verde media    [%]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_green, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					  break;

				  case 6:
					  printf("\n Intensidad azul máxima    [%]: %.2f",valores_max[i]);
					  printf("\n Intensidad azul minima    [%]: %.2f",valores_min[i]);
					  printf("\n Intensidad azul media     [%]: %.2f",medias[i]);
		          	  stdeviation[i] = calculo_std(medias[i], &almacenados_blue, number_message_received);
					  printf("\n Desviacion típica        : %.2f",stdeviation[i]);
					  number_message_received = 0;
					  break;
				  default:
					  break;
				  }
          	  }
          }
       }
    }

    printf("\n IOTServer@main > Freeing up UDP socket allocated resources.");

    // Closing socket file descriptor
    if ( close(sockfd) < 0 ) {
        perror("\n IOTServer@main #> Closing socket file descriptor.");
        exit(EXIT_FAILURE);
    }

    printf("\n IOTServer@main > Successful freeing up of allocated resources. See you soon");

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

	printf("\n IOTServer@main > SIGINT detected resources. Finishing the process and freeing up allocated resources.");
    if ( close(sockfd) < 0 ) {
        perror("\n IOTServer@main #> Closing socket file descriptor.");
        exit(EXIT_FAILURE);
    }
    exit(EXIT_SUCCESS);
}

float calculo_media(int contador, float datos, float prev_media){
    //La mejor forma de calcular esto es con la anterior media
	float media = ((contador-1)*prev_media + datos)/(contador);
	return media;
}

float calculo_max(float datos, float maximo){

	if (datos > maximo){
		return datos;
	}
	else{
		return maximo;
	}

}

float calculo_min(float datos, float minimo){

	if(datos < minimo){
		return datos;
	}
	else{
		return minimo;
	}
}


float calculo_std(float media, float *valor, int contador){

	float std = 0;
	float numerador = 0;
	int i;
	for(i=0;i<contador;i++){ //sizeof(valor)
		numerador = numerador + pow((valor[i] - media),2);
	}
	std = sqrt(numerador/sizeof(valor));
	return std;
}



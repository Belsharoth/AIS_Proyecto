#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include "portable.h"
#include "nmea.h"
#include "sixbit.h"
#include "vdm_parse.h"

#include "naves.h"
#include "msg_list.h"

int thread_count;
pthread_mutex_t lock;
message_list* list;

void* boat_thread(void* num);
void* receptor(void* num);

int main()
{   
    
    long thread;
	pthread_t* thread_handler;
    list = NULL;
    printf("Cada segudo equivale a 1 minuto\n");
	thread_count = 3;
    thread_handler = malloc (thread_count * sizeof (pthread_t));
    
    // ATENCION!!
    boat* barcos[5];
    barcos[0] = (boat *) malloc (sizeof ( boat ));
    // (objeto barco, userid, nav_status, rot, sog, cog, pos_accu, lat, long )
    boat_init(barcos[0], 1, 0, 0, 10, 135, 0, 0.0, 0.0); 
    barcos[1] = (boat *) malloc (sizeof ( boat ));
    // (objeto barco, userid, nav_status, rot, sog, cog, pos_accu, lat, long )
    boat_init(barcos[1], 2, 0, 0, 10, 90, 0, 0.8, 0.05); 
    barcos[2] = (boat *) malloc (sizeof ( boat ));
    // (objeto barco, userid, nav_status, rot, sog, cog, pos_accu, lat, long )
    boat_init(barcos[2], 3, 0, 0, 10, 45, 0, -0.5, -0.5); 
    barcos[3] = (boat *) malloc (sizeof ( boat ));
    // (objeto barco, userid, nav_status, rot, sog, cog, pos_accu, lat, long )
    boat_init(barcos[3], 4, 0, 0, 10, 270, 0, 2, -0.5); 
    barcos[4] = (boat *) malloc (sizeof ( boat ));
    // (objeto barco, userid, nav_status, rot, sog, cog, pos_accu, lat, long )
    boat_init(barcos[4], 5, 0, 0, 10, 0, 0, 0, 0); 
    
    pthread_mutex_init (&lock, NULL);
    pthread_create(&thread_handler[0], NULL, receptor, (void*) barcos[0]);
	for(thread = 1 ; thread != thread_count + 1 ; thread++)
        pthread_create(&thread_handler[thread], NULL, boat_thread, (void*) barcos[thread]);
    
    for(thread = 0 ; thread != thread_count ; thread++)
        printf("Aqui\n");pthread_join(thread_handler[thread], NULL);
    return 0;
}


void* boat_thread(void* data)
{   
    boat* barco = (boat*) data;
    char* msg;
    msg = boat_message(barco); 
    pthread_mutex_lock(&lock);
    list = add_message(list, msg);
    pthread_mutex_unlock(&lock);
	while(1)
    {	
        boat_new_pos(barco, 3);
        msg = boat_message(barco); 
        pthread_mutex_lock(&lock);
        list = add_message(list, msg);
        pthread_mutex_unlock(&lock);
        sleep(3); // mensajes cada 3 minutos
    }
}

void* receptor(void* data) 
{   
    boat* barco = (boat*) data;
    char msg[256];
    ais_state ais;
    aismsg_1  msg_1; // Solo se contempla mensages de tipo 1.
    long lat,lon;
    double lat_dd, long_ddd;
    float time;
    while (1)
    {   
        time = 0;
        memset( &ais, 0, sizeof( ais_state ) );
        pthread_mutex_lock(&lock);
        if (list != NULL)
        {   
            list = remove_message(list, msg);
            pthread_mutex_unlock(&lock);
            if (assemble_vdm( &ais, msg ) == 0)
            {
                ais.msgid = (unsigned char) get_6bit( &ais.six_state, 6 );
                
                if( (ais.msgid == 1)&&(parse_ais_1( &ais, &msg_1 ) == 0) )
                {   
                    float dist = distancia(barco->cog, msg_1.cog, barco->sog, msg_1.sog, barco->latitude, barco->longitude, msg_1.latitude, msg_1.longitude);
                    lat = msg_1.latitude; lon = msg_1.longitude;
					conv_pos(&lat, &lon);
                    pos2ddd( lat, lon, &lat_dd, &long_ddd );
                    printf("*****-----*****-----*****-----*****\n");
	                printf( "MESSAGE ID: %d\t", ais.msgid );
			        printf( "USER ID   : %ld\n", msg_1.userid );
			        printf( "SOG       : %d\t", msg_1.sog );
			        printf( "COG       : %d\n", msg_1.cog );
                    printf( "POSITION  : Lat %0.6f Lon %0.6f\n", lat_dd, long_ddd );
                    printf("MDCPA :%0.6f \n", dist); 
                }else printf("ERROR!!\nmsgid:%d msg:%s \n", ais.msgid, msg);
            }
        }
        else 
        {
            pthread_mutex_unlock(&lock);
            printf("No hay mensages\n");
            sleep(1);
            time += 1;
        }
        usleep(100000); // 0.1 seg
        time += 0.1;
        boat_new_pos(barco, time);
    }
}


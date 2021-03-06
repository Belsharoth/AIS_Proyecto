
typedef struct{
    unsigned long userid;
    char nav_status;
    unsigned char rot;
    int sog;
    int cog;
    char pos_acc;
    long latitude;
    long longitude;
}boat;

int boat_init (boat *barco, unsigned long userid,char nav_status, unsigned char rot,
    int sog, int cog, char pos_acc, double lat, double lon);

int boat_new_pos(boat *barco, float time);

char* boat_message(boat *barco);

float distancia (int cog_a, int cog_b, float speed_a, float speed_b, long lat_a, long lon_a, long lat_b, long lon_b);


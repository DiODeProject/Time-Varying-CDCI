#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "agentCDCIlocal.h"
#include <debug.h>
#include <float.h>

#define PI 3.14159265

#define UNCOMMITTED 0
#define	BEACON 77
#define	AGENT 21

/* Enum for different motion types */
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


/* Flag for successful message sent */
bool message_sent = false;

/* Flag for decision to broadcast a message */
bool broadcast_msg = false;

/* current motion type */
motion_t current_motion_type = STOP;

/* current commitment */
uint8_t my_commitment;
uint8_t my_option_GPS_X;
uint8_t my_option_GPS_Y;
uint8_t my_option_quality;


/* counters for motion, turning, broadcasting and status-update */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t max_straight_ticks = 300;
const uint32_t broadcast_ticks = 32;
uint32_t last_motion_ticks = 0;
uint32_t last_broadcast_ticks = 0;
uint32_t update_ticks = 50; /* setting how often performing the commitment update. a tick here is every ~31ms */
uint32_t last_update_ticks = 0;


/* Variables for outgoing messages */
message_t message;

/* Variables for incoming messages from another robot */
uint8_t received_option_GPS_X;
uint8_t received_option_GPS_Y;
uint8_t received_quality;
bool received_message;

/* Variables for incoming messages from ARK */
uint8_t discovered_option_GPS_X;
uint8_t discovered_option_GPS_Y;
double discovered_option_mean_quality;
uint8_t discovered_option_quality;
bool discovered;

/* Variables for Smart Arena messages */
unsigned int sa_type = 3;
unsigned int sa_payload = 0;
bool new_sa_msg_discovery = false;

/* Noise variables */
double variance=0.0;

/* Robot GPS variables */
uint8_t Robot_GPS_X;
uint8_t Robot_GPS_Y;
double Robot_orientation;
bool new_sa_msg_gps=false;

/* Robot Goal variables*/
uint8_t Goal_GPS_X;
uint8_t Goal_GPS_Y;
bool GoingAway=false;


/* Options lookup table*/
uint8_t options_IDs[20];
uint8_t options_GPS_X[20];
uint8_t options_GPS_Y[20];
uint8_t number_of_options=0;

bool GoingToResampleOption=false;

/*-------------------------------------------------------------------*/
/* Function to generate a random nuber from a gaussian               */
/*-------------------------------------------------------------------*/
double generateGaussianNoise(double mu, double variance )
{
    const double epsilon = DBL_MIN;
    const double two_pi = 2.0*3.14159265358979323846;
    double sigma=sqrt(variance);
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    double z0;
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    return z0 * sigma + mu;
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(double* angle){
    if(*angle>180){
        *angle=*angle-360;
    }
    if(*angle<-180){
        *angle=*angle+360;
    }
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
double AngleToGoal() {
    double angletogoal=atan2(Goal_GPS_Y-Robot_GPS_Y,Goal_GPS_X-Robot_GPS_X)/PI*180-Robot_orientation;
    NormalizeAngle(&angletogoal);
    return angletogoal;
}


/*-------------------------------------------------------------------*/
/* Coordinates to option ID                                          */
/*-------------------------------------------------------------------*/
void CoordsToID()
{
    for(int i=0;i<number_of_options;i++){
        if( (my_option_GPS_X==options_GPS_X[i]) && (my_option_GPS_Y==options_GPS_Y[i]) )
        {
            my_commitment=options_IDs[i];
            break;
        }
        else{
            my_commitment=UNCOMMITTED;
        }
    }
}


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    if( current_motion_type != new_motion_type ){
        int calibrated = true;
        switch( new_motion_type ) {
        case FORWARD:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_straight_left,kilo_straight_right);
            else
                set_motors(67,67);
            break;
        case TURN_LEFT:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_turn_left,0);
            else
                set_motors(70,0);
            break;
        case TURN_RIGHT:
            spinup_motors();
            if (calibrated)
                set_motors(0,kilo_turn_right);
            else
                set_motors(0,70);
            break;
        case STOP:
        default:
            set_motors(0,0);
        }
        current_motion_type = new_motion_type;
    }
}


/*-------------------------------------------------------------------*/
/* Function for setting the the new commitment state                 */
/* (including LED colour and message initialisation)                 */
/*-------------------------------------------------------------------*/
void set_commitment( uint8_t new_option_GPS_X, uint8_t new_option_GPS_Y, uint8_t new_quality) {

    /* update the commitment state varieable */
    my_option_GPS_X = new_option_GPS_X;
    my_option_GPS_Y = new_option_GPS_Y;
    my_option_quality = new_quality;
    CoordsToID();

    /* update message to send*/
    message.data[0] = my_option_GPS_X;
    message.data[1] = my_option_GPS_Y;
    message.data[2] = my_option_quality;
    message.type    = AGENT;
    message.crc     = message_crc(&message);

}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise commitment and LED */
    set_commitment(0 , 0 , 0);

    /* Initialise motors */
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    set_motion( FORWARD );
    last_motion_ticks = rand_soft() % max_straight_ticks + 1;

    /* Initialise broadcast variables */
    last_broadcast_ticks = rand_soft() % broadcast_ticks + 1;

    /** Initialise update variables */
    last_update_ticks= rand() % update_ticks;

    /* Initialise received message variables */
    received_message = false;
    received_option_GPS_X=0;
    received_option_GPS_Y=0;
    received_quality= 0;

    discovered = false;
    discovered_option_GPS_X=0;
    discovered_option_GPS_Y=0;
    discovered_option_quality = 0;

    /* Intialize time to 0 */
    kilo_ticks=0;
}

/*--------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message) */
/*--------------------------------------------------------------------------*/
void update_commitment() {
    if(!GoingToResampleOption){
        if (my_commitment == UNCOMMITTED)
        {
            /* Discovery */
            if ( discovered ){
                set_commitment( discovered_option_GPS_X , discovered_option_GPS_Y , discovered_option_quality );

                /* Go away from the discovered option */
                Goal_GPS_X=rand()%32;
                uint32_t distance=0;
                do {
                    Goal_GPS_Y=rand()%32;
                    distance=sqrt((discovered_option_GPS_X-Goal_GPS_X)*(discovered_option_GPS_X-Goal_GPS_X)+(discovered_option_GPS_Y-Goal_GPS_Y)*(discovered_option_GPS_Y-Goal_GPS_Y));
                } while(distance<=8);

                GoingAway=true;
                set_color(RGB(3,0,0));
            }
            /* Recruitement */
            if (received_message && ( (received_option_GPS_X!=0) || (received_option_GPS_Y!=0) ) ) {
                set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                /* Go resample the option */
                Goal_GPS_X=my_option_GPS_X;
                Goal_GPS_Y=my_option_GPS_Y;
                GoingToResampleOption=true;
                set_color(RGB(3,0,0));
            }
        }
        else {
            /* Conversion */
            if (received_message && ( (received_option_GPS_X!=0) || (received_option_GPS_Y!=0) ) && ( (my_option_GPS_X!=received_option_GPS_X) || (my_option_GPS_Y!=received_option_GPS_Y) ) )
            {
                if( (received_quality > my_option_quality) || ( (received_quality==my_option_quality) && (rand()%2==0) ) )
                {
                    set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                    /* Go resample the option */
                    Goal_GPS_X=my_option_GPS_X;
                    Goal_GPS_Y=my_option_GPS_Y;
                    GoingToResampleOption=true;
                    set_color(RGB(3,0,0));
                }
            }
        }

        received_message = false;
        discovered = false;
    }
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
    if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0];
        int id2 = msg->data[3];
        int id3 = msg->data[6];

        if (id1 == kilo_uid) {
            // unpack type
            sa_type = msg->data[1] >> 7;
            if(sa_type==0)
            {
                // unpack payload
                Robot_GPS_X = msg->data[1]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[1] & 0x03)<< 3 | msg->data[2]>>5 ;
                Robot_orientation = (msg->data[2] & 0x1F)*12;
                NormalizeAngle(&Robot_orientation);
                new_sa_msg_gps = true;
            }
            else{
                // unpack payload
                discovered_option_GPS_X = msg->data[1]>>2 & 0x1F;
                discovered_option_GPS_Y = (msg->data[1] & 0x03)<< 3 | msg->data[2]>>5 ;
                discovered_option_mean_quality = (msg->data[2] & 0x1F);

                if(discovered_option_mean_quality>10.0)
                {
                    discovered_option_mean_quality = 9+0.1*(discovered_option_mean_quality-10.0);
                }
                new_sa_msg_discovery = true;
            }
        }

        if (id2 == kilo_uid) {
            sa_type = msg->data[4] >> 7;
            if(sa_type==0){
                // unpack payload
                Robot_GPS_X = msg->data[4]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[4] & 0x03)<< 3 | msg->data[5]>>5 ;
                Robot_orientation = (msg->data[5] & 0x1F)*12;
                NormalizeAngle(&Robot_orientation);
                new_sa_msg_gps = true;
            }
            else{
                // unpack payload
                discovered_option_GPS_X = msg->data[4]>>2 & 0x1F;
                discovered_option_GPS_Y = (msg->data[4] & 0x03)<< 3 | msg->data[5]>>5 ;
                discovered_option_mean_quality = (msg->data[5] & 0x1F);

                if(discovered_option_mean_quality>10.0)
                {
                    discovered_option_mean_quality = 9+0.1*(discovered_option_mean_quality-10.0);
                }

                new_sa_msg_discovery = true;
            }
        }
        if (id3 == kilo_uid)
        {
            sa_type = msg->data[7] >> 7;
            if(sa_type==0){
                // unpack payload
                Robot_GPS_X = msg->data[7]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[7] & 0x03)<< 3 | msg->data[8]>>5 ;
                Robot_orientation = (msg->data[8] & 0x1F)*12;
                NormalizeAngle(&Robot_orientation);
                new_sa_msg_gps = true;
            }
            else{
                // unpack payload
                discovered_option_GPS_X = msg->data[7]>>2 & 0x1F;
                discovered_option_GPS_Y = (msg->data[7] & 0x03)<< 3 | msg->data[8]>>5 ;
                discovered_option_mean_quality = (msg->data[8] & 0x1F);

                if(discovered_option_mean_quality>10.0)
                {
                    discovered_option_mean_quality = 9+0.1*(discovered_option_mean_quality-10.0);
                }
                new_sa_msg_discovery = true;
            }
        }

    }
    else if (msg->type == 1) { // Options lookup table
        options_IDs[number_of_options] = msg->data[0];
        options_GPS_X[number_of_options] = msg->data[1];
        options_GPS_Y[number_of_options] = msg->data[2];
        variance=msg->data[3]/10.0;
        number_of_options++;
    }
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    else if (msg->type == AGENT && !GoingToResampleOption ) { // the received message is from another KB
        received_option_GPS_X = msg->data[0];
        received_option_GPS_Y = msg->data[1];
        received_quality = msg->data[2];
        received_message = true;
    }
    if (new_sa_msg_discovery == true)
    {
        new_sa_msg_discovery = false;
        discovered_option_quality = (uint8_t) ( 10 * (  generateGaussianNoise( discovered_option_mean_quality,variance) ) );
        if(discovered_option_quality>100){
            discovered_option_quality=100;
        }
        if(discovered_option_quality<0){
            discovered_option_quality=0;
        }

        if( GoingToResampleOption && ( discovered_option_GPS_X == my_option_GPS_X ) && ( discovered_option_GPS_Y == my_option_GPS_Y ) )
        {
            set_commitment(my_option_GPS_X,my_option_GPS_Y,discovered_option_quality);
            GoingToResampleOption=false;
            GoingAway=true;

            Goal_GPS_X=rand()%32;
            uint32_t distance=0;
            do {
                Goal_GPS_Y=rand()%32;
                distance=sqrt((discovered_option_GPS_X-Goal_GPS_X)*(discovered_option_GPS_X-Goal_GPS_X)+(discovered_option_GPS_Y-Goal_GPS_Y)*(discovered_option_GPS_Y-Goal_GPS_Y));
            } while(distance<=8);

            return;
        }

        discovered = true;
    }

    update_commitment();
}



/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
    switch( current_motion_type ) {
    case TURN_LEFT:
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
            /* perform a radnom turn */
            last_motion_ticks = kilo_ticks;
            if( rand_soft()%2 ) {
                set_motion(TURN_LEFT);
            }
            else {
                set_motion(TURN_RIGHT);
            }
            turning_ticks = rand_soft()%max_turning_ticks + 1;
        }
        break;
    case STOP:
    default:
        set_motion(STOP);
    }
}


/*-------------------------------------------------------------------*/
/* Function to go resample an option                                 */
/*-------------------------------------------------------------------*/
void GoToOption(){
    if(new_sa_msg_gps){
        new_sa_msg_gps=false;

        if( (Robot_GPS_X==Goal_GPS_X) &&  (Robot_GPS_Y==Goal_GPS_Y) ){
            if( GoingAway){
                GoingAway=false;
                set_color(RGB(0,0,0));
            }
        }
        else{
            if(fabs(AngleToGoal()) <= 20){
                set_motion(FORWARD);
                last_motion_ticks = kilo_ticks;
            }
            else{
                if(AngleToGoal()>0){
                    set_motion(TURN_LEFT);
                    last_motion_ticks = kilo_ticks;
                    turning_ticks=(unsigned int) ( fabs(AngleToGoal())/45.0*30.0 );
                }
                else{
                    set_motion(TURN_RIGHT);
                    last_motion_ticks = kilo_ticks;
                    turning_ticks=(unsigned int) ( fabs(AngleToGoal())/45.0*30.0 );
                }
            }
        }
    }

    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
        }
        break;
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        break;

    case STOP:
    default:
        set_motion(STOP);
    }
}

/*-------------------------------------------------------------------*/
/* Function to broadcast the commitment message                     */
/*-------------------------------------------------------------------*/
void broadcast() {
    if( kilo_ticks > last_broadcast_ticks + broadcast_ticks ) {
        last_broadcast_ticks = kilo_ticks;

        /* set broadcast flag for transmission */
        broadcast_msg = true;
    }
}


/*-------------------------------------------------------------------*/
/* Callback function for message transmission                        */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
    if( broadcast_msg ) {
        return &message;
    }
    return 0;
}


/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void tx_message_success() {
    broadcast_msg = false;
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {

    if(GoingToResampleOption || GoingAway){
        GoToOption();
    }
    else{
        random_walk();
    }
    
    if(!GoingToResampleOption){
        broadcast();
    }

    /* Set LED color*/
    if(GoingToResampleOption || GoingAway){
        set_color(RGB(3,0,0));
    }
    else{

        switch( my_commitment ) {
        case 6:
            set_color(RGB(0,0,3));
            break;
        case 5:
            set_color(RGB(0,3,0));
            break;
        case 4:
            set_color(RGB(3,3,3));
            break;
        case 3:
            set_color(RGB(0,3,3));
            break;
        case 2:
            set_color(RGB(3,0,3));
            break;
        case 1:
            set_color(RGB(3,3,0));
            break;
        case 0:
        default:
            set_color(RGB(0,0,0));
            break;
        }
    }

    debug_info_set(commitement, my_commitment);
}


/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = tx_message_success;
    kilo_message_rx=message_rx;
    debug_info_create();
    kilo_start(setup, loop);
    return 0;
}

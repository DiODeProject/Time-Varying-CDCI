#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "agentCDCIlocal.h"
#include <debug.h>

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

/* option quality in range [0,100] (unit digit used as a decimal, i.e., v=v*0.1) */
uint8_t option_quality;

/* counters for motion, turning, broadcasting and status-update */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t max_straight_ticks = 300;
const uint32_t broadcast_ticks = 32;
uint32_t last_motion_ticks = 0;
uint32_t last_broadcast_ticks = 0;

/* Variables for outgoing messages */
message_t message;

/* Variables for incoming messages */
uint8_t received_option;
uint8_t received_quality;
bool received_message;
uint8_t discovered_option;
uint8_t discovered_quality;
bool discovered;

/* Variables for Smart Arena messages */
unsigned int sa_type = 3;
unsigned int sa_payload = 0;
bool new_sa_msg = false;
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
void set_commitment( uint8_t new_commitment_state, uint8_t new_quality ) {
    /* update the commitment state varieable */
    my_commitment = new_commitment_state;
    option_quality = new_quality;
    /* Initialise the message variable */
    message.data[0] = my_commitment;
    message.data[1] = option_quality;
    message.type    = AGENT;
    message.crc     = message_crc(&message);
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise commitment and LED */
    set_commitment(UNCOMMITTED, 0);

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

    /* Initialise received message variables */
    received_message = false;
    received_option = UNCOMMITTED;
    received_quality = 0;
    discovered = false;
    discovered_option = UNCOMMITTED;
    discovered_quality = 0;
    kilo_ticks=0;
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {

    if (msg->type == 0) {
        // the received message is from ARK
        // unpack message
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
        if (id1 == kilo_uid) {
            // unpack type
            sa_type = ( msg->data[1] >> 2 )& 0x0F; // options ID
            // unpack payload
            sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]); // options quality
            new_sa_msg = true;
        }
        if (id2 == kilo_uid) {
            // unpack type
            sa_type = msg->data[4] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
            new_sa_msg = true;
        }
        if (id3 == kilo_uid) {
            // unpack type
            sa_type = msg->data[7] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
            new_sa_msg = true;
        }
    }
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    else if (msg->type == AGENT) { // the received message is from another KB
        received_option = msg->data[0];
        received_quality = msg->data[1];
        received_message = true;
    }

    if (new_sa_msg == true) {
        discovered_option = sa_type;
        discovered_quality = sa_payload;
        discovered = true;
        new_sa_msg = false;
    }

    update_commitment();
}

/*--------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message) */
/*--------------------------------------------------------------------------*/
void update_commitment() {
    if (my_commitment == UNCOMMITTED){

        /* compute the transition probabilities as a fucntion of the estimated qualities */
        /* discovery is only possible if the message is received from a BEACON robot */
        if (discovered){
            set_commitment(discovered_option, discovered_quality);
        }
        /* recruitment is only possible if the message is received from (i) an AGENT robot (ii) committed to an option */
        if (received_message && received_option != UNCOMMITTED){
            set_commitment(received_option, received_quality);
        }
    }
    /* if the agent is committed */
    else {
        /* the other agent must be: (i) committed and (ii) with option different than mine */
        if (received_message && (received_option != UNCOMMITTED && my_commitment != received_option)){
            if(received_quality>option_quality){
                set_commitment(received_option, received_quality);
            }
            else {
                if(received_quality==option_quality){
                    int randomInt = rand();
                    if(randomInt%2==0){
                        set_commitment(received_option, received_quality);
                    }
                }
            }
        }
    }

    received_message = false;
    discovered = false;
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
/* Function to broadcast the commitment message                     */
/*-------------------------------------------------------------------*/
void broadcast() {

    // Broadcast every broadcast_ticks seconds
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
void tx_message_success()
{
    broadcast_msg = false;
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
    // Move randomly
    random_walk();

    // Broadcast committement state
    broadcast();

    // Share commitement state with ARK LoopFunction (for degguging purpose)
    debug_info_set(commitement, my_commitment);

    // Show commitement state via LED color (for debugging purpose)

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

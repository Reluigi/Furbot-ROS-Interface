#include "protocol.h"

// parse for each type of the status
int ParseCommStatus(char * comm_msg){
    // no info about structure
    return 0;
}

int ParseBmsStatus(char * bms_msg, void * bms_struct){
    // TODO: 1. Check length

    // TODO: 2. Parse and save to struct
}

// parse for each system ID
int ParseStatusFrame(char * frame, size_t frame_size){
    // check magic word

    // grab timer value

    // grab system count value

    // go through frame parsing status for each system by it's ID and checking
}
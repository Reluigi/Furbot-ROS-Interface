#include "furbot_protocol.h"

int ParseCommStatus(char * comm_msg){
    // no info about structure
    return 0;
}

// TODO: define function ParseBmsStatus
int ParseBmsStatus(char * bms_msg, void * bms_struct){}

// TODO: define function ParseMcStatus
int ParseMcStatus(char * mc_msg, void * mc_struct){}

int ParseTrackStatus(char * td_msg, void * td_struct){
    //
}

int ParseSteerStatus(char * sd_msg, void * sd_struct){
    //
}

// TODO: define other functions for parsing status parts.

int ParseStatusFrame(char * frame, size_t frame_size, status_struct * status){
    // check magic word
    for (int i = 0; i < 4; i++){
        if (frame[i] != STATUS_MAGIC_WORD[i]){
            return 1;
        }
    }
    // grab timer value and check

    // grab system count value and check

    // go through frame parsing status for each system by it's ID and checking
    int position = 12;
    while ( position < frame_size){
        switch (frame[position]) {
            //case SystemID.COMM:
                // TODO: process COMM part
                // continue;
            //case SystemID.BMS:
                // TODO: process BMS part
                // continue;
            case SystemID.TRACTION:
                //check length
                if (frame[position+1] == SystemMsgsLen.TRACTION){
                    // call parser
                }
                // increase position
                position = position + (int)frame[position+1] + 1;
                continue;
            case SystemID.STEERING:
                //check length
                if (frame[position+1] == SystemMsgsLen.STEERING){
                    // call parser
                }
                // increase position
                position = position + (int)frame[position+1] + 1;
                continue;
            //case SystemID.?? :
                // TODO: process other parts
                // continue;
            default:
                // use length to increase position
                position = position + (int)frame[position+1] + 1;
                continue;
        }
    }
}
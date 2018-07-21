#include "furbot_protocol.h"

int ParseCommStatus(char * comm_msg){
    // no info about structure
    return 0;
}

// TODO: define function ParseBmsStatus
int ParseBmsStatus(char * bms_msg, void * bms_struct){}

// TODO: define function ParseMcStatus
int ParseMcStatus(char * mc_msg, void * mc_struct){}

int ParseTrackStatus(char * td_msg, TractionStruct * td_struct){
    int position = 0;
    while ( position < SystemMsgsLen.TRACTION){
        // Parse td_msg bytes according to protocol bytes order
        switch (position){
            case 0 :
                td_struct->state = td_msg[position];
                position += 1;
                continue;
            case 1:
                td_struct->mode = td_msg[position];
                position += 1;
                continue;
            case 2:
                std::memcpy(td_struct->speed, td_msg[position], 2);
                position += 2;
                continue;
            case 4:
                std::memcpy(td_struct->vel_l, td_msg[position], 2);
                position += 2;
                continue;
            case 6:
                std::memcpy(td_struct->vel_r, td_msg[position], 2);
                position += 2;
                continue;
            case 8:
                std::memcpy(td_struct->throttle, td_msg[position], 2);
                position += 2;
                continue;
            case 10:
                std::memcpy(td_struct->brake, td_msg[position], 2);
                position += 2;
                continue;
            case 12:
                td_struct->reverse_flag = td_msg[position];
                position += 1;
                continue;
            case 13:
                std::memcpy(td_struct->odo_travel, td_msg[position], 4);
                position += 4;
                continue;
            default:
                return 1;
        }
    }
    return 0;
}

int ParseSteerStatus(char * sd_msg, void * sd_struct){
    //
}

// TODO: define other functions for parsing status parts.

int ParseStatusFrame(char * frame, int frame_size, StatusStruct * status){
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
                    if (ParseTractStatus(frame[position+2], status->traction_part)){
                        // Error - smth wrong
                    }
                }
                // increase position by length of the field and by 2 for SID and LEN
                position = position + (int)frame[position+1] + 2;
                continue;
            case SystemID.STEERING:
                //check length
                if (frame[position+1] == SystemMsgsLen.STEERING){
                    // call parser
                    if (ParseSteerStatus(frame[position+2], status->steering_part)){
                        // Error - smth wrong
                    }
                }
                // increase position
                position = position + (int)frame[position+1] + 2;
                continue;
            //case SystemID.?? :
                // TODO: process other parts
                // continue;
            default:
                // use length to increase position
                position = position + (int)frame[position+1] + 2;
                continue;
        }
    }
}
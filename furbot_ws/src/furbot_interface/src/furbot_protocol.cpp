#include "furbot_interface/furbot_protocol.h"

//// TODO: define function ParseBmsStatus
//int ParseBmsStatus(char * bms_msg, void * bms_struct){}
//
//// TODO: define function ParseMcStatus
//int ParseMcStatus(char * mc_msg, void * mc_struct){}


/**
 * First 4 bytes of status frame.
 */
char STATUS_MAGIC_WORD[4] = {0x41, 0x54, 0x53, 0x46};

/**
 * First 4 bytes of remote frame.
 */
char REMOTE_MAGIC_WORD[4] = {0x46, 0x53, 0x54, 0x41};


int ParseTractStatus(char * td_msg, TractionStruct * td_struct){
    int position = 0;
    while ( position < static_cast<int>(SystemMsgsLen::TRACTION)){
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
                std::memcpy((void *)td_struct->speed, (const void *)td_msg[position], 2);
                position += 2;
                continue;
            case 4:
                std::memcpy((void *)td_struct->vel_l, (const void *)td_msg[position], 2);
                position += 2;
                continue;
            case 6:
                std::memcpy((void *)td_struct->vel_r, (const void *)td_msg[position], 2);
                position += 2;
                continue;
            case 8:
                std::memcpy((void *)td_struct->throttle, (const void *)td_msg[position], 2);
                position += 2;
                continue;
            case 10:
                std::memcpy((void *)td_struct->brake, (const void *)td_msg[position], 2);
                position += 2;
                continue;
            case 12:
                td_struct->reverse_flag = td_msg[position];
                position += 1;
                continue;
            case 13:
                std::memcpy((void *)td_struct->odo_travel, (const void *)td_msg[position], 4);
                position += 4;
                continue;
            default:
                return 1;
        }
    }
    return 0;
}

int ParseSteerStatus(char * sd_msg, SteeringStruct * sd_struct){
    return 0;
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
            //case SystemID::COMM:
                // TODO: process COMM part
                // continue;
            //case SystemID::BMS:
                // TODO: process BMS part
                // continue;
            case static_cast<int>(SystemID::TRACTION):
                // check length
                if (frame[position+1] == static_cast<char>(SystemMsgsLen::TRACTION)){
                    // call parser
                    if (ParseTractStatus(&frame[position+2], status->traction_part)){
                        // Error - smth wrong
                    }
                }
                // increase position by length of the field and by 2 for SID and LEN
                position = position + (int)frame[position+1] + 2;
                continue;
            case static_cast<int>(SystemID::STEERING):
                //check length
                if (frame[position+1] == static_cast<char>(SystemMsgsLen::STEERING)){
                    // call parser
                    if (ParseSteerStatus(&frame[position+2], status->steering_part)){
                        // Error - smth wrong
                    }
                }
                // increase position
                position = position + (int)frame[position+1] + 2;
                continue;
            //case SystemID::?? :
                // TODO: process other parts
                // continue;
            default:
                // use length to increase position
                position = position + (int)frame[position+1] + 2;
                continue;
        }
    }
}
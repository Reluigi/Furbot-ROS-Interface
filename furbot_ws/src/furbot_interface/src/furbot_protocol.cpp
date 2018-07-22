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
        //std::cout << "ParseTractStatus while loop ... \n";
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
                int16_t speed;
                std::memcpy((void *) &speed, (const void *) &td_msg[position], 2);
                td_struct->speed = ntohs(speed);
                position += 2;
                continue;
            case 4:
                int16_t vel_l;
                std::memcpy((void *) &vel_l, (const void *) &td_msg[position], 2);
                td_struct->vel_l = ntohs(vel_l);
                position += 2;
                continue;
            case 6:
                int16_t vel_r;
                std::memcpy((void *) &vel_r, (const void *) &td_msg[position], 2);
                td_struct->vel_r = ntohs(vel_r);
                position += 2;
                continue;
            case 8:
                int16_t throttle;
                std::memcpy((void *) &throttle, (const void *) &td_msg[position], 2);
                td_struct->throttle = ntohs(throttle);
                position += 2;
                continue;
            case 10:
                int16_t brake;
                std::memcpy((void *) &brake, (const void *) &td_msg[position], 2);
                td_struct->brake = ntohs(brake);
                position += 2;
                continue;
            case 12:
                td_struct->reverse_flag = td_msg[position];
                position += 1;
                continue;
            case 13:
                int32_t odo_travel;
                std::memcpy((void *) &odo_travel, (const void *) &td_msg[position], 4);
                td_struct->odo_travel = ntohl(odo_travel);
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

    // grab systems count value and check

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
            case static_cast<char>(SystemID::TRACTION):
                // check length
                if (frame[position+1] == static_cast<int>(SystemMsgsLen::TRACTION)){
                    // call parser
                    if (ParseTractStatus(&frame[position+2], status->traction_part)){
                        // Error - smth wrong
                    }
                }
                // increase position by length of the field and by 2 for SID and LEN
                position = position + (int)frame[position+1] + 2;
                continue;
            case static_cast<char>(SystemID::STEERING):
                //check length
                if (frame[position+1] == static_cast<int>(SystemMsgsLen::STEERING)){
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
                //std::cout << "ParseStatusFrame while loop: skip \n";
                // use length to increase position
                position = position + (int)frame[position+1] + 2;
                continue;
        }
    }
    return 0;
}
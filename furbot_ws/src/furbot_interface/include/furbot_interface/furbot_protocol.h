//
// Created by rr on 16/07/18.
//

#ifndef FURBOT_PROTOCOL_H
#define FURBOT_PROTOCOL_H

// Max possible bytes in status frame
#define STATUS_FRAME_BUFFER_SIZE 256

// First 4 bytes of status frame
char STATUS_MAGIC_WORD[4] = {0x41, 0x54, 0x53, 0x46};

// First 4 bytes of remote frame
char REMOTE_MAGIC_WORD[4] = {0x46, 0x53, 0x54, 0x41};

// ID of the system in UDP Status Frame
enum class SystemID : unsigned
{
    COMM = 0,
    BMS = 1,
    MOTION_CONTROL = 2,
    TRACTION = 3,
    SREERING = 4,
    HYDRAULICS = 5,
    SUSP_RL = 6,
    SUSP_RR = 7,
    SUSP_FL = 8,
    SUSP_FR = 9,
    REAR_FORK = 10,
    FRONT_FORK = 11
};

enum class SystemMsgsLen : unsigned
{
    COMM = 1, // no description
    BMS = 8,
    MOTION_CONTROL = 7,
    TRACTION = 17,
    SREERING = 5,
    HYDRAULICS = 8,
    SUSPENSION = 7,
    FORK = 11
};

// System state
enum class SystemState : unsigned
{
    COLD = 0,
    INIT = 1,
    RUNNING = 2,
    HALTED = 3
};

// Hydraulics drive state
enum class HydrState : unsigned
{
    NO_OP = 0,
    ENABLE_MOT = 1,
    ENABLE_BRIDGE = 2,
    WRSWON = 3,
    RSWON = 4,
    WENABLE = 5,
    ENABLED = 6,
    DISABLE_MOT = 7,
    DISABLED = 8,
    FAULT = 9
};

// Suspension axis state
enum class SuspState : unsigned
{
    INIT = 0,
    STOPPED = 1,
    RISING = 2,
    LOWERING = 3
};

// Fork axis state
enum class ForkState : unsigned
{
    UNKNOWN = 0,
    STOPPED = 1,
    EXTENDING = 2,
    RETRACTING = 3
};

// Fork sensor ???
enum class ForkSensor : unsigned
{
    FWD = 0,
    CENT = 1,
    BACK = 2,
    IN1 = 3,
    MID1 = 4,
    OUT1 = 5,
    IN2 = 6,
    OUT2 = 7,
    TOP = 8,
    BOTTOM = 9,
    PRESF = 10,
    PRESB = 11,
    TIPF = 12,
    TIPB = 13
};

// TODO: define structures for different system's statuses

// TODO: define structure to combine all system statuses

int ParseCommStatus(char * comm_msg){}

/**
 * Battery Management System (BMS) status parser.
 *
 * @param [in] bms_msg Pointer to BMS part of status frame.
 * @param [in] bms_struct Pointer to structure, which should be filled with status values.
*/
int ParseBmsStatus(char * bms_msg, void * bms_struct){}

/**
 * Motion Control (MC) status parser.
 * @param [in] mc_msg Pointer to MC part of status frame.
 * @param [in] mc_struct Pointer to structure, which should be filled with status values.
 */
int ParseMcStatus(char * mc_msg, void * mc_struct){}

// parse for each system ID
int ParseStatusFrame(char * frame, size_t frame_size){}

#endif //FURBOT_PROTOCOL_H

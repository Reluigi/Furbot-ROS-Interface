/**
 * Furbot protocol parsing tools.
 */

#ifndef FURBOT_PROTOCOL_H
#define FURBOT_PROTOCOL_H

#include <cstring> // std::memcpy(dest, src, len);

/**
 * Max possible bytes in status frame
 */
#define STATUS_FRAME_BUFFER_SIZE 256

/**
 * First 4 bytes of status frame.
 */
char STATUS_MAGIC_WORD[4] = {0x41, 0x54, 0x53, 0x46};

/**
 * First 4 bytes of remote frame.
 */
char REMOTE_MAGIC_WORD[4] = {0x46, 0x53, 0x54, 0x41};

/**
 * IDs of the system in UDP Status Frame.
 */
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

/**
 * Lengths of different systems' messages.
 */
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

/**
 * System states.
 */
enum class SystemState : unsigned
{
    COLD = 0,
    INIT = 1,
    RUNNING = 2,
    HALTED = 3
};

/**
 * Hydraulics drive states.
 */
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

/*
 * Suspension States.
 */
enum class SuspState : unsigned
{
    INIT = 0,
    STOPPED = 1,
    RISING = 2,
    LOWERING = 3
};

/**
 * Fork States.
 */
enum class ForkState : unsigned
{
    UNKNOWN = 0,
    STOPPED = 1,
    EXTENDING = 2,
    RETRACTING = 3
};

/**
 * States of Fork Sensor.
 */
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

/**
 * @struct TractionStruct
 * @brief Structure to keep Traction Data. See protocol description.
 */
struct TractionStruct {
    char state;
    char mode;
    int speed;
    int vel_l;
    int vel_r;
    int throttle;
    int brake;
    char reverse_flag;
    int odo_travel;
};

/**
 * @struct SteeringStruct
 * @brief Structure to keep Steering Data. See protocol description.
 */
struct SteeringStruct {
    char state;
    int current_angle;
    int target_angle;
};

/**
 * @struct StatusStruct
 * @brief Structure to combine all systems' data.
 */
struct StatusStruct {
    TractionStruct * traction_part;
    SteeringStruct * steering_part;
};

/**
 * Battery Management System (BMS) status parser.
 *
 * @param [in] bms_msg Pointer to BMS part of status frame.
 * @param [in] bms_struct Pointer to structure, which should be filled with status values.
 * @return [out] int value, 0 if there were no errors, 1 in other case.
 */
int ParseBmsStatus(char * bms_msg, void * bms_struct){}

/**
 * Motion Control (MC) data status parser.
 *
 * @param mc_msg Pointer to MC part of status frame.
 * @param mc_struct Pointer to structure, which should be filled with status values.
 * @return int value, 0 if there were no errors, 1 in other case.
 */
int ParseMcStatus(char * mc_msg, void * mc_struct){}

/**
 * Traction Data (TD) status parser.
 *
 * @param td_msg Pointer to TD part of status frame.
 * @param td_struct Pointer to structure, which should be filled with status values.
 * @return int value, 0 if there were no errors, 1 in other case.
 */
int ParseTractStatus(char * td_msg, void * td_struct){}

/**
 * Steering Data (SD) status parser.
 *
 * @param sd_msg Pointer to SD part of status frame.
 * @param sd_struct Pointer to structure, which should be filled with status values.
 * @return int value, 0 if there were no errors, 1 in other case.
 */
int ParseSteerStatus(char * sd_msg, void * sd_struct){}

// TODO: declare function for other systems statuses parsing.

/**
 * Parse income Status Frame. Check if it's valid using magic word, time stamp, and system count.
 *
 * @param frame Pointer to Status Frame char array.
 * @param frame_size Size of the Status Frame.
 * @return int value, 0 if there were no errors, 1 in other case.
 */
int ParseStatusFrame(char * frame, int frame_size, StatusStruct * status){}

#endif //FURBOT_PROTOCOL_H

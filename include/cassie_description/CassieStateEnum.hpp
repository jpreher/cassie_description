/**
    @author J. Reher (jreher@caltech.edu)
*/


#ifndef CASSIE_STATE_ENUM_HPP
#define CASSIE_STATE_ENUM_HPP

typedef enum {
    BasePosX = 0,
    BasePosY = 1,
    BasePosZ = 2,
    BaseRotX = 3,
    BaseRotY = 4,
    BaseRotZ = 5,
    LeftHipRoll = 6,
    LeftHipYaw = 7,
    LeftHipPitch = 8,
    LeftKneePitch = 9,
    LeftShinPitch = 10,
    LeftTarsusPitch = 11,
    LeftHeelSpring = 12,
    LeftFootPitch = 13,
    RightHipRoll = 14,
    RightHipYaw = 15,
    RightHipPitch = 16,
    RightKneePitch = 17,
    RightShinPitch = 18,
    RightTarsusPitch = 19,
    RightHeelSpring = 20,
    RightFootPitch = 21
}CassieStateEnum;

#endif // CASSIE_STATE_ENUM_HPP

classdef CassieStateEnum < int8 %#codegen
    %% Class: CassieStateEnum
    %
    % Description: This class serves as an enumerator for the joint angles.
    %
    % Author: Jenna Reher, jreher@caltech.edu
    % _____________________________________________________________________
    
    enumeration
        BasePosX         (1),
        BasePosY         (2),
        BasePosZ         (3),
        BaseRotX         (4),
        BaseRotY         (5),
        BaseRotZ         (6),
        LeftHipRoll      (7),
        LeftHipYaw       (8),
        LeftHipPitch     (9),
        LeftKneePitch    (10),
        LeftShinPitch    (11),
        LeftTarsusPitch  (12),
        LeftHeelSpring   (13),
        LeftToePitch     (14),
        RightHipRoll     (15),
        RightHipYaw      (16),
        RightHipPitch    (17),
        RightKneePitch   (18),
        RightShinPitch   (19),
        RightTarsusPitch (20),
        RightHeelSpring  (21),
        RightToePitch    (22)
    end
end
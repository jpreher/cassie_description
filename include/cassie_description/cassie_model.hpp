/*
 * @brief Primary interface and class for evaluating the Cassie model dynamics and kinematics.
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef CASSIE_MODEL_HPP
#define CASSIE_MODEL_HPP

#include <cassie_common_toolbox/CassieStateEnum.hpp>
#include <cassie_common_toolbox/geometry.hpp>
#include <rbdl/rbdl.h>
#include <frost_expr/all.hpp>

namespace cassie_model {

class BodyPoint
{
public:
    std::string  body_name;
    unsigned int body_id = 999;

    std::string  point_name;
    RigidBodyDynamics::Math::Vector3d     point_position;

    BodyPoint() {}

    void initialize(RigidBodyDynamics::Model *model, std::string point_name, std::string body_name, Eigen::Vector3d &point_position);
};

class Kinematics
{
public:
    struct Cache {
        Eigen::MatrixXd J_poseLeft;
        Eigen::MatrixXd J_poseRight;
        Eigen::MatrixXd J_positionLeft;
        Eigen::MatrixXd J_positionRight;
        Eigen::MatrixXd J_achilles;
        Eigen::MatrixXd J_leftLegLength;
        Eigen::MatrixXd J_rightLegLength;

        Eigen::MatrixXd J_poseLeftConstraint;
        Eigen::MatrixXd J_poseRightConstraint;
        Eigen::MatrixXd Jdot_poseLeftConstraint;
        Eigen::MatrixXd Jdot_poseRightConstraint;

        Eigen::MatrixXd J_rigid;
        Eigen::MatrixXd Jdot_rigid;

        Eigen::MatrixXd Jdot_achilles;
        Eigen::MatrixXd Jdot_poseLeft;
        Eigen::MatrixXd Jdot_poseRight;

        void init();
        void reset();
    } cache;


    BodyPoint LeftMidFoot_Point;
    BodyPoint LeftToe_Point;
    BodyPoint LeftHeel_Point;
    BodyPoint RightMidFoot_Point;
    BodyPoint RightToe_Point;
    BodyPoint RightHeel_Point;

    Kinematics();

    void initialize(RigidBodyDynamics::Model *model);
    void update(RigidBodyDynamics::Model *model, const Eigen::VectorXd &q, const Eigen::VectorXd &dq);

    void computeConstrainedToeJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &Jl, Eigen::MatrixXd &Jr);
    void computeConstrainedFootJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &Jl, Eigen::MatrixXd &Jr);
    void computeStanceConstrainedJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &Jl, Eigen::MatrixXd &Jr);
};

class Dynamics
{
public:
    Dynamics() {} // Empty constructor

    MatrixXd H;
    VectorXd C;

    void initialize(RigidBodyDynamics::Model *model);
    void calcHandC(RigidBodyDynamics::Model *model, VectorXd &Q, VectorXd &QDot);
};

class Cassie {
public:
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
    Eigen::Quaterniond quat_pelvis;
    Eigen::Vector3d gyroscope;
    Eigen::Vector3d accelerometer;
    Eigen::VectorXd torque;

    Eigen::VectorXi iRotorMap;
    Eigen::VectorXi iJointMap;
    Eigen::VectorXi iEncoderMap;

    double leftContact;
    double rightContact;
    Eigen::VectorXd GRF;

    RigidBodyDynamics::Model         *model;
    Kinematics                       kinematics;
    Dynamics                         dynamics;
    RigidBodyDynamics::ConstraintSet constraints;

    Cassie(bool verbose=false);
    ~Cassie() {free(model);}
};

}


#endif // CASSIE_MODEL_HPP

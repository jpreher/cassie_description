#ifndef FROSTALL_HPP
#define FROSTALL_HPP

#include <Eigen/Dense>

class VectorWrap : public Eigen::MatrixXd
{
    Eigen::VectorXd &v;
public:
    VectorWrap(Eigen::VectorXd &v)
        : Eigen::MatrixXd(v), v(v)
    { }
    ~VectorWrap()
    {
        write();
    }
    void read()
    {
        this->resizeLike(v);
        *this << v;
    }
    void write()
    {
        v = *this;
    }
};

class Vector3Wrap : public Eigen::MatrixXd
{
    Eigen::Vector3d &v;
public:
    Vector3Wrap(Eigen::Vector3d &v)
        : Eigen::MatrixXd(v), v(v)
    { }
    ~Vector3Wrap()
    {
        write();
    }
    void read()
    {
        this->resizeLike(v);
        *this << v;
    }
    void write()
    {
        v = *this;
    }
};


class ScalarWrap : public Eigen::MatrixXd
{
    double &v;
public:
    ScalarWrap(double &v)
        : Eigen::MatrixXd(1, 1), v(v)
    {
        read();
    }
    ~ScalarWrap()
    {
        write();
    }

    void read()
    {
        (*this)(0) = v;
    }

    void write()
    {
        v = (*this)(0);
    }
};



// Constraints
#include <constraints/J_leftPoint_constraint.hh>
#include <constraints/J_rightPoint_constraint.hh>
#include <constraints/J_achilles_constraint.hh>
#include <constraints/J_leftHeel_constraint.hh>
#include <constraints/J_leftToe_constraint.hh>
#include <constraints/J_leftSole_constraint.hh>
#include <constraints/J_rightHeel_constraint.hh>
#include <constraints/J_rightToe_constraint.hh>
#include <constraints/J_rightSole_constraint.hh>
#include <constraints/J_left_fixed_constraint.hh>
#include <constraints/J_right_fixed_constraint.hh>
#include <constraints/Jdot_leftPoint_constraint.hh>
#include <constraints/Jdot_rightPoint_constraint.hh>
#include <constraints/Jdot_achilles_constraint.hh>
#include <constraints/Jdot_leftHeel_constraint.hh>
#include <constraints/Jdot_leftToe_constraint.hh>
#include <constraints/Jdot_leftSole_constraint.hh>
#include <constraints/Jdot_rightHeel_constraint.hh>
#include <constraints/Jdot_rightToe_constraint.hh>
#include <constraints/Jdot_rightSole_constraint.hh>
#include <constraints/Jdot_left_fixed_constraint.hh>
#include <constraints/Jdot_right_fixed_constraint.hh>
#include <constraints/p_leftPoint_constraint.hh>
#include <constraints/p_rightPoint_constraint.hh>
#include <constraints/p_achilles_constraint.hh>
#include <constraints/p_leftHeel_constraint.hh>
#include <constraints/p_leftToe_constraint.hh>
#include <constraints/p_leftSole_constraint.hh>
#include <constraints/p_rightHeel_constraint.hh>
#include <constraints/p_rightToe_constraint.hh>
#include <constraints/p_rightSole_constraint.hh>
#include <constraints/p_left_fixed_constraint.hh>
#include <constraints/p_right_fixed_constraint.hh>

// Dynamics
//#include <dynamics/Ce_cassie_v4.hh>
#include <dynamics/De_cassie_v4.hh>
#include <dynamics/Ge_cassie_v4.hh>
#include <dynamics/F_spring_cassie_v4.hh>

// Kinematics
#include <kinematics/pose_leftFoot.hh>
#include <kinematics/pose_rightFoot.hh>
#include <kinematics/J_leftFoot.hh>
#include <kinematics/J_rightFoot.hh>
#include <kinematics/position_leftFoot.hh>
#include <kinematics/position_rightFoot.hh>
#include <kinematics/J_position_leftFoot.hh>
#include <kinematics/J_position_rightFoot.hh>
#include <kinematics/p_leftToe.hh>
#include <kinematics/p_rightToe.hh>
#include <kinematics/J_leftToe.hh>
#include <kinematics/J_rightToe.hh>
#include <kinematics/p_com.hh>

// Outputs
#include <outputs/DLfya_LeftStanceActual.hh>
#include <outputs/DLfya_RightStanceActual.hh>
#include <outputs/DLfya_standCOM.hh>
#include <outputs/Jdot_yaLeftStance.hh>
#include <outputs/Jdot_yaRightStance.hh>
#include <outputs/Dya_LeftStanceActual.hh>
#include <outputs/Dya_RightStanceActual.hh>
#include <outputs/Dya_standCOM.hh>
#include <outputs/J_yaLeftStance.hh>
#include <outputs/J_yaRightStance.hh>
#include <outputs/Jya_standCOM.hh>
#include <outputs/dyaLeftStance.hh>
#include <outputs/dyaRightStance.hh>
#include <outputs/dyaStandCOM.hh>
#include <outputs/yaLeftStance.hh>
#include <outputs/yaRightStance.hh>
#include <outputs/yaStandCOM.hh>
#include <outputs/leftLegPitch.hh>
#include <outputs/rightLegPitch.hh>
#include <outputs/leftLegPitchVelocity.hh>
#include <outputs/rightLegPitchVelocity.hh>
#include <outputs/leftLegRoll.hh>
#include <outputs/rightLegRoll.hh>
#include <outputs/leftLegRollVelocity.hh>
#include <outputs/rightLegRollVelocity.hh>

#endif // FROSTALL_HPP

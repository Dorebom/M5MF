#pragma once

#include <ArduinoEigen.h>

struct DHParam
{
    double a;
    double alpha;
    double d;
    double theta;
};

class RigidTransform {
private:
    /* data */
    DHParam dhparam_;
    Eigen::Matrix4d htmat_coef;

public:
    RigidTransform(/* args */) {};
    ~RigidTransform() {};
    Eigen::Matrix4d rot_x(double aloha) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(aloha, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        return mat;
    };
    Eigen::Matrix4d rot_y(double beta) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()))
                .toRotationMatrix();
        return mat;
    };
    Eigen::Matrix4d rot_z(double gamma) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        return mat;
    };
    Eigen::Matrix4d trn_x(double a) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat(0, 3) = a;
        return mat;
    };
    Eigen::Matrix4d trn_y(double b) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat(1, 3) = b;
        return mat;
    };
    Eigen::Matrix4d trn_z(double c) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat(2, 3) = c;
        return mat;
    };
    //
    Eigen::Matrix4d rot_dz(double gamma) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat(0, 0) = -sin(gamma);
        mat(0, 1) = -cos(gamma);
        mat(1, 0) = cos(gamma);
        mat(1, 1) = -sin(gamma);
        return mat;
    };
    //
    void set_dhparam(Eigen::Vector3d pos, Eigen::Matrix3d rot_mat,
                     double theta) {
        // htmat_coef
        Eigen::Matrix4d htmat;
        htmat.block<3, 3>(0, 0) = rot_mat;
        htmat.block<3, 1>(0, 3) = pos;
        htmat(3, 3) = 1.0;
        //
        Eigen::Matrix4d rotz = rot_z(theta);
        //
        htmat_coef = htmat * rotz.inverse();
    };
    void set_dhparam(double a, double alpha, double d) {
        dhparam_.a = a;
        dhparam_.alpha = alpha;
        dhparam_.d = d;
        //
        htmat_coef = trn_x(a) * rot_x(alpha) * trn_z(d);
    };
    //
    Eigen::Matrix4d get_htmat(double theta) {
        return htmat_coef * rot_z(theta);
    }
    Eigen::Matrix4d get_dhtmat(double theta) {
        return htmat_coef * rot_dz(theta);
    }
};

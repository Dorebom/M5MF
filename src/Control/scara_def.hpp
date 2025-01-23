#pragma once

#include <ArduinoEigen.h>

#include <vector>

struct ScaraInfo
{
    double L1;
    double L2;
    double L3;

    Eigen::VectorXd m;
    std::vector<Eigen::Vector3d> s_i_i;
    std::vector<Eigen::Matrix3d> I_i_i;

    ScaraInfo() {
        L1 = 0.303;
        L2 = 0.303;
        L3 = 0.12;

        m.resize(3);
        m << 1.068, 1.068, 0.728;

        s_i_i.resize(3);
        s_i_i[0] << 0.19, 0.0, 0.027;
        s_i_i[1] << 0.19, 0.0, 0.027;
        s_i_i[2] << 0.04, 0.0, 0.046;

        I_i_i.resize(3);
        I_i_i[0] << 0.0517, 0.0, 0.0, 0.0, 0.0014, 0.0, 0.0, 0.0, 0.0513;
        I_i_i[1] << 0.0517, 0.0, 0.0, 0.0, 0.0014, 0.0, 0.0, 0.0, 0.0513;
        I_i_i[2] << 0.0034, 0.0, 0.0001, 0.0, 0.0023, 0.0014, 0.0001, 0.0014,
            0.0018;
    }
};
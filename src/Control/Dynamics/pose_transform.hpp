#pragma once

#include <ArduinoEigen.h>

/*
 * Pose = [position, rotvec]
 */

/*
 * Pose = [position, rotvec]
 */

class PoseTransform {
private:
public:
    PoseTransform() {};
    ~PoseTransform() {};

    // htmatrix 4x4 -> pose 6x1 >> OK
    static void transform_htmat2pose(const Eigen::Matrix4d& htmat,
                                     Eigen::VectorXd& pose) {
        pose.block<3, 1>(0, 0) = htmat.block<3, 1>(0, 3);
        Eigen::Matrix3d R = htmat.block<3, 3>(0, 0);

        // 回転角度thetaを求める
        double trace = R.trace();
        double cos_theta = (trace - 1.0) * 0.5;
        // クリップしてarccosの領域誤差を防止
        if (cos_theta > 1.0)
            cos_theta = 1.0;
        if (cos_theta < -1.0)
            cos_theta = -1.0;

        double theta = std::acos(cos_theta);

        // thetaが非常に小さい場合は、rotvecはほぼ0に近い
        const double eps = 1e-14;
        if (std::fabs(theta) < eps) {
            // ほとんど回転なし
            pose.block<3, 1>(3, 0) = Eigen::Vector3d::Zero();
        } else {
            double sin_theta = std::sin(theta);
            // 軸成分の計算: (R_{32} - R_{23}, R_{13} - R_{31}, R_{21} - R_{12})
            // / (2 sin theta)
            Eigen::Vector3d n;
            n << (R(2, 1) - R(1, 2)), (R(0, 2) - R(2, 0)), (R(1, 0) - R(0, 1));
            n /= (2.0 * sin_theta);

            // rotvec = theta * n
            pose.block<3, 1>(3, 0) = theta * n;
        }
    };
    // pose 6x1 -> htmatrix 4x4 >> OK
    static void transform_pose2htmat(const Eigen::VectorXd& pose,
                                     Eigen::Matrix4d& htmat) {
        Eigen::Vector3d p = pose.block<3, 1>(0, 0);
        Eigen::Vector3d q = pose.block<3, 1>(3, 0);

        htmat.block<3, 1>(0, 3) = p;
        // rotvec -> rotmat

        double th_ = q.norm();
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

        const double eps = 1e-14;
        if (th_ < eps) {
            htmat.block<3, 3>(0, 0) = R;
        } else {
            Eigen::Vector3d n = q / th_;

            Eigen::Matrix3d K;
            K << 0, -n.z(), n.y(), n.z(), 0, -n.x(), -n.y(), n.x(), 0;

            double s = std::sin(th_);
            double c = std::cos(th_);

            R = Eigen::Matrix3d::Identity() + s * K + (1.0 - c) * (K * K);

            htmat.block<3, 3>(0, 0) = R;
        }
    };
    // htmatrix 4x4 -> position 3x1, rotvec 3x1 >> OK
    static void transform_htmat2pose(const Eigen::Matrix4d& htmat,
                                     Eigen::Vector3d& p, Eigen::Vector3d& q) {
        Eigen::VectorXd pose(6);
        transform_htmat2pose(htmat, pose);
        p = pose.block<3, 1>(0, 0);
        q = pose.block<3, 1>(3, 0);
    };
    // position 3x1, rotvec 3x1 -> htmatrix 4x4  >> OK
    static void transform_pose2htmat(const Eigen::Vector3d& p,
                                     const Eigen::Vector3d& q,
                                     Eigen::Matrix4d& htmat) {
        Eigen::VectorXd pose(6);
        pose.block<3, 1>(0, 0) = p;
        pose.block<3, 1>(3, 0) = q;
        transform_pose2htmat(pose, htmat);
    };

    // htmatrix 4x4 -> position 3x1, quaternion 4x1 >> OK
    static void transform_htmat2pq(const Eigen::Matrix4d& htmat,
                                   Eigen::Vector3d& p, Eigen::Vector4d& q) {
        p = htmat.block<3, 1>(0, 3);
        q = Eigen::Quaterniond(htmat.block<3, 3>(0, 0)).coeffs();
    };
    // position 3x1, quaternion 4x1 -> htmatrix 4x4 >> OK
    static void transform_pq2htmat(const Eigen::Vector3d& p,
                                   const Eigen::Vector4d& q,
                                   Eigen::Matrix4d& htmat) {
        htmat.block<3, 1>(0, 3) = p;
        htmat.block<3, 3>(0, 0) = Eigen::Quaterniond(q).toRotationMatrix();
    };
    // htmat 4x4 -> position 3x1, rotation 3x3  >> OK
    static void transform_htmat2pos_rotmat(const Eigen::Matrix4d& htmat,
                                           Eigen::Vector3d& p,
                                           Eigen::Matrix3d& q) {
        p = htmat.block<3, 1>(0, 3);
        q = htmat.block<3, 3>(0, 0);
    };
    // position 3x1, rotation 3x3 -> htmat 4x4  >> OK
    static void transform_pos_rotmat2htmat(const Eigen::Vector3d& p,
                                           const Eigen::Matrix3d& q,
                                           Eigen::Matrix4d& htmat) {
        htmat.block<3, 1>(0, 3) = p;
        htmat.block<3, 3>(0, 0) = q;
    };
    static double deg2rad(double deg) {
        return deg * EIGEN_PI / 180.0;
    };
    static double rad2deg(double rad) {
        return rad * 180.0 / EIGEN_PI;
    };
    static void transform_rotmat2rotvec(const Eigen::Matrix3d& R,
                                        Eigen::Vector3d& q) {
        Eigen::VectorXd pose(6);
        Eigen::Matrix4d htmat = Eigen::Matrix4d::Identity();
        htmat.block<3, 3>(0, 0) = R;
        transform_htmat2pose(htmat, pose);
        q = pose.block<3, 1>(3, 0);
    };
};

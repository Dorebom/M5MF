#pragma once

#include <ArduinoEigen.h>

#include <vector>

#define GRAVITY 9.80665

class RobotDynamics {
private:
    /* data */
    // >> 運動学と同一
    int joint_num_;
    int tool_point_num_;
    //
    Eigen::VectorXd dq_;   // J1, ..., Jn
    Eigen::VectorXd ddq_;  // J1, ..., Jn
    //
    std::vector<Eigen::Matrix3d> R_from_prev_axis;  // {i}^R_i+1
    std::vector<Eigen::Vector3d> p_from_prev_axis;  // {i}^p_i,i+1
    // 外部環境変数
    // >> 台(Base)側
    Eigen::Vector3d ext_omega_0_0_;
    Eigen::Vector3d ext_dot_omega_0_0_;
    Eigen::Vector3d ext_ddot_p_0_0_;
    // >> 手先(tool)側
    Eigen::Vector3d ext_f_E_E_;
    Eigen::Vector3d ext_n_E_E_;
    // 中間変数 step1
    std::vector<Eigen::Vector3d> omega_i_i;      // {i}^w_i
    std::vector<Eigen::Vector3d> omega_dot_i_i;  // {i}^w_dot_i
    std::vector<Eigen::Vector3d> p_ddot_i_i;     // {i}^p_ddot_i
    // 中間変数 step2
    std::vector<Eigen::Vector3d> s_ddot_i_i;  // {i}^s_ddot_i
    std::vector<Eigen::Vector3d> f_hat_i_i;   // {i}^f_hat_i
    std::vector<Eigen::Vector3d> n_hat_i_i;   // {i}^n_hat_i
    // 中間変数 step3
    std::vector<Eigen::Vector3d> f_i_i;
    std::vector<Eigen::Vector3d> n_i_i;
    std::vector<Eigen::Vector3d> tau_i;  // {i}^tau_i
    // >> 定数
    // 上にある、p_from_prev_axisも定数だが、運動学で使うので、ここには入れない
    std::vector<Eigen::Vector3d> s_hat_i_i_;  // 重心位置
    std::vector<Eigen::Matrix3d> I_i_i_;
    Eigen::VectorXd m_;

    // Flag
    bool is_init_state = false;
    bool is_init_param = false;

    // Function
    void _calc_inv_dynamics_step_1();
    void _calc_inv_dynamics_step_2();
    void _calc_inv_dynamics_step_3();

public:
    RobotDynamics(int joint_num)
        : joint_num_(joint_num), tool_point_num_(joint_num + 1) {
        //
        dq_ = Eigen::VectorXd::Zero(joint_num_);
        ddq_ = Eigen::VectorXd::Zero(joint_num_);
        //	運動学では手先の位置も含めるため、そちらと合わせる意味で+1次元してる
        R_from_prev_axis.reserve(tool_point_num_);
        p_from_prev_axis.reserve(tool_point_num_);
        for (size_t i = 0; i < tool_point_num_; i++) {
            R_from_prev_axis.push_back(Eigen::Matrix3d::Identity());
            p_from_prev_axis.push_back(Eigen::Vector3d::Zero());
        }
        //
        ext_omega_0_0_ = Eigen::Vector3d::Zero();
        ext_dot_omega_0_0_ = Eigen::Vector3d::Zero();
        ext_ddot_p_0_0_ = Eigen::Vector3d::Zero();
        //
        s_hat_i_i_.reserve(joint_num_);
        I_i_i_.reserve(joint_num_);
        m_ = Eigen::VectorXd::Zero(joint_num_);
        // Step1
        omega_i_i.reserve(tool_point_num_);
        omega_dot_i_i.reserve(tool_point_num_);
        p_ddot_i_i.reserve(tool_point_num_);
        // Step2
        s_ddot_i_i.reserve(joint_num_);
        f_hat_i_i.reserve(joint_num_);
        n_hat_i_i.reserve(joint_num_);
        // Step3
        f_i_i.reserve(tool_point_num_);
        n_i_i.reserve(tool_point_num_);
        tau_i.reserve(joint_num_);
        //
        for (size_t i = 0; i < joint_num_; i++) {
            s_hat_i_i_.push_back(Eigen::Vector3d::Zero());
            I_i_i_.push_back(Eigen::Matrix3d::Zero());
            //
            s_ddot_i_i.push_back(Eigen::Vector3d::Zero());
            f_hat_i_i.push_back(Eigen::Vector3d::Zero());
            n_hat_i_i.push_back(Eigen::Vector3d::Zero());
            //
        }
        for (size_t i = 0; i < tool_point_num_; i++) {
            omega_i_i.push_back(Eigen::Vector3d::Zero());
            omega_dot_i_i.push_back(Eigen::Vector3d::Zero());
            p_ddot_i_i.push_back(Eigen::Vector3d::Zero());
            //
            f_i_i.push_back(Eigen::Vector3d::Zero());
            n_i_i.push_back(Eigen::Vector3d::Zero());
        }
    };
    ~RobotDynamics() {};
    //
    void set_state(Eigen::VectorXd& dq, Eigen::VectorXd& ddq,
                   std::vector<Eigen::Matrix3d>& R,
                   Eigen::Vector3d& ext_omega_0_0,
                   Eigen::Vector3d& ext_dot_omega_0_0,
                   Eigen::Vector3d& ext_ddot_p_0_0, Eigen::Vector3d& ext_f_E_E,
                   Eigen::Vector3d& ext_n_E_E) {
        update_state(dq, ddq, R, ext_omega_0_0, ext_dot_omega_0_0_,
                     ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E);
        is_init_state = true;
    };

    void set_param(const std::vector<Eigen::Vector3d>& p,
                   const std::vector<Eigen::Vector3d>& s_hat,
                   const std::vector<Eigen::Matrix3d>& I,
                   const Eigen::VectorXd& m) {
        update_param(p, s_hat, I, m);
        is_init_param = true;
    };
    //
    void update_state(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                      const std::vector<Eigen::Matrix3d>& R) {
        for (size_t i = 0; i < tool_point_num_; i++) {
            R_from_prev_axis[i] = R[i];
        }
        dq_ = dq;
        ddq_ = ddq;
    };
    //
    void update_base_state(const Eigen::Vector3d& ext_omega_0_0,
                           const Eigen::Vector3d& ext_dot_omega_0_0,
                           const Eigen::Vector3d& ext_ddot_p_0_0) {
        ext_omega_0_0_ = ext_omega_0_0;
        ext_dot_omega_0_0_ = ext_dot_omega_0_0;
        ext_ddot_p_0_0_ = ext_ddot_p_0_0;
    };
    //
    void update_tool_point_state(const Eigen::Vector3d& ext_f_E_E,
                                 const Eigen::Vector3d& ext_n_E_E) {
        ext_f_E_E_ = ext_f_E_E;
        ext_n_E_E_ = ext_n_E_E;
    };
    //
    void update_state(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                      const std::vector<Eigen::Matrix3d>& R,
                      const Eigen::Vector3d& ext_omega_0_0,
                      const Eigen::Vector3d& ext_dot_omega_0_0,
                      const Eigen::Vector3d& ext_ddot_p_0_0,
                      const Eigen::Vector3d& ext_f_E_E,
                      const Eigen::Vector3d& ext_n_E_E) {
        update_state(dq, ddq, R);
        update_base_state(ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0);
        update_tool_point_state(ext_f_E_E, ext_n_E_E);
    };
    //
    //
    void update_param(const std::vector<Eigen::Vector3d>& p,
                      const std::vector<Eigen::Vector3d>& s_hat,
                      const std::vector<Eigen::Matrix3d>& I,
                      const Eigen::VectorXd& m) {
        for (size_t i = 0; i < joint_num_; i++) {
            s_hat_i_i_[i] = s_hat[i];
            I_i_i_[i] = I[i];
        }
        for (size_t i = 0; i < tool_point_num_; i++) {
            p_from_prev_axis[i] = p[i];
        }
        m_ = m;
    };
    //
    void calc_tau(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                  const std::vector<Eigen::Matrix3d>& R,
                  const Eigen::Vector3d& ext_omega_0_0,
                  const Eigen::Vector3d& ext_dot_omega_0_0,
                  const Eigen::Vector3d& ext_ddot_p_0_0,
                  const Eigen::Vector3d& ext_f_E_E,
                  const Eigen::Vector3d& ext_n_E_E, Eigen::VectorXd& tau);
    void calc_hg(const Eigen::VectorXd& dq,
                 const std::vector<Eigen::Matrix3d>& R,
                 const Eigen::Vector3d& ext_omega_0_0,
                 const Eigen::Vector3d& ext_dot_omega_0_0,
                 const Eigen::Vector3d& ext_ddot_p_0_0,
                 const Eigen::Vector3d& ext_f_E_E,
                 const Eigen::Vector3d& ext_n_E_E, Eigen::VectorXd& hg) {
        calc_tau(dq, Eigen::VectorXd::Zero(joint_num_), R, ext_omega_0_0,
                 ext_dot_omega_0_0, ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E, hg);
    }
    void calc_h_and_g(const Eigen::VectorXd& dq,
                      const std::vector<Eigen::Matrix3d>& R,
                      const Eigen::Vector3d& ext_omega_0_0,
                      const Eigen::Vector3d& ext_dot_omega_0_0,
                      const Eigen::Vector3d& ext_ddot_p_0_0,
                      const Eigen::Vector3d& ext_f_E_E,
                      const Eigen::Vector3d& ext_n_E_E, Eigen::VectorXd& h,
                      Eigen::VectorXd& g) {
        calc_hg(dq, R, ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0,
                ext_f_E_E, ext_n_E_E, h);
        calc_tau(Eigen::VectorXd::Zero(joint_num_),
                 Eigen::VectorXd::Zero(joint_num_), R, ext_omega_0_0,
                 ext_dot_omega_0_0, ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E, g);
        h -= g;
    }
    void calc_M(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                const std::vector<Eigen::Matrix3d>& R,
                const Eigen::Vector3d& ext_omega_0_0,
                const Eigen::Vector3d& ext_dot_omega_0_0,
                const Eigen::Vector3d& ext_ddot_p_0_0,
                const Eigen::Vector3d& ext_f_E_E,
                const Eigen::Vector3d& ext_n_E_E, Eigen::MatrixXd& M) {
        Eigen::VectorXd temp_m = Eigen::VectorXd::Zero(joint_num_);
        //
        Eigen::VectorXd hg = Eigen::VectorXd::Zero(joint_num_);
        calc_hg(dq, R, ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0,
                ext_f_E_E, ext_n_E_E, hg);

        Eigen::VectorXd temp_acc;
        for (size_t i = 0; i < joint_num_; i++) {
            temp_acc = Eigen::VectorXd::Zero(joint_num_);
            temp_acc[i] = 1.0;
            //
            calc_tau(dq, temp_acc, R, ext_omega_0_0, ext_dot_omega_0_0,
                     ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E, temp_m);

            M.block(0, i, joint_num_, 1) = temp_m - hg;
        }
    };
    //
    void calc_M_and_hg(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                       const std::vector<Eigen::Matrix3d>& R,
                       const Eigen::Vector3d& ext_omega_0_0,
                       const Eigen::Vector3d& ext_dot_omega_0_0,
                       const Eigen::Vector3d& ext_ddot_p_0_0,
                       const Eigen::Vector3d& ext_f_E_E,
                       const Eigen::Vector3d& ext_n_E_E, Eigen::MatrixXd& M,
                       Eigen::VectorXd& hg) {
        calc_M(dq, ddq, R, ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0,
               ext_f_E_E, ext_n_E_E, M);
        calc_hg(dq, R, ext_omega_0_0, ext_dot_omega_0_0, ext_ddot_p_0_0,
                ext_f_E_E, ext_n_E_E, hg);
    };
    //
    void calc_M_h_and_g(const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq,
                        const std::vector<Eigen::Matrix3d>& R,
                        const Eigen::Vector3d& ext_omega_0_0,
                        const Eigen::Vector3d& ext_dot_omega_0_0,
                        const Eigen::Vector3d& ext_ddot_p_0_0,
                        const Eigen::Vector3d& ext_f_E_E,
                        const Eigen::Vector3d& ext_n_E_E, Eigen::MatrixXd& M,
                        Eigen::VectorXd& h, Eigen::VectorXd& g) {
        calc_M_and_hg(dq, ddq, R, ext_omega_0_0, ext_dot_omega_0_0,
                      ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E, M, h);
        calc_tau(Eigen::VectorXd::Zero(joint_num_),
                 Eigen::VectorXd::Zero(joint_num_), R, ext_omega_0_0,
                 ext_dot_omega_0_0, ext_ddot_p_0_0, ext_f_E_E, ext_n_E_E, g);
        h -= g;
    };
};

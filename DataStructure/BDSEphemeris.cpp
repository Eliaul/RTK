//
// Created by 39894 on 2023/3/14.
//

#include "BDSEphemeris.h"

#include "../Calculate/Constant.h"
#include "../LinerAlgebra/Matrix3d.hpp"

XYZCoordinate BDSEphemeris::CalSatePos() const
{
    double sin_2Phi_k = sin(2 * Phi_k);
    double cos_2Phi_k = cos(2 * Phi_k);
    double delta_u_k = cus * sin_2Phi_k + cuc * cos_2Phi_k;
    double delta_r_k = crs * sin_2Phi_k + crc * cos_2Phi_k;
    double delta_i_k = cis * sin_2Phi_k + cic * cos_2Phi_k;
    double u_k = Phi_k + delta_u_k;
    double r_k = A * (1 - ecc * cos(E_k)) + delta_r_k;
    double i_k = I_0 + delta_i_k + I_dot * t_k;
    double sin_i_k = sin(i_k);
    double cos_i_k = cos(i_k);
    double x_k1 = r_k * cos(u_k);
    double y_k1 = r_k * sin(u_k);
    double sin_Omega_k = sin(Omega_k);
    double cos_Omega_k = cos(Omega_k);
    double x_k = x_k1 * cos_Omega_k - y_k1 * cos_i_k * sin_Omega_k;
    double y_k = x_k1 * sin_Omega_k + y_k1 * cos_i_k * cos_Omega_k;
    double z_k = y_k1 * sin_i_k;
    if (!(PRN < 6 || PRN > 58))
    {
        return XYZCoordinate(x_k, y_k, z_k);
    }
    Omega_k = omega_0 + omega_dot * t_k - this->OMAGE_E() * toe_sec;
    sin_Omega_k = sin(Omega_k);
    cos_Omega_k = cos(Omega_k);
    double x_gk = x_k1 * cos_Omega_k - y_k1 * cos_i_k * sin_Omega_k;
    double y_gk = x_k1 * sin_Omega_k + y_k1 * cos_i_k * cos_Omega_k;
    double z_gk = y_k1 * sin_i_k;
    Vec3d xyz_gk = { x_gk, y_gk, z_gk };
    Matrix3d R_x = Matrix3d::Identity();
    R_x(2, 2) = cos(DEG2RAD(-5));
    R_x(2, 3) = sin(DEG2RAD(-5));
    R_x(3, 2) = -R_x(2, 3);
    R_x(3, 3) = R_x(2, 2);
    Matrix3d R_z = Matrix3d::Identity();
    R_z(1, 1) = cos(OMAGE_E() * t_k);
    R_z(1, 2) = sin(OMAGE_E() * t_k);
    R_z(2, 1) = -R_z(1, 2);
    R_z(2, 2) = R_z(1, 1);
    return XYZCoordinate::FromVec(R_z * R_x * xyz_gk);
}


Vec3d BDSEphemeris::CalSateVelocity() const
{
    double E_k_dot = (sqrt(this->GM() / pow(A, 3)) + Delta_N) / (1 - ecc * cos(E_k));
    double cos_2Phi_k = cos(2 * Phi_k);
    double sin_2Phi_k = sin(2 * Phi_k);
    double delta_u_k = cus * sin_2Phi_k + cuc * cos_2Phi_k;
    double delta_i_k = cis * sin_2Phi_k + cic * cos_2Phi_k;
    double delta_r_k = crs * sin_2Phi_k + crc * cos_2Phi_k;
    double i_k = I_0 + delta_i_k + I_dot * t_k;
    double u_k = Phi_k + delta_u_k;
    double cos_Omega_k = cos(Omega_k);
    double sin_Omega_k = sin(Omega_k);
    double sin_i_k = sin(i_k);
    double cos_i_k = cos(i_k);
    double r_k = A * (1 - ecc * cos(E_k)) + delta_r_k;
    double Phi_k_dot = sqrt((1 + ecc) / (1 - ecc)) * pow(cos(v_k / 2) / cos(E_k / 2), 2) * E_k_dot;
    double u_k_dot = 2 * (cus * cos_2Phi_k - cuc * sin_2Phi_k) * Phi_k_dot + Phi_k_dot;
    double r_k_dot = A * ecc * sin(E_k) * E_k_dot + 2 * (crs * cos_2Phi_k - crc * sin_2Phi_k) * Phi_k_dot;
    double I_k_dot = I_dot + 2 * (cis * cos_2Phi_k - cic * sin_2Phi_k) * Phi_k_dot;
    double x_k1 = r_k * cos(u_k);
    double y_k1 = r_k * sin(u_k);
    double x_k_dot = r_k_dot * cos(u_k) - r_k * u_k_dot * sin(u_k);
    double y_k_dot = r_k_dot * sin(u_k) + r_k * u_k_dot * cos(u_k);
    if (!(PRN < 6 || PRN > 58))
    {
        double Omega_k_dot = omega_dot - OMAGE_E();
        Matrix R_dot
                {
                        {cos_Omega_k, -sin_Omega_k * cos_i_k, -x_k1 * sin_Omega_k - y_k1 * cos_Omega_k * cos_i_k, y_k1 * sin_Omega_k * sin_i_k},
                        {sin_Omega_k, cos_Omega_k * cos_i_k, x_k1 * cos_Omega_k - y_k1 * sin_Omega_k * cos_i_k, -y_k1 * cos_Omega_k * sin_i_k},
                        {0, sin_i_k, 0, y_k1 * cos_i_k}
                };
        Vec v
                {
                        x_k_dot,
                        y_k_dot,
                        Omega_k_dot,
                        I_k_dot
                };
        return (Vec3d)(R_dot * v);
    }
    double omega_e = this->OMAGE_E();
    Omega_k = omega_0 + omega_dot * t_k - omega_e * toe_sec;
    double x_gk = x_k1 * cos_Omega_k - y_k1 * cos_i_k * sin_Omega_k;
    double y_gk = x_k1 * sin_Omega_k + y_k1 * cos_i_k * cos_Omega_k;
    double z_gk = y_k1 * sin_i_k;
    sin_Omega_k = sin(Omega_k);
    cos_Omega_k = cos(Omega_k);
    double x_gk_dot = x_k_dot * cos_Omega_k - x_k1 * sin_Omega_k * omega_dot - y_k_dot * cos_i_k * sin_Omega_k
                      - y_k1 * (-sin_i_k * I_k_dot * sin_Omega_k + cos_i_k * cos_Omega_k * omega_dot);
    double y_gk_dot = x_k_dot * sin_Omega_k + x_k1 * cos_Omega_k * omega_dot + y_k_dot * cos_i_k * cos_Omega_k
                      + y_k1 * (-sin_i_k * I_k_dot * cos_Omega_k - cos_i_k * sin_Omega_k * omega_dot);
    double z_gk_dot = y_k_dot * sin_i_k + y_k1 * cos_i_k * I_k_dot;
    double cos_phi1 = cos(DEG2RAD(-5));
    double sin_phi1 = sin(DEG2RAD(-5));
    double cos_phi = cos(omega_e * t_k);
    double sin_phi = sin(omega_e * t_k);
    return Vec3d(
            -sin_phi * omega_e * x_gk + cos_phi * x_gk_dot + cos_phi1 * (cos_phi * omega_e * y_gk + sin_phi * y_gk_dot)
            + sin_phi1 * (cos_phi * omega_e * z_gk + sin_phi * z_gk_dot),
            -cos_phi * omega_e * x_gk - sin_phi * x_gk_dot + cos_phi1 * (-sin_phi * omega_e * y_gk + cos_phi * y_gk_dot)
            + sin_phi1 * (-sin_phi * omega_e * z_gk + cos_phi * z_gk_dot),
            -sin_phi1 * y_gk_dot + cos_phi1 * z_gk_dot
    );
}
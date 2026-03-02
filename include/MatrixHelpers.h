#pragma once

#include <iostream>

#include "Eigen"
#include "Eigen/Geometry"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
}

/**
 * @brief Convert any type with a stream operator into a string.
 * @tparam T parameter type
 * @param value parameter value
 * @return a string representation of the parameter value
 */
template <typename T>
inline std::string to_string( const T& value )
{
    std::ostringstream ss;
    ss << value;
    return ss.str();
}

/**
 * @brief Convert degrees to radians.
 * @param deg Value in degrees.
 * @return Value in radians.
 */
inline double Deg2Rad(double deg){
    return deg * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 * @param rad Value in radians.
 * @return Value in degrees.
 */
inline double Rad2Deg(double rad){
    return rad * 180.0 / M_PI;
}

/**
 * @brief Convert an Eigen::Vector3d from radians to degrees.
 * @param vec The vector with values in radians.
 * @return A vector with values in degrees.
 */
inline Eigen::Vector3d Rad2Deg(Eigen::Vector3d& vec){
    Eigen::Vector3d ret_vec;
    for (int i=0; i<3; i++){
        ret_vec(i) = vec(i) * 180.0 / M_PI;
    }
    return ret_vec;
}

/**
 * @brief Convert an Eigen::Vector3d from degrees to radians.
 * @param vec The vector with values in degrees.
 * @return A vector with values in radians.
 */
inline Eigen::Vector3d Deg2Rad(Eigen::Vector3d& vec){
    Eigen::Vector3d ret_vec;
    for (int i=0; i<3; i++){
        ret_vec(i) = vec(i) * M_PI / 180.0;
    }
    return ret_vec;
}

/**
 * @brief Template function to convert an Eigen matrix or vector from radians to degrees.
 * @tparam Derived The base Eigen type.
 * @param matrix The data of the Eigen type in radians.
 * @return The same Eigen type with data in degrees.
 */
// Template function to convert Eigen matrix or vector from radians to degrees
template <typename Derived>
inline Eigen::MatrixBase<Derived>& Rad2Deg(Eigen::MatrixBase<Derived>& matrix) {
    matrix = matrix.unaryExpr([](typename Derived::Scalar angle) {
        return angle * 180.0 / M_PI;
    });
    return matrix;
}

/**
 * @brief Templated function to convert a raw C++ array to an Eigen matrix. Row-major indexing.
 * @tparam Scalar Array data type.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @param array The raw array (row-major: index = row*Cols + col). Matches apriltag matd_t layout.
 * @return The new Eigen matrix.
 */
template <typename Scalar, int Rows, int Cols>
inline Eigen::Matrix<Scalar, Rows, Cols> Array2EM(const Scalar* array) {
    Eigen::Matrix<Scalar, Rows, Cols> matrix;
    for (int i = 0; i < Rows; ++i) {
        for (int j = 0; j < Cols; ++j) {
            matrix(i, j) = array[i * Cols + j];
        }
    }
    return matrix;
}

/**
 * @brief Overload the stream operator for all types of Eigen matrices and vectors.
 * @tparam Derived The Eigen matrix or vector type.
 * @param os The ostream object to stream to.
 * @param matrix The Eigen matrix.
 * @return The ostream object with Eigen matrix data.
 */
template <typename Derived>
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixBase<Derived>& matrix) {
    os << "[";
    for (int i = 0; i < matrix.rows(); ++i) {
        os << "[";
        for (int j = 0; j < matrix.cols(); ++j) {
            os << matrix(i, j);
            if (j != matrix.cols() - 1) {
                os << ", ";
            }
        }
        os << "]";
        if (i != matrix.rows() - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

/**
 * @brief Create a rotation matrix from roll, pitch, and yaw in degrees.
 * Uses the 3-2-1 conversion ZYX (apply roll, then pitch, then yaw).
 *
 * Convention note (robot frame X forward, Y left, Z up):
 * With a strict right-hand rotation about +Y, positive pitch would tip the nose down.
 * This project defines positive pitch as "nose up", so the internal +Y rotation is negated.
 * @param V The ordered vector of roll, pitch, and yaw in degrees.
 * @return The 3x3 rotation matrix describing the orientation.
 */
inline Eigen::Matrix3d CreateRotationMatrix(const Eigen::Vector3d& V) {
    double roll = Deg2Rad(V(0));
    double pitch = Deg2Rad(V(1));
    double yaw = Deg2Rad(V(2));

    Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(-pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
    // Order: R = Rz * Ry * Rx (apply Rx first, then Ry, then Rz)
    Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();
    return R;
}


/**
 * @brief Normalize Euler angles (roll, pitch, yaw) to prefer the representation with
 * roll and pitch near 0. For ZYX with R = Rz(yaw)*Ry(-pitch)*Rx(roll), the equivalence
 * is (roll, pitch, yaw) <-> (roll+180, 180+pitch, yaw+180). Pick the one that minimizes
 * max(|roll|, |pitch|) to avoid the ±180 flip when the physical orientation is upright.
 */
inline Eigen::Vector3d NormalizeRPY(double roll, double pitch, double yaw) {
    auto wrap = [](double x) {
        while (x > 180.0) x -= 360.0;
        while (x < -180.0) x += 360.0;
        return x;
    };
    double r0 = wrap(roll), p0 = wrap(pitch), y0 = wrap(yaw);
    double r1 = wrap(roll + 180.0), p1 = wrap(180.0 + pitch), y1 = wrap(yaw + 180.0);
    double score0 = std::max(std::abs(r0), std::abs(p0));
    double score1 = std::max(std::abs(r1), std::abs(p1));
    return (score0 <= score1) ? Eigen::Vector3d{r0, p0, y0} : Eigen::Vector3d{r1, p1, y1};
}

/**
 * @brief Create a roll, pitch, yaw vector in degrees from a given rotation matrix.
 * 3-2-1 (Z Y X) conversion in the intrinsic frame. R = Rz(yaw) * Ry(-pitch) * Rx(roll)
 * Returns the normalized representation (roll and pitch near 0 when equivalent).
 * @param R The 3x3 rotation matrix describing the orientation.
 * @return The ordered vector of roll, pitch, and yaw in degrees.
 */
inline Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d angles = R.eulerAngles(2, 1, 0); // Z Y X

    double roll  = Rad2Deg(angles[2]);
    double pitch = -Rad2Deg(angles[1]);
    double yaw   = Rad2Deg(angles[0]);

    return NormalizeRPY(roll, pitch, yaw);
}

// Construct homogeneous 4x4 from Rotation matrix and translation vector
inline Eigen::Matrix4d makeTransform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

// Inverse of SE(3) homogeneous matrix (works for rigid transforms).
inline Eigen::Matrix4d invertTransform(const Eigen::Matrix4d &T) {
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
    Tinv.block<3,3>(0,0) = R.transpose();
    Tinv.block<3,1>(0,3) = -R.transpose() * t;
    return Tinv;
}

// Extract rotation (as Eigen::Matrix3d) and translation from T
inline Eigen::Matrix3d getRot(const Eigen::Matrix4d &T) { return T.block<3,3>(0,0); }
inline Eigen::Vector3d getTrans(const Eigen::Matrix4d &T) { return T.block<3,1>(0,3); }

/**
 * @brief Create a roll, pitch, yaw vector in degrees from a given rotation matrix. The matd_t datatype is used in the
 * Apriltag library. Handle gimbal lock when cos(pitch) is close to zero.
 * @param R The 3x3 rotation matrix describing the orientation.
 * @return The ordered vector of roll, pitch, and yaw in degrees.
 */
inline Eigen::Vector3d RotationMatrixToRPY(const matd_t* R) {
    Eigen::Matrix3d E_R;
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            E_R(i,j) = MATD_EL(R, i, j);
        }
    }
    return RotationMatrixToRPY(E_R);
}

/**
 * @brief Compare two Eigen objects elementwise for equality. Optionally specify a maximum tolerance per element. Both objects
 *  must be the same shape.
 * @param a The first Eigen object to compare.
 * @param b The second Eigen object to compare.
 * @param tolerance An optional tolerance to use when comparing elements. Defaults to 0.0.
 * @return true if arrays are equal to each other, false otherwise.
 */
template <typename DerivedA, typename DerivedB>
inline bool EigenEquals(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, double tolerance=0.0) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) {
        return false;
    }

    for (int i = 0; i < a.rows(); ++i) {
        for (int j = 0; j < a.cols(); ++j) {
            if (std::fabs(a(i,j) - b(i,j)) > tolerance) return false;
        }
    }
    return true;
}
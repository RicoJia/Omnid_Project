//
// Created by ricojia on 11/16/20.
//

#ifndef OMNID_PROJECT_TF2_ARMADILLO_H
#define OMNID_PROJECT_TF2_ARMADILLO_H

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <armadillo>
#include <math.h>
#include <tf2/LinearMath/Transform.h>

namespace tf2{
    typedef arma::Mat<double> mat;

    /// \brief Quaternion to a SE(3) matrix with the corresponding rotation matrix and 0 translation
    /// \param x - x in quaternion
    /// \param y - y in quaternion
    /// \param z - z in quaternion
    /// \param w - w in quaternion
    /// \return 4x4 SE(3) matrix with the corresponding rotation matrix and 0 translation
    inline mat::fixed<4, 4> quatToRotation(const double x, const double y, const double z, const double w) {
        mat::fixed<4, 4> R = {{(1 - 2 * y*y - 2 * z*z), (2 * x*y - 2 * z*w), (2 * x*z + 2 * y*w), 0},
                              {(2 * x*y + 2 * z*w), (1 - 2 * x*x - 2 * z*z), (2 * y*z - 2 * x*w), 0},
                              {(2 * x*z - 2 * y*w) , (2 * y*z + 2 * x*w) , (1 - 2 * x*x - 2 * y*y), 0},
                              {0, 0, 0, 1}};
        return R;
    }

    /// \brief Translation in the Cartesian Space to a SE(3) Matrix with the corresponding translation and identity rotation matrix
    /// \param x - Translation along the x axis
    /// \param y - Translation along the y axis
    /// \param z - Translation along the z axis
    /// \return 4x4 SE(3) matrix with the corresponding translation and identity rotation
    inline mat::fixed<4,4> xyzToTranslation(const double x, const double y, const double z){
        mat::fixed<4, 4> T = {{1, 0, 0, x},
                              {0, 1, 0, y},
                              {0, 0, 1, z},
                              {0, 0, 0, 1}};
        return T;
    }

    /// \brief Matrix3x3 to a SE(3) matrix with the corresponding rotation matrix and 0 translation
    /// \param R_tf2 Rotation matrix in Matrix3x3
    /// \return Euler angles expressed in tf2::Vector3
    inline mat::fixed<4,4> matrix3x3ToRotation(const Matrix3x3& R_tf2){
        auto row_0 = R_tf2.getRow(0);
        auto row_1 = R_tf2.getRow(1);
        auto row_2 = R_tf2.getRow(2);
        mat::fixed<4, 4> R = {{row_0.x(), row_0.y(), row_0.z(), 0},
                              {row_1.x(), row_1.y(), row_1.z(), 0},
                              {row_2.x(), row_2.y(), row_2.z(), 0},
                              {0, 0, 0, 1}};
        return R;
    }

    /// \brief Transform to Armadillo matrix for a tf2::Transform message
    /// \param tf Transform
    /// \param out output transform in mat::fixed<4, 4>
    /// \return SE(3) matrix for the transform
    inline void fromMsg (const Transform &msg, mat::fixed<4, 4>& out) {
        auto o = msg.getOrigin();
        out = (xyzToTranslation(o.x(), o.y(), o.z())) * (matrix3x3ToRotation(msg.getBasis()));
    }

    /// \brief Transform to Armadillo matrix for a geometry_msgs::Transform message
    /// \param tf - in geometry_msgs::Transform
    /// \param out output transform in mat::fixed<4, 4>
    /// \return SE(3) matrix for the transform
    inline void fromMsg(const geometry_msgs::Transform& in, mat::fixed<4,4>& out){
        auto p = in.translation;
        auto r = in.rotation;
        out = (xyzToTranslation(p.x, p.y, p.z)) * (quatToRotation(r.x, r.y, r.z, r.w));
    }

    /// \brief Armadillo Matrix to tf2::Transform messages
    /// \param T - Transform
    /// \return Transform in tf2::Transform
    inline Transform toMsg (const mat::fixed<4, 4> T, Transform& TF){
        Matrix3x3 R;
        R.setValue(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
        Vector3 p;
        p.setValue(T(0, 3), T(1, 3), T(2, 3));

        TF.setBasis(R);
        TF.setOrigin(p);
        return TF;
    }

    /// \brief convert armadillo matrix to pose
    /// \param T - Transform
    /// \param pose - pose message to be outputed
    /// \return - the same pose message as in the parameters
    inline geometry_msgs::Pose toMsg (const mat::fixed<4, 4> T, geometry_msgs::Pose& pose){
        Transform TF;
        toMsg(T, TF);
        toMsg(TF, pose);
        return pose;
    }

    /// \brief Armadillo Matrix to geometry_msgs::Transform
    /// \param T - Transform
    /// \return Transform in geometry_msgs::Transform
    inline geometry_msgs::Transform toMsg(const mat::fixed<4, 4> T){
        Transform  tf_tf2;
        tf_tf2 = toMsg(T, tf_tf2);
        geometry_msgs::Transform tf_geo = tf2::toMsg(tf_tf2);
        return tf_geo;
    }

}
#endif //OMNID_PROJECT_TF2_ARMADILLO_H

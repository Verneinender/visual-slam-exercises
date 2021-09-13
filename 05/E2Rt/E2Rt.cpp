//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    //JacobiSVD<Matrix3d> svd(E, ComputeThinU | ComputeThinV); 要么改成MatrixXd要么就是现在用的那样 
    //Error:"JacobiSVD: thin U and V are only available when your matrix has a dynamic number of columns.
    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV); 
    Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Vector3d sigma = svd.singularValues();
    //DiagonalMatrix<float, 3> SIGMA_ ((sigma(0,0)+sigma(1,0))/2, (sigma(0,0)+sigma(1,0))/2, 0);
    //Matrix3f SIGMA = SIGMA_.diagonal().asDiagonal();
    Matrix3d SIGMA;
    SIGMA << (sigma(0,0)+sigma(1,0))/2, 0, 0,
             0, (sigma(0,0)+sigma(1,0))/2, 0,
             0, 0, 0;

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    
    Matrix3d R1;
    Matrix3d R2;

    //Sophus::SO3 R_z1_ (0,0, M_PI/2);//从输出的形式可以看出，虽然SO3是李群，是旋转矩阵，但是输出形式还是向量（被转化成李代数输出）。
    //Sophus::SO3 R_z2_ (0,0, -M_PI/2);
    //Matrix3d R_z1 = R_z1_.matrix();
    //Matrix3d R_z2 = R_z2_.matrix();
    Matrix3d R_z1 = AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix(); //定义旋转矩阵,沿 Z 轴旋转 90 度
    Matrix3d R_z2 = AngleAxisd(-M_PI/2,Vector3d(0,0,1)).toRotationMatrix();

    t_wedge1 = U * R_z1 * SIGMA * U.transpose(); //caution the type of matrix!!!
    t_wedge2 = U * R_z2 * SIGMA * U.transpose();
    R1 = U * R_z1.transpose() * V.transpose();
    R2 = U * R_z2.transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}
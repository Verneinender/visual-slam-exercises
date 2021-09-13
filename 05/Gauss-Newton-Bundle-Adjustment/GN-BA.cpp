//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    

    std::ifstream open_p2d (p2d_file);
    std::ifstream open_p3d (p3d_file);
    if(!open_p2d.is_open()){
        cout << "p2d read fail" << endl;
        return -1;
    }else {
        double x, y;
        while(open_p2d >> x >> y) {
            Eigen::Vector2d p2d_ (x, y);
            p2d.push_back(p2d_);

        }
    }
    if(!open_p3d.is_open()){
        cout << "p3d read fail" << endl;
        return -1;
    }else {
        double x, y, z;
        while(open_p3d >> x >> y >> z) {
            Eigen::Vector3d p3d_ (x, y, z);
            p3d.push_back(p3d_);

        }
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());//如果他的条件返回错误，则终止程序执行

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector4d P = T_esti.matrix()*Vector4d(p3d[i](0,0),p3d[i](1,0),p3d[i](2,0),1);
            Vector3d u = K * Vector3d(P(0,0),P(1,0),P(2,0)) / P(2,0);
            Vector2d e = p2d[i] - Vector2d(u(0,0),u(1,0));
            cost += e.squaredNorm()/2;//求解2范数平方的1/2

	    // END YOUR CODE HERE
        

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            double x=P(0,0);
            double y=P(1,0);
            double z=P(2,0);

            J(0,0)=-fx/z;
            J(0,1)=0;
            J(0,2)=fx*x/(z*z);
            J(0,3)=fx*x*y/(z*z);
            J(0,4)=-fx-fx*(x*x)/(z*z);
            J(0,5)=fx*y/z;

            J(1,0)=0;
            J(1,1)=-fy/z;
            J(1,2)=fy*y/(z*z);
            J(1,3)=fy+fy*y*y/(z*z);
            J(1,4)=-fy*x*y/(z*z);
            J(1,5)=-fy*x/z;            


	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;//左乘扰动更新
        //pre-mulyiply disturbation to update


        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}

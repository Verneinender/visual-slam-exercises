#include "sophus/se3.h"
#include "sophus/so3.h"
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cmath>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

// path to trajectory file
string compare_file = "./compare.txt";



// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        const string& ID);
void read_compare(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &, 
                  vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &,
                  vector<Point3f> &,
                  vector<Point3f> &);

void icp_svd( const vector<Point3f>& pts1, const vector<Point3f>& pts2,
              Eigen::Matrix3d & R, Eigen::Vector3d& t);


int main(int argc, char **argv) {
    //vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_estimated;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_groundtruth;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_groundtruth_;
    vector<Point3f> t_e, t_g;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    read_compare(poses_estimated, poses_groundtruth, t_e, t_g);
    icp_svd(t_e, t_g, R, t);
    Sophus::SE3 T_eg(R,t);
    for(auto SE3_g:poses_groundtruth){
        SE3_g =T_eg * SE3_g; // T_e[i]=T_eg*T_g[i]
        poses_groundtruth_.push_back(SE3_g);
    }
    // draw trajectory in pangolin
    //DrawTrajectory(poses);
    DrawTrajectory(poses_estimated,poses_groundtruth," Before Align");
    DrawTrajectory(poses_estimated,poses_groundtruth_," After Align");
    return 0;
}



void read_compare(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & v1, 
                  vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & v2,
                  vector<Point3f> &t1,
                  vector<Point3f> &t2) {
    ifstream fin_compare;
    fin_compare.open(compare_file);
        if(!fin_compare) {
        cout<<"file not found!"<<endl;
    }
    while(!fin_compare.eof()) {
        double time_e, te_x, te_y, te_z, qe_x, qe_y, qe_z, qe_w, time_g, tg_x, tg_y, tg_z, qg_x, qg_y, qg_z, qg_w;
        fin_compare>>time_e>>te_x>>te_y>>te_z>>qe_x>>qe_y>>qe_z>>qe_w
                   >>time_g>>tg_x>>tg_y>>tg_z>>qg_x>>qg_y>>qg_z>>qg_w;
        
        t1.push_back(Point3d(te_x, te_y, te_z)); //将t取出，为了进行用icp进行计算
        t2.push_back(Point3d(tg_x, tg_y, tg_z));
        
        Eigen::Quaterniond qe = Eigen::Quaterniond(qe_x, qe_y, qe_z, qe_w).normalized(); // order in Eigen is xyzw
        Eigen::Vector3d Te(te_x, te_y, te_z);
        Sophus::SE3 SE3_e(qe,Te);
        v1.push_back(SE3_e);

        Eigen::Quaterniond qg = Eigen::Quaterniond(qg_x, qg_y, qg_z, qg_w).normalized(); // order in Eigen is xyzw
        Eigen::Vector3d Tg(tg_x, tg_y, tg_z);
        Sophus::SE3 SE3_g(qg,Tg);
        v2.push_back(SE3_g);

    }
}

void icp_svd( const vector<Point3f>& pts1, const vector<Point3f>& pts2,
              Eigen::Matrix3d & R, Eigen::Vector3d& t) {
    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) / N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f> q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout << "W=" << W << endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    R = U * ( V.transpose() ); //p1=R_12*p_2,注意R的意义，p2到p1的旋转关系
    //t = points of estimated - R * points of groundtruth, conversed is also right
    t = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R * Eigen::Vector3d ( p2.x, p2.y, p2.z );
               

    }



/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        const string& ID) {
    if (poses_e.empty() || poses_g.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    string windowtitle = "Trajectory Viewer" + ID;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind(windowtitle, 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses_e.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses_e[i], p2 = poses_e[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses_g.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = poses_g[i], p2 = poses_g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}


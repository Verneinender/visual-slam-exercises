#include "sophus/se3.h"
#include "sophus/so3.h"
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";
string estimated_file = "./estimated.txt";
string groundtruth_file = "./groundtruth.txt";



// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void read_estimated(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &);
void read_groundtruth(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &);

int main(int argc, char **argv) {
    //vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_estimated;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_groundtruth;

    /// implement pose reading code
    // start your code here (5~10 lines)
    /*ifstream fin(trajectory_file);
    if(!fin){
        cout<<"file not found!"<<endl;
    }
    while(!fin.eof()){
        double t, tx, ty, tz, qx, qy, qz, qw;
        fin>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Eigen::Quaterniond q(qx, qy, qz, qw); // order in Eigen is xyzw
        Eigen::Vector3d T(tx, ty, tz);
        Sophus::SE3 SE3_qt(q,T);
        poses.push_back(SE3_qt);

    }
    */
    // end your code here
    read_estimated(poses_estimated);
    read_groundtruth(poses_groundtruth);

    // draw trajectory in pangolin
    //DrawTrajectory(poses);
    DrawTrajectory(poses_estimated);
    DrawTrajectory(poses_groundtruth);
    return 0;
}

void read_estimated(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & vv) {
    ifstream fin_estimated (estimated_file);
        if(!fin_estimated) {
        cout<<"file not found!"<<endl;
    }
    while(!fin_estimated.eof()) {
        double t, tx, ty, tz, qx, qy, qz, qw;
        fin_estimated>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Eigen::Quaterniond q(qx, qy, qz, qw); // order in Eigen is xyzw
        Eigen::Vector3d T(tx, ty, tz);
        Sophus::SE3 SE3_qt(q,T);
        poses_estimated.push_back(SE3_qt);

    }

}

void read_groundtruth(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & vv) {
    ifstream fin_groundtruth (groundtruth_file);
        if(!fin_groundtruth) {
        cout<<"file not found!"<<endl;
    }
    while(!fin_groundtruth.eof()) {
        double t, tx, ty, tz, qx, qy, qz, qw;
        fin_groundtruth>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Eigen::Quaterniond q(qx, qy, qz, qw); // order in Eigen is xyzw
        Eigen::Vector3d T(tx, ty, tz);
        Sophus::SE3 SE3_qt(q,T);
        poses_groundtruth.push_back(SE3_qt);

    }


}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
#include <iostream>
#include "opencv2/opencv.hpp"
#include "string"
#include "csv_reader.hpp"
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace cv;
using namespace Eigen;

const string image_file ="/home/yihang/Desktop/Hongyu_Chen_Calib/coding_challenge_perception_internship/cameraimage.jpeg";
const string save_file  ="/home/yihang/Desktop/Hongyu_Chen_Calib/coding_challenge_perception_internship/result.jpeg";



using namespace cv;

// parameter //
double camera_x_m      = 1.95;
double camera_y_m      = 0.0;
double camera_z_m      = 1.29;
double camera_roll_rad = -0.012;
double camera_pitch_rad= 0.02;
double camera_yaw_rad  = 0.0;
double camera_focal_len_m = 6.05e-3;
double camera_pixel_dim_m = 3.75e-6;
int camera_size_u_px = 1280;
int camera_size_v_px = 960;



std::vector<std::vector<double>> read_file(){
    std::vector<std::vector<double>> datas;
    CsvReader::ReadCsvFile("/home/yihang/Desktop/Hongyu_Chen_Calib/coding_challenge_perception_internship/pointcloud.csv", datas);//read data from csv file        
    return datas;
}

void transform_R( vector<vector<double>> origin , vector<vector<double>>& pc_camera , Eigen::Matrix3d R, double t_x, double t_y, double t_z){
    
    for( int i = 0; i < origin[0].size(); ++i){
        pc_camera[0][i] = R(0,0)*(origin[0][i]-t_x)+ R(0,1)*(origin[1][i]-t_y) + R(0,2)*(origin[2][i]-t_z) ;
        pc_camera[1][i] = R(1,0)*(origin[0][i]-t_x)+ R(1,1)*(origin[1][i]-t_y) + R(1,2)*(origin[2][i]-t_z) ;
        pc_camera[2][i] = R(2,0)*(origin[0][i]-t_x)+ R(2,1)*(origin[1][i]-t_y) + R(2,2)*(origin[2][i]-t_z);

        // pc_camera[0][i] = R(0,0)*(origin[0][i])+ R(0,1)*(origin[1][i]) + R(0,2)*(origin[2][i])+t_x ;
        // pc_camera[1][i] = R(1,0)*(origin[0][i])+ R(1,1)*(origin[1][i]) + R(1,2)*(origin[2][i])+t_y ;
        // pc_camera[2][i] = R(2,0)*(origin[0][i])+ R(2,1)*(origin[1][i]) + R(2,2)*(origin[2][i])+t_z;

        pc_camera[3][i] = origin[3][i];
    }
}

void transform_B2C(vector<vector<double>> origin, vector<vector<double>>& pc_camera, double roll, double pitch, double yaw, double t_x, double t_y, double t_z){
    
    
    // cout<<pc_camera.size()<<endl;    // row
    // cout<<pc_camera[0].size()<<endl; // column
    Eigen::Vector3d euler_angle(roll, pitch, yaw);
    Eigen::Matrix3d rotation_matrix; // rotation from camera to world
    Eigen::Matrix3d rotation_matrix1; 
    rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());
    
    // rotation_matrix1 = rotation_matrix.transpose(); // rotation from world to camera
    rotation_matrix1 = rotation_matrix.inverse(); // rotation from world to camera
    // cout<<rotation_matrix1(0,0)<<endl;
    // cout<<rotation_matrix1(0,1)<<endl;
    // cout<<rotation_matrix1(0,2)<<endl;
    //q_c = R^(-1) * (p-t)
    // cout<< origin[0][0]<<"/"<< origin[1][0]<<"/"<<origin[2][0]<<endl;
    transform_R(origin , pc_camera , rotation_matrix1, t_x,  t_y,  t_z);
    //   cout << "\nrotation matrix1 =\n" << rotation_matrix << endl << endl;
    //   cout << "\nrotation matrix2 =\n" << rotation_matrix1 << endl << endl;
    // cout<< pc_camera[0][0]<<"/"<< pc_camera[1][0]<<"/"<<pc_camera[2][0]<<endl;
}



void projection(vector<vector<double>> pc_camera, cv::Mat& image, double fl , double pd){ // project the points in camera frame into uv frame 
vector<vector<double>> uv(3, vector<double>(pc_camera[0].size(), 0) );
// u
// v
// Intensity
double cx = 640.0 ;
double cy = 480.0 ;
double fx = fl/pd;
double fy = fl/pd;
double theta;
// cout<<cx<<"/"<<cy<<"/"<<fx<<"/"<<fy<<endl;
cv::Mat image_test=cv::Mat::zeros(image.size(),0);
int intensity;
int u;
int v;
for( int i = 0; i < pc_camera[0].size(); ++i){
    uv[0][i] =  -fx*pc_camera[1][i]/pc_camera[0][i]+cx; //  transformation: z=x, y=-z, x=-y;  something wired about the frame. is some information for the frame transformation missed?
    uv[1][i] =  -fy*pc_camera[2][i]/pc_camera[0][i]+cy; //  i use the frame transformation relationship of KITTI dataset and i get the correct result 
    uv[2][i] = pc_camera[3][i]*10000;

    //filtering based on the vectical angle
    theta = atan2(sqrt(pc_camera[1][i]*pc_camera[1][i]+pc_camera[2][i]*pc_camera[2][i]),pc_camera[0][i])*180/M_PI;
    if(theta > 60){
        uv[2][i]=0;
    }
}


for(int i =0; i<uv[0].size(); ++i){
    
    u = (int) uv[0][i];
    v = (int) uv[1][i];
    
    intensity = (int)uv[2][i];
    // if(u<0 || v<0){
    //     cout<<"wrong"<<endl;
    // }
    // cout<<intensity<<endl;

    if(u>0 && u< camera_size_u_px && v>0 && v< camera_size_v_px && intensity >=30){ // intensity show 
        // cout<<u<<"/"<<v<<intensity<<endl;
        image.at<Vec3b>(v,u) = Vec3b(0,0,intensity);
        // image_test.at<Vec3b>(v,u) = Vec3b(0,intensity,0);
        if(u>3 &&u< camera_size_u_px-3 && v>3 && v< camera_size_v_px-3);
        circle(image, Point(u,v),3, Scalar(0,0,intensity),-1,8,0);
    }
}
// imshow("img",image_test);
}




// void Intesity_test(std::vector<std::vector<double>> pc){
//     std::vector<double> ll(pc[0].size(), 0);
//     for( int i = 0; i < pc[0].size(); ++i){
//         ll[i]= pc[3][i];
//     }
//     auto it =max_element(ll.begin(), ll.end());
    
//     cout<<ll[5]<<endl<<endl;
//     cout<<"max"<<*(it)<<endl;
// }

int main() {
    cv::Mat image=cv::imread(image_file,cv::IMREAD_UNCHANGED);
    
    cout<<image.channels()<<endl;
    std::vector<std::vector<double>> point_cloud;
    
    point_cloud = read_file();
    cout<<"read_finish"<<endl;
    // Intesity_test(point_cloud);
    
    std::vector<std::vector<double>> point_cloud_camera(point_cloud.size(), std::vector<double>(point_cloud[0].size(),0)); 
    transform_B2C(point_cloud, point_cloud_camera, camera_roll_rad,camera_pitch_rad,camera_yaw_rad,camera_x_m ,camera_y_m,camera_z_m); //transform points into the camera frame
    cout<<"transform_finish"<<endl;

    projection(point_cloud_camera, image, camera_focal_len_m , camera_pixel_dim_m); //transform points into the camera uv frame
    cout<<"projection_finish"<<endl;

    
    /* debug  */
    // cout<< point_cloud[0][10]<<endl;
    // cout<< point_cloud[1][10]<<endl;
    // cout<< point_cloud[2][10]<<endl;
    // cout<< point_cloud[3][10]<<endl;
    //    cout<< point_cloud[0].size()<<endl;
    //    cout<< point_cloud.size()<<endl;
    // cv::Mat image_raw=cv::imread(image_file_raw+"9.png",cv::IMREAD_UNCHANGED);

    // Mat out_image=Mat::zeros(image_raw.size(),image_raw.type());
    // cout<<"type:"<<image_raw.type()<<endl;
    // for(int i=0;i<100;i++){
    //     for (int j= 0; j <100;j++) {
    //         image.at<Vec3b>(i,j)=Vec3b (254,0,0);

    //     }
    // }
    imshow("image",image);
    imwrite(save_file,image);
    // cout<<image.size()<<endl;
    // cout<<image.rows<<endl;
    // cout<<image<<endl;
    // cout<<image.type()<<endl;
    
    waitKey(0);
    return 0;
}

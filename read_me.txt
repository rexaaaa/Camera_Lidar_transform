Hongyu Chen
22.03.2022
############################


########## Dependencies

linux ubuntu18.04 

Eigen  3.3.4

opencv 3.2.0

########## Installation

1. change image path in /Hongyu_Chen_Calib/main.cpp #11

const string image_file ="**your path**/Hongyu_Chen_Calib/coding_challenge_perception_internship/cameraimage.jpeg";

2. change path to save image  in /Hongyu_Chen_Calib/main.cpp #12

const string save_file  ="**your path**/Hongyu_Chen_Calib/coding_challenge_perception_internship/result.jpeg";

3. cd **(your path )/**Hongyu_Chen_Calib

4. cd build/

5. cmake ..

6. make



########## Run

1. cd  (your path )/Hongyu_Chen_Calib/build 

2. ./calib 

########## result
result image is saved in  file /Hongyu_Chen_Calib/coding_challenge_perception_internship/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#define BASE_PATH "/home/mckros/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/"    // 资源路径

using namespace std;
using namespace cv;


/*******************************************************
* Function name ：cam_photo_pub
* Description   ：图像发布函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void cam_photo_pub_fun(string image_path,int num,image_transport::Publisher pub){
    char path_num[10];
    // 产生数据集照片的名称0000000000
    sprintf(path_num,"%010d",num);
    // 读入数据集图像
    Mat img = imread(string(BASE_PATH)+image_path+"/data/"+string(path_num)+".png");
    // 输出数据集图像路径
    cout<< string(BASE_PATH)+image_path+"/data/"+string(path_num)+".png"<<endl;
    // 将Mat类型的数据转换为sensor_msgs类型
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg();
    pub.publish(img_msg);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "cam_node");
    ROS_INFO("Cam_node Started success ");
    ros::NodeHandle n;
    // 声明的节点句柄初始化it
    image_transport::ImageTransport it(n);
    image_transport::Publisher cam_color_left_pub;      // 左侧彩色相机发布
    image_transport::Publisher cam_color_right_pub;     // 右侧彩色相机发布
    image_transport::Publisher cam_gray_left_pub;       // 左侧灰度相机发布
    image_transport::Publisher cam_gray_right_pub;      // 右侧灰度相机发布
    cam_color_left_pub = it.advertise("/kitti/cam_color_left",10);
    cam_color_right_pub = it.advertise("/kitti/cam_color_right",10);
    cam_gray_left_pub = it.advertise("/kitti/cam_gray_left",10);
    cam_gray_right_pub = it.advertise("/kitti/cam_gray_right",10);

    ros::Rate Rate(10);
    int num=1;
    while (ros::ok()){ 
        cam_photo_pub_fun("image_00",num,cam_gray_left_pub);
        cam_photo_pub_fun("image_01",num,cam_gray_right_pub);
        cam_photo_pub_fun("image_02",num,cam_color_left_pub);
        cam_photo_pub_fun("image_03",num,cam_color_right_pub);
        ++num;
        if(num == 154)num = 1;
        ros::spinOnce();
        Rate.sleep();
    }
}
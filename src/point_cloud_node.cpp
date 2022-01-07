#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "iostream"
#include "fstream"
#define BASE_PATH "/home/mckros/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/"    // 资源路径

using namespace std;



/*******************************************************
* Function name ：point_cloud_pub
* Description   ：点云数据发布函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void point_cloud_pub_fun(string bin_path,int num,ros::Publisher pub){
    char path_num[10];
    size_t length;
    sprintf(path_num, "%010d", num);
    ifstream inF(string(BASE_PATH)+bin_path+"/data/"+string(path_num)+".bin",ifstream::binary);
    cout<<string(BASE_PATH)+bin_path+"/data/"+string(path_num)+".bin"<<endl;
    inF.seekg(0, ios::end);// 将指针指向末尾
    length = inF.tellg();// 获取当前指针的位置
    unsigned char* data = new unsigned char[length]();// 读取文件数据
    inF.seekg(0,ios::beg);// 指针指向文件开头
    inF.get();
    if(!inF.eof()){
        cout<<"target reading"<<endl;
        inF.read(reinterpret_cast<char*>(data),sizeof(char)* length);// 读入文件数据流
        cout<<"finished reading"<<endl;
    }
    inF.close();
    for (int i = 0; i < length; i++)
    {
        cout<<data[i]<<endl;
    }
    
    
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_node");
    ROS_INFO("point_cloud_node Started success ");
    ros::NodeHandle n;
    ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/kitti/PointCloud2", 10);


    ros::Rate Rate(10);
    int num=1;
    point_cloud_pub_fun("velodyne_points",num,point_cloud_pub);
    while (ros::ok()){ 
        
        ++num;
        if(num == 154)num = 1;
        ros::spinOnce();
        Rate.sleep();
    }
    
}
#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <queue>
#include <ros/package.h>
#define POS_X 100
#define POS_Y 100
#define SIM_X 1000
#define SIM_Y 1000

const int bot_x = 50, bot_y = 50;
int step_move = -70;
const cv::Point origin(0, 100); //Wrt top left corner
int count = 0;
int debug=1;
ros::Publisher pub_point;
geometry_msgs::Pose2D msge;
//queue<geometry_msgs::Pose2D> temp;
int counter=0;
cv::Mat image;

//geometry_msgs::Pose2D* temp=new geometry_msgs::Pose2D [4];


geometry_msgs::Pose2D findTarget(cv::Mat img) {
    cv::Mat cdst, mdst;
    mdst = img - img;
    std::vector<cv::Vec4i>lines;
    cv::Point center_point;
    center_point.x = 0, center_point.y = 0;
    double center_angle = 0.0;
    cdst = img;
    cv::HoughLinesP(cdst, lines, 1, CV_PI / 180, 25, 15, 5);
    int image_halfy=0;
    cv::Point top(0,0), bottom(0,0);
    int c=0;


    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(mdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
    }

    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        center_point.x += (p[0] + p[2]) / 2;
        center_point.y += (p[1] + p[3]) / 2;
        if ((p[1] - p[3]) != 0) {
            double theta = std::atan2((p[1] - p[3]), (p[0] - p[2]));
            if (theta < 0)
                theta = 3.14 + theta;
            center_angle += (theta);
        }
        else
        {
            center_angle+=1.57;
        }
    }
    if (lines.size() != 0) {
        center_angle = center_angle / lines.size();
        center_point.x = center_point.x / lines.size();
        center_point.y = center_point.y / lines.size();
    }
    else
    {
	center_angle= 1.57;
	center_point.x= bot_x;
	center_point.y= bot_y;
    }
    double m = tan(center_angle);
    if (m == HUGE_VAL) {
        m = 10000;
    }
    cv::Point leftCenter(0, 0), rightCenter(0, 0);
    double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0;
    cv::Point proj,target;
    geometry_msgs::Pose2D target_pose;


        for (int i = 0; i < lines.size(); i++)
        {
            cv::Vec4i p = lines[i];
            cv::Point midPoint = cv::Point((p[0] + p[2]) / 2, (p[1] + p[3]) / 2);
            if(m==0)
                continue;
            double L11 = (0 - center_point.y) / m - (0 - center_point.x);
            double L22 = (midPoint.y - center_point.y) / m - (midPoint.x - center_point.x + 0.0);

            if (L11 * L22 > 0) {
                leftCenter.x += midPoint.x;
                leftCenter.y += midPoint.y;
                if ((lines[i][0] - lines[i][2]) != 0) {
                    leftSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
                }
                leftCount++;
            }
            else {
                rightCenter.x += midPoint.x;
                rightCenter.y += midPoint.y;
                if ((lines[i][0] - lines[i][2]) != 0) {
                    rightSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
                }
                rightCount++;
            }
        }
        if(leftCount!=0 && rightCount!=0){
            leftCenter.x /= leftCount;
            leftCenter.y /= leftCount;
            leftSlope /= leftCount;

            rightCenter.x /= rightCount;
            rightCenter.y /= rightCount;

            rightSlope /= rightCount;
            if ((center_point.x - leftCenter.x) < 50 || (leftCenter.x - center_point.x) < 50) {
                center_point.x += 150; //Target must not lie on the lane
                target.x = 150;
            }
            if ((rightCenter.x - center_point.x) < 50 || (center_point.x - rightCenter.x) < 50) {
                center_point.x = center_point.x - 150;
                target.x -= 150;
            }
        }

        proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
        proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
        target.x += proj.x + cos(center_angle) * step_move;
        target.y = proj.y + sin(center_angle) * step_move;
        center_angle = -1 * center_angle * 180 / CV_PI;
        target_pose.x = target.x;
        target_pose.y = (-1 * target.y + origin.y);
        target_pose.theta = center_angle;
        if(target_pose.x>img.cols)
            target_pose.x=img.cols;
        if(target_pose.y>img.rows)
            target_pose.y=img.rows;
        if(target_pose.y<0)
            target_pose.y=0;

        std::cout << "target.x: " << target_pose.x << " target.y: " << target_pose.y << std::endl;
        //std::cout << "proj.x: " << proj.x << " " << "proj.y: " << proj.y << std::endl;
        //cv::line(mdst, cv::Point(bot_x, bot_y), target, cv::Scalar(255), 2, 8);
        //cv::namedWindow("Center_path", cv::WINDOW_NORMAL);
        //cv::imshow("Center_path", mdst);
        //cv::waitKey(0);

    return target_pose;

}

void publishTarget(const geometry_msgs::Pose2D msg ) {

    geometry_msgs::Pose2D temp1;
    temp1.x=msg.x * 513 / SIM_X;
    temp1.y=msg.y * 513 / SIM_Y;
    cv::Mat rec;
    rec=image(cv::Rect(temp1.x,temp1.y,101,101));

    if(temp1.x>250 * 513 / SIM_X)
    {
        for(int i=0;i<rec.rows/2;i++)
        {
            for(int j=0;j<rec.cols;j++)
            {
                int temp=rec.at<uchar>(i,j);
                rec.at<uchar>(i,j)=rec.at<uchar>(rec.rows-i,j);
                rec.at<uchar>(rec.rows-i,j)=temp;
            }
        }
    }
    geometry_msgs::Pose2D result=findTarget(rec);
    if(temp1.x>250 * 513 / SIM_X)
    {
        result.y=100-result.y;
        if(result.y<0)
            result.y=0;
        result.theta*=-1;
    }
    pub_point.publish(result);
}

int main(int argc, char **argv) {
    std::string node_name= "lane_navigator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    msge.x=0;
    msge.y=0;
    msge.theta=0;
    std::string file_name;
    file_name=ros::package::getPath("simulator_ip")+"/src/"+"b.png";
    image=cv::imread(file_name,0);
    while(!image.data)
    {
        image=cv::imread(file_name,CV_LOAD_IMAGE_GRAYSCALE);
    }
    pub_point = node_handle.advertise<geometry_msgs::Pose2D>("/lane_navigator/proposed_target", 50);
    ros::Subscriber lanes_subscriber = node_handle.subscribe("/lane_detector1/lanes", 1, &publishTarget);
    while(ros::ok()){
    //node_handle.getParam(node_name + "/debug", debug);
    ros::spinOnce();
    }
    return 0;
}

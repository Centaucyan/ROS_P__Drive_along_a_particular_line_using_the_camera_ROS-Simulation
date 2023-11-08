#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 영상 관련 토픽을 구독하고 처리하기 위한 헤더 파일
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int err; // err 변수 : 외부 변수로 표현

void img_cb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;   //이미지 받을 OpenCV 형식의 데이터 구조를 가리키는 포인터 변수 선언
    // ROS에서 사용되는 이미지 메시지 'msg'를 OpenCV에서 사용되는 'BGR8' 형식으로 변환하여 cv_ptr 변수에 이미지 저장.
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
    int h = cv_ptr->image.rows; // 구독한 이미지 세로 크기 구하기
    int w = cv_ptr->image.cols; // 구독한 이미지 가로 크기 구하기
    cv::Mat hsv_img;    // OpenCV 라이브러리의 Mat 객체(행렬 데이터 구조)의 hsv_img 변수 선언
    cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);   // HSV로 변경. hsv_img 변수로 저장

    // 노란색 추출(색상 60/2 근처, 채도 100 이상, 명도 100 이상)
    // 참고(openCV에서는 색상은 360도로 표현안하고 반인 180도로 표현 됨)
    cv::Mat mask;   // OpenCV 라이브러리의 Mat 객체(행렬 데이터 구조)의 mask 변수 선언
    cv::Scalar lower_yellow = cv::Scalar(20, 100, 100); // 각 범위의 낮은 값
    cv::Scalar upper_yellow = cv::Scalar(40, 255, 255); // 각 범위의 높은 값
    inRange(hsv_img, lower_yellow, upper_yellow, mask); // inRange 객체 : lower와 upper 범위의 추출한 값(이미지)을 범위 내 값은 
                                                        // 흰색으로 표시. 범위 밖의 값은 검정색으로 표시되어 mask에 이미지가 저장 됨.

    int search_top = 3 * h / 4;      // 이미지 높이 상단 기준 3/4지점부터
    int search_bot = 3 * h / 4 + 20; // 3/4지점에서 +20까지의 영역 지정. 즉, 이미지 하단 부분

    // mask에 저장된 이미지에서 탐색 영역 밖의 값들은 모두 0으로 처리하여 검정색으로 바꿈
    for (int y=0; y<h; y++){
        for (int x=0; x<w; x++){
            if(y<search_top){
                mask.at<uchar>(y,x) = 0;    // uchar : 부호 없는 8비트 정수 데이터 타입 자료형
                                            // 8비트이므로 0~255까지 10진수 표현 가능
            }
            else if(y>=search_bot){
                mask.at<uchar>(y,x) = 0;
            }
        }
    }

    // 탐색 영역만 남긴 mask에 저장된 영상의 무게 중심 x,y 좌표 구하기 => 이미지의 크기의 중심이 아님.
    // 이미지에 남아있는 흰색 부분의 중심이다.
    // m00:객체 면적 / m10,m01:객체 중심에 대한 x,y 좌표 / m20,m11,m02:객체의 모양과 방향 관련
    // cv::Moments 관련 - https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
    cv::Moments m = moments(mask);
    if(m.m00 > 0){  // m00 > 0 의미는 면적이 있다는 의미이므로 이미지 감지가 되었다는 것을 의미
        int cx = m.m10 / m.m00; 
        int cy = m.m01 / m.m00;

        // 시각적 효과를 위해 무게중심에 빨간 원이 그려준 영상을 화면에 보임.
        // (이미지,circle 위치, circle 반지름, circle 색상, 원 채우기) -> 원 채우기 값이 양수이면 원 테두리 두께가 됨.
        cv::circle(cv_ptr->image, cv::Point(cx, cy), 20, CV_RGB(255,0,0), -1);
        err = cx - w/2; // err : 화면이 가운데에서 얼만큼 떨어져 있는가.
                        // +값이면 화면의 가운데보다 무게 중심이 오른쪽에,
                        // -값이면 화면의 가운데보다 무게 중심이 왼쪽에...
    }

    imshow("road window", cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "followbot");  // 노드 생성과 노드 이름 설정
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);  // image_transport 변수 it 선언. ROS에서 이미지를 효율적으로 전송할 때 사용
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);   // turtlebot에 내리는 이동 명령 선언.
    image_transport::Subscriber img_sub = it.subscribe("camera/rgb/image_raw", 1, img_cb);  //카메라 영상 구독. 토픽 이름, 메시지 큐 크기, 콜백 함수
    cv::namedWindow("road window"); // openCV 윈도우 선언. 카메라 영상을 화면에서 보기 위함

    ros::Rate loop_rate(30);
    geometry_msgs::Twist cmd;   // turtlebot에 명령을 내리기 위한 cmd 변수

    cmd.linear.x = 0.6; // 기본적으로 전진

    while(ros::ok){
        cmd.angular.z = -err/100.;  // err가 양수이면 차선이 화면 중앙에서 오른쪽에 있음
                                    // -> 차량을 오른쪽으로 돌려야 함.
                                    // err에 -를 곱해 음수로 만들면 차량은 -방향(우회전)으로 회전.
        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
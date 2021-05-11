#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "Comm.h"

using namespace std;
using namespace cv;

void dummy(int, void*);
void mouse_callback(int event, int x, int y, int flags, void* param);

int threshold1 = 90;
int target_x, target_y = 0;

Vec3b lower_blue1, upper_blue1, lower_blue2, upper_blue2, lower_blue3, upper_blue3;
Mat img_color;

int main() {

    int x, y;
    Point2f Input[4];
    Point2f Output[4];

    Input[0] = Point2f(88, 8);
    Input[1] = Point2f(551, 7);
    Input[2] = Point2f(85, 476);
    Input[3] = Point2f(556, 478);

    Output[0] = Point2f(0, 0);
    Output[1] = Point2f(531-85, 0);
    Output[2] = Point2f(0, 474-19);
    Output[3] = Point2f(531-85,474-19);

    Mat temp_cartisian = getPerspectiveTransform(Input, Output);

    namedWindow("img_color");
    setMouseCallback("img_color", mouse_callback);
    createTrackbar("threshold", "img_color", &threshold1, 255, dummy);
    setTrackbarPos("threshold", "img_color", threshold1);

    Mat img_hsv;

    VideoCapture cap(0);
    if (!cap.isOpened()) {

        cout << "Can't detect camera." << endl;
        return -1;
    }

    CComm serial;
    const char* port = "COM4";
    int mod = 0;

    cout << "Connecting Serial" << endl;
    serial.Open(port, 115200);//시리얼 송신은 3번 포트로 설정 되어 있음

    if (!serial.isOpen()) {//시리얼 통신 유무 파악
       cout << "Serial Connecting Failed" << endl;
       cout << "Retry Connecting Serial" << endl;
       serial.Open(port, 115200);//1회차 실패시 다시 한번 시도
       if (!serial.isOpen()) {
          cout << "Serial Connecting Failed" << endl;
          return -1;//재시도 실패시 프로그램 종료
       }
       else   cout << "Serial Connecting Success" << endl;
    }
    else   cout << "Serial Connecting Success" << endl;
    

    while (1)
    {
        cap.read(img_color);

        
        warpPerspective(img_color, img_color, temp_cartisian,Size(531 - 85+2, 474 - 19+2));//Size(437, 452)

        threshold1 = getTrackbarPos("threshold", "img_color");
        cvtColor(img_color, img_hsv, COLOR_BGR2HSV);

        Mat img_mask1, img_mask2, img_mask3, img_mask;
        inRange(img_hsv, lower_blue1, upper_blue1, img_mask1);
        inRange(img_hsv, lower_blue2, upper_blue2, img_mask2);
        inRange(img_hsv, lower_blue3, upper_blue3, img_mask3);
        img_mask = img_mask1 | img_mask2 | img_mask3;//범위 내 이미지 흰색으로 이진화

        Mat img_result;
        bitwise_and(img_color, img_color, img_result, img_mask);//img_mask범위 내에서 img_colordml &연산결과 모두 true img_result에 저장

        medianBlur(img_result, img_result, 3);
        //Opening
        erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        //Closing
        dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));


        //Labeling
        Mat img_labels, stats, centroids;
        int numOfLables = connectedComponentsWithStats(img_mask, img_labels,
            stats, centroids, 8, CV_32S);

        //영역박스 그리기
        int max = -1, idx = 0;
        for (int j = 1; j < numOfLables; j++) {
            int area = stats.at<int>(j, CC_STAT_AREA);
            if (max < area)
            {
                max = area;
                idx = j;
            }
        }
        int left = stats.at<int>(idx, CC_STAT_LEFT);
        int top = stats.at<int>(idx, CC_STAT_TOP);
        int width = stats.at<int>(idx, CC_STAT_WIDTH);
        int height = stats.at<int>(idx, CC_STAT_HEIGHT);

        string xlabel, ylabel;
        string xtarget, ytarget;
        string totlabel;

        xtarget = to_string(target_x);
        ytarget = to_string(target_y);

        if (target_x < 10)  xtarget = "00" + xtarget;
        else if (target_x < 100) xtarget = '0' + xtarget;
        
        if (target_y < 10)  ytarget = "00" + ytarget;
        else if (target_y < 100) ytarget = '0' + ytarget;


        if (width > 15 && height > 15 && width < 196) {
            x = (int)centroids.at<double>(idx, 0);
            y = (int)centroids.at<double>(idx, 1);
            circle(img_color, Point(x, y), (width + height) / 4, Scalar(255, 0, 0), 5);
            //circle(img_color, Point(left + width / 2, top + height / 2), 1, Scalar(0, 0, 255), 5);
            xlabel = to_string(x);
            if (x < 10) xlabel = "00" + xlabel;
            else if (x < 100) xlabel = '0' + xlabel;
            ylabel = to_string(y);
            if (y < 10) ylabel = "00" + ylabel;
            else if (y < 100) ylabel = '0' + ylabel;

            
            totlabel = xlabel + ylabel + xtarget + ytarget;
            const char* c = totlabel.c_str();
            serial.Write(c,12);

            //putText(img_color, totlabel, Point(left - width * 1.2, top), FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255));

        }
        else {
            totlabel = "000000" + xtarget + ytarget;
            const char* c = totlabel.c_str();
            serial.Write(c, 12);
        }
        circle(img_color, Point(target_x, target_y), 2, Scalar(255, 255, 0), 4);
        imshow("img_color", img_color);
        //imshow("img_result", img_result);
        if (waitKey(1) > 0)
            break;
    }
    serial.Close();
    destroyAllWindows();



    return 0;
}

void dummy(int, void*) {

}

void mouse_callback(int event, int x, int y, int flags, void* param) {

    if (event == EVENT_RBUTTONDOWN)//우클릭으로 통제
    {
        Vec3b color_pixel = img_color.at<Vec3b>(y, x);//클릭한 위치의 데이터를 받아옴

        Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);//한 픽셀로 구성된 이미지로 변경(cvtColor를 이용하여 hsv변환하기 때문)


        Mat hsv_color;
        cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);//BGR 색공간에서 HSV로 전환

        int hue = hsv_color.at<Vec3b>(0, 0)[0];
        int saturation = hsv_color.at<Vec3b>(0, 0)[1];
        int value = hsv_color.at<Vec3b>(0, 0)[2];

        cout << "hue = " << hue << endl;
        cout << "saturation = " << saturation << endl;
        cout << "value = " << value << endl;


        if (hue < 10)//HUE 값을 제한하여 비슷한 색만 추출
        {
            cout << "case 1" << endl;
            lower_blue1 = Vec3b(hue - 10 + 180, threshold1, threshold1);
            upper_blue1 = Vec3b(180, 255, 255);
            lower_blue2 = Vec3b(0, threshold1, threshold1);
            upper_blue2 = Vec3b(hue, 255, 255);
            lower_blue3 = Vec3b(hue, threshold1, threshold1);
            upper_blue3 = Vec3b(hue + 10, 255, 255);
        }
        else if (hue > 170)
        {
            cout << "case 2" << endl;
            lower_blue1 = Vec3b(hue, threshold1, threshold1);
            upper_blue1 = Vec3b(180, 255, 255);
            lower_blue2 = Vec3b(0, threshold1, threshold1);
            upper_blue2 = Vec3b(hue + 10 - 180, 255, 255);
            lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
            upper_blue3 = Vec3b(hue, 255, 255);
        }
        else
        {
            cout << "case 3" << endl;
            lower_blue1 = Vec3b(hue, threshold1, threshold1);
            upper_blue1 = Vec3b(hue + 10, 255, 255);
            lower_blue2 = Vec3b(hue - 10, threshold1, threshold1);
            upper_blue2 = Vec3b(hue, 255, 255);
            lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
            upper_blue3 = Vec3b(hue, 255, 255);
        }

        cout << "hue = " << hue << endl;
        cout << "#1 = " << lower_blue1 << "~" << upper_blue1 << endl;
        cout << "#2 = " << lower_blue2 << "~" << upper_blue2 << endl;
        cout << "#3 = " << lower_blue3 << "~" << upper_blue3 << endl;
    }

    if (event == EVENT_LBUTTONDOWN)//좌클릭으로 통제
    {
        target_x = x;
        target_y = y;
        cout << "xtarget : " << x << " ytarget : " << y << endl;
    }

}
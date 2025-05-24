#include <iostream>
#include "opencv_aee.hpp"
#include "pi2c.h"

using namespace cv;
using namespace std;

int main()
{
    //Setup camera
    setupCamera(320, 240);

    //Initialize I2C Communiation
    Pi2c arduino(9); //Adress of ESP32 (0x07)

    //Load reference images
    Mat arrowForward = readImage("forward.png");
    Mat arrowLeft = readImage("left.png");
    Mat arrowRight = readImage("right.png");

    //Preprocess reference images: convert to greyscale and threshold to binary
    Mat arrowFwdBW, arrowLeftBW, arrowRightBW;
    cvtColor(arrowForward, arrowFwdBW, COLOR_BGR2GRAY);
    cvtColor(arrowLeft, arrowLeftBW, COLOR_BGR2GRAY);
    cvtColor(arrowRight, arrowRightBW, COLOR_BGR2GRAY);
    threshold(arrowFwdBW, arrowFwdBW, 0, 255, THRESH_BINARY | THRESH_OTSU);
    threshold(arrowLeftBW, arrowLeftBW, 0, 255, THRESH_BINARY | THRESH_OTSU);
    threshold(arrowRightBW, arrowRightBW, 0, 255, THRESH_BINARY | THRESH_OTSU);

    //Standardize size
    Size standarSize(200,200);
    resize(arrowFwdBW, arrowFwdBW, standarSize);
    resize(arrowLeftBW, arrowLeftBW, standarSize);
    resize(arrowRightBW, arrowRightBW, standarSize);

    //Endless loop
    while(true)
    {
        Mat frame = captureFrame();
        if(frame.empty()){
            cerr << "Error: No frame captured" << endl;
            continue;
        }

        //Flip frame vertically and horizontally
        flip(frame, frame, 0);
        flip(frame, frame, 1);

        //Convert to HSV
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        Mat mask;
        inRange(hsv, Scalar(112, 25, 25), Scalar(179, 100, 147), mask);

        //Morphological Closing
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        //Find Contours
        vector<vector<Point>>contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if(!contours.empty()){
            double maxArea = 0.0;
            int maxIndex = -1;
            for(size_t i = 0; i < contours.size(); i++){
                double area = contourArea(contours[i]);
                if(area > maxArea){
                    maxArea = area;
                    maxIndex = static_cast<int>(i);
                }
            }
            if(maxIndex != -1){
                vector<Point>approx;
                approxPolyDP(contours[maxIndex], approx, 15, true);
                if(approx.size() == 4){
                    Mat symbol = transformPerspective(approx, frame, standarSize.width, standarSize.height);
                    if(!symbol.empty()){
                        Mat symbolGray;
                        cvtColor(symbol, symbolGray, COLOR_BGR2GRAY);
                        threshold(symbolGray, symbolGray, 0, 255, THRESH_BINARY| THRESH_OTSU);

                        //Compare binary symbol with each reference
                        float matchFwd = compareImages(symbolGray, arrowFwdBW);
                        float matchLeft = compareImages(symbolGray, arrowLeftBW);
                        float matchRight = compareImages(symbolGray, arrowRightBW);

                        cout << "MatchFwd: " << matchFwd << "MatchLeft: " << matchLeft << "MatchRight: " << matchRight << endl;
                        int direction = 4550;

                        if (matchFwd >= matchLeft && matchFwd >= matchRight){
                            cout <<"Symbol recognised : Forward " << endl;
                            direction = 4355;
                        }
                        else if (matchLeft >= matchFwd && matchLeft >= matchRight){
                            cout <<"Symbol recognised : Left " << endl;
                            direction = 4356
                        }
                        else if (matchRight >= matchFwd && matchRight >= matchLeft){
                            cout <<"Symbol recognised : Right " << endl;
                            direction = 4357
                        }

                        //Send direction value through I2C
                        arduino.i2cWriteArduinoInt(direction);

                        imshow("Detected Symbol (BW)", symbolGray);
                    }
                    else{
                        cout << "Perspective transform failed" << endl;
                    }
                }
                else {
                    cout << "Contour is not 4 corners" << endl;
                }
            }
        }
        imshow("Camera Feed", frame);
        imshow("Mask", mask);

        if ((waitKey(30) & 0xFF) == 27)
            break;
    }

    closeCV();
    return 0;
}
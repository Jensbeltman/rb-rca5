#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <string>

#define PARAMETER1START 1
#define PARAMETER2START 1
#define PARAMETER1END 100
#define PARAMETER2END 100
std::vector<cv::Mat> images;
bool centrumClick = true;
std::pair<float, float> centrum;
bool firstImage = true;
int imageIndex = 1;
char keyInput = '0';
int currentlyViewedImg = 0;

std::pair<int, std::vector<cv::Vec3f>> detect(cv::Mat img, bool display, int param1, int param2) {
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circs;
    cv::HoughCircles(gray, circs, cv::HOUGH_GRADIENT, 1, 10, param1, param2, 0, 0);

    for (size_t i = 0; i < circs.size(); i++) {
        cv::Point center(cvRound(circs[i][0]), cvRound(circs[i][1]));
        int radius = cvRound(circs[i][2]);
        //Center
        circle(img, center, 3, cv::Scalar(0, 0, 150), -1, 8, 0);
        //Outline
        circle(img, center, radius, cv::Scalar(0, 0, 150), 3, 8, 0);
    }

    cv::putText(img, "P1: " + std::to_string(param1) + ", P2: " + std::to_string(param2),
                  cv::Point(5, 150), cv::FONT_HERSHEY_PLAIN, 1.0,
                  cv::Scalar(150, 0, 250));

    int lineHeight = 0;
        for (int i = 0; i < circs.size(); i++) {

            std::ostringstream stats;
            stats.precision(2);
            stats << std::fixed << "Cx, Cy: (" << circs[i][0] << ", " << circs[i][1] << "), R: " << circs[i][2];

            cv::putText(img, stats.str(),
                  cv::Point(5, 20+lineHeight), cv::FONT_HERSHEY_PLAIN, 1.0,
                  cv::Scalar(150, 0, 250));
            lineHeight += 20;
        }

    if (display) {
        cv::imshow("", img);
    }
    
    return std::make_pair(circs.size(), circs);
}


void determineParameters(std::vector<cv::Mat> &imageList, std::vector<std::vector<std::array<float, 3>>> &circleParameters) {
    int parameterWeights[PARAMETER1END+1][PARAMETER2END+1] = {0}; //p1, p2, weight.
    bool firstImage = true;
    int passes = 1;


    for (int im_index = 0; im_index < imageList.size(); im_index++) {
        for (int p1 = PARAMETER1START; p1 <= PARAMETER1END; p1++) {
            for (int p2 = PARAMETER2START; p2 <= PARAMETER2END; p2++) {
                cv::Mat test_img = imageList[im_index].clone();
                std::pair<int, std::vector<cv::Vec3f>> img_specs = detect(test_img, 0, p1, p2);
                if (img_specs.first == circleParameters[im_index].size()) {
                    int numberOfEqCircles = 0;
                    for (int i = 0; i < img_specs.first; i++) {
                        for (int j = 0; j < img_specs.first; j++) {
                            if (
                                //Check if radius is the same:
                                img_specs.second[i][2]+5 >= circleParameters[im_index][j][2] &&
                                img_specs.second[i][2]-5 <= circleParameters[im_index][j][2] &&
                                //Check if x and y coordinates are the same:
                                img_specs.second[i][0]+5 >= circleParameters[im_index][j][0] &&
                                img_specs.second[i][0]-5 <= circleParameters[im_index][j][0] &&
                                img_specs.second[i][1]+5 >= circleParameters[im_index][j][1] &&
                                img_specs.second[i][1]-5 <= circleParameters[im_index][j][1]
                            ) {
                                numberOfEqCircles++;
                            }
                        }
                    }
                    if (numberOfEqCircles == img_specs.first) {
                        parameterWeights[p1][p2]++;
                    }
                }
                std::cout << "Passes: " << passes++ << std::endl;
            }
        }
    }
    std::cout << "Done" << std::endl;

    //Write to file:
    std::ofstream parameterFile;
    parameterFile.open("parameters.txt");
    for (int i = 0; i <= PARAMETER1END; i++) {
        for (int j = 0; j <= PARAMETER2END; j++) {
            if (parameterWeights[i][j] > 0)
                parameterFile << "P1: " << i << " P2: " << j << " W: " << parameterWeights[i][j] << " \n";
        }    
    }
    parameterFile.close();
}

void testWithImgCheck() {
    char keyPress;
    int expectedCircles;
    std::vector<std::array<float, 3>> circlesInImg;
    std::vector<std::vector<std::array<float, 3>>> acceptedCircles;
    std::vector<cv::Mat> acceptedImages;
    for (int im_index = 0; im_index < images.size(); im_index++) {
        cv::imshow("", images[im_index]);
        std::cout << "How many circles are seen in the image?" << std::endl;
        expectedCircles = cv::waitKey(0) - '0';
        std::cout << expectedCircles << std::endl;
        for (int i = PARAMETER1START; i <= PARAMETER1END; i++) {
            for (int j = PARAMETER2START; j <= PARAMETER2END; j++) {
                cv::Mat test = images[im_index].clone();
                cv::Mat show = images[im_index].clone();
                std::pair<int, std::vector<cv::Vec3f>> imgTest = detect(test, 0, i, j);
                if (imgTest.first == expectedCircles) {
                    detect(show, 1, i, j);
                    keyPress = cv::waitKey(0);
                    if (keyPress == 's') {
                        //Save parameters in a vector:
                        for (int circNum = 0; circNum < imgTest.second.size(); circNum++) {
                            circlesInImg.push_back({imgTest.second[circNum][0], imgTest.second[circNum][1], imgTest.second[circNum][2]});
                        }
                        acceptedCircles.push_back(circlesInImg);
                        acceptedImages.push_back(images[im_index]);
                        circlesInImg.clear();
                        std::cout << "Parameters saved." << std::endl;
                        break;
                    } else if (keyPress == 'd') {
                        
                        std::cout << "Image discarded." << std::endl;
                        break;
                    } else if (keyPress == 'z') {
                        //Not sure if needed for anything?...
                        std::cout << "Zero balls in image." << std::endl;
                        break;
                    }
                }
            }
            if (keyPress == 's' || keyPress == 'd' || keyPress == 'z') {
                keyPress = '0';
                break;
            }
        }
    }
    
    determineParameters(acceptedImages, acceptedCircles);
}

void dataToFile(int event, int x, int y, int flags, void* userdata) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        if (centrumClick) {
            std::cout << "x: " << x << " y: " << y << std::endl;
            centrum = std::make_pair(x, y);
            centrumClick = false;
        } else {
            std::pair<float, float> radiusPoint = std::make_pair(x, y);
            float radius = sqrt(pow(x - centrum.first, 2)+pow(y - centrum.second, 2));
            centrumClick = true;

            cv::Mat showCircles = images[currentlyViewedImg].clone();
            //Center
            circle(showCircles, cv::Point(centrum.first, centrum.second), 3, cv::Scalar(0, 0, 150), -1, 8, 0);
            //Outline
            circle(showCircles, cv::Point(centrum.first, centrum.second), radius, cv::Scalar(0, 0, 150), 3, 8, 0);

            cv::namedWindow("Circles", CV_WINDOW_AUTOSIZE);
            cv::imshow("Circles", showCircles);

            std::ofstream imageData;
            imageData.open("datasets/buildDataset/image_data.txt", std::fstream::app);

            std::cout << "x: " << centrum.first << " y: " << centrum.second << " r: " << radius << std::endl;
            std::cout << "s: save, r: retry, n: next image" << std::endl;

            keyInput = cv::waitKey(0);
            switch (keyInput) {
                case 's':
                    if (firstImage) {
                        imageData << "* " << imageIndex++ << "\n";
                        firstImage = false;
                    }
                    imageData << centrum.first << " " << centrum.second << " " << radius << "\n";
                    std::cout << "Saved." << std::endl;
                    break;
                case 'r':
                    std::cout << "Try again." << std::endl;
                    break;
            }
            
            imageData.close();
        }
    }
}

void buildDataset() {
    for (int i = 0; i < images.size(); i++) {
        cv::setMouseCallback("", dataToFile, NULL);
        cv::imshow("", images[i]);
        keyInput = cv::waitKey(0);
        if (keyInput != 'n') {
            i--;
        } else if (keyInput == 'n') {
            if (imageIndex <= images.size()) {
            std::ofstream imageData;
            imageData.open("datasets/buildDataset/image_data.txt", std::fstream::app);
            imageData << "* " << imageIndex++ << "\n";
            imageData.close();
            }
            currentlyViewedImg++;
        }
    }
}


int main() {
    
    std::cout << "b: build dataset, r: run test." << std::endl;
    char keyCheck;
    std::cin >> keyCheck;
    if (keyCheck == 'b') {
        std::ifstream sizeOfSet("datasets/buildDataset/set_size.txt");
        std::string setSize;
        sizeOfSet >> setSize;
        for (int i = 1; i <= std::stoi(setSize); i++) {
            images.push_back(cv::imread("datasets/buildDataset/test_img"+std::to_string(i)+".png", CV_LOAD_IMAGE_COLOR));
        }
        buildDataset();
    } else if (keyCheck == 'r') {
        system("./setDetails.sh");
        std::vector<std::array<float, 3>> circleParameters;
        std::vector<std::vector<std::array<float, 3>>> chosenCircles;
        std::string foldersStr;
        std::ifstream folders("datasets/details.txt");
        folders >> foldersStr;
        int numberOfFolders = std::stoi(foldersStr);
        
        for (int f = 1; f <= numberOfFolders; f++) {
            //Load in the number of images in each folder and then add the images to the vector.
            folders >> foldersStr;
            for (int i = 1; i <= std::stoi(foldersStr); i++) {
                images.push_back(cv::imread("datasets/images"+std::to_string(f)+"/test_img"+std::to_string(i)+".png", CV_LOAD_IMAGE_COLOR));
            }
            //Load in the data for the images and put them in the vector.
            bool firstImage = true;
            std::string lineOfData;
            std::ifstream img_data("datasets/images"+std::to_string(f)+"/image_data.txt");
            while (std::getline(img_data, lineOfData)) {
                if (lineOfData[0] == '*') {
                    if (firstImage) {
                        firstImage = false;
                    } else {
                        chosenCircles.push_back(circleParameters);
                        circleParameters.clear();
                    }
                } else {
                    float x, y, r;
                    int whichP = 0;
                    int prevSpace = 0;
                    for (int c = 0; c < lineOfData.length(); c++) {
                        if (lineOfData[c] == ' ') {
                            switch (whichP) {
                                case 0:
                                    x = std::stof(lineOfData.substr(0, c));
                                    prevSpace = c;
                                    whichP++;
                                    break;
                                case 1:
                                    y = std::stof(lineOfData.substr(prevSpace + 1, c - prevSpace));
                                    r = std::stof(lineOfData.substr(c + 1, lineOfData.length() - c - 1));
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                    circleParameters.push_back({x, y, r});
                }
            }
        }
        determineParameters(images, chosenCircles);
    }


    //testWithImgCheck();

    cv::waitKey(0);

    return 0;
}
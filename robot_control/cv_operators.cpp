#ifndef CV_OPERATORS_CPP
#define CV_OPERATORS_CPP

#include <opencv2/imgproc.hpp>


bool operator==(const cv::Point& l, const cv::Point& r)
{
    return (l.x == r.x && l.x == r.y);
}

bool operator<(const cv::Point& l, const cv::Point& r)
{
    if(l.x == r.x)
        return (l.y < r.y);
    return (l.x < r.x);
}

bool operator>(const cv::Point& l, const cv::Point& r)
{
    if(l.x == r.x)
        return (l.y > r.y);
    return (l.x > r.x);
}

#endif // CV_OPERATORS_CPP

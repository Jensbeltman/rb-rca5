#ifndef CV_OPERATORS_H
#define CV_OPERATORS_H

#include <opencv2/imgproc.hpp>

template <typename T>
bool operator==(const cv::Point_<T>& l, const cv::Point_<T>& r);

template <typename T>
bool operator<(const cv::Point_<T>& l, const cv::Point_<T>& r);

template <typename T>
bool operator>(const cv::Point_<T>& l, const cv::Point_<T>& r);


template <typename T>
bool operator==(const cv::Point_<T>& l, const cv::Point_<T>& r)
{
    return (l.x == r.x && l.y == r.y);
}

template <typename T>
bool operator<(const cv::Point_<T>& l, const cv::Point_<T>& r)
{
    if(l.x == r.x)
        return (l.y < r.y);
    return (l.x < r.x);
}



template <typename T>
bool operator>(const cv::Point_<T>& l, const cv::Point_<T>& r)
{
    if(l.x == r.x)
        return (l.y > r.y);
    return (l.x > r.x);
}


#endif // CV_OPERATORS_H

#ifndef BUFFER_H
#define BUFFER_H

#include <vector>

/***** Buffer to avoid detecting "flashing" circles *****/

class buffer
{
public:
    buffer();

    void addToBuf(bool in);
    bool isEmpty();
    int size();
    void setSize(int size);
    void setLowerB(int size);

private:
    void initBuffer();

    int bufSize;
    int lowerBound;
    std::vector<bool> buffer_vec;
};

#endif // BUFFER_H

#include "buffer.h"


void buffer::initBuffer() {
    for (int i = 0; i < bufSize; i++) {
        buffer_vec.push_back(0);
    }
}

buffer::buffer()
{
    bufSize = 10;
    lowerBound = 5;
    //initBuffer();
}

void buffer::addToBuf(bool in)
{
    if (in) {
        buffer_vec.push_back(in);
        if (buffer_vec.size() >= bufSize) {
            buffer_vec.erase(buffer_vec.begin());
        }
    } else if(buffer_vec.size() > 0)
        buffer_vec.erase(buffer_vec.begin());
}

bool buffer::isEmpty()
{
    bool out;
    (buffer_vec.size() > lowerBound) ? (out = false) : (out = true);
    return out;
}

int buffer::size()
{
    return buffer_vec.size();
}

void buffer::setSize(int size)
{
    bufSize = size;
}

void buffer::setLowerB(int size)
{
    lowerBound = size;
}

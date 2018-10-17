#ifndef FUZZY2_H
#define FUZZY2_H

#include <fl/Headers.h>

struct controlOutput2 {
    float direction;
    float speed;
};

class fuzzy2
{
public:
    fuzzy2();

    void process();
    virtual controlOutput2 getOutput();
    void setValues(float distance, float direction, float marbleDist);

private:
    fl::Engine*         p_engine;
    fl::InputVariable*  p_Odist;
    fl::InputVariable*  p_Odir;
    fl::OutputVariable* p_Sdir;
    fl::OutputVariable* p_speed;
    fl::InputVariable*  p_marbleC;
};

#endif // FUZZY2_H

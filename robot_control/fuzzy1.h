#ifndef FUZZY1_H
#define FUZZY1_H

#include<fl/Headers.h>

struct controlOutput {
    float direction;
    float speed;
};



class fuzzy1
{
public:
    fuzzy1();

    void process();
    virtual controlOutput getOutput();
    void setValues(float distance, float direction);

private:
    fl::Engine*         p_engine;
    fl::InputVariable*  p_Odist;
    fl::InputVariable*  p_Odir;
    fl::OutputVariable* p_Sdir;
    fl::OutputVariable* p_speed;
};

#endif // FUZZY1_H

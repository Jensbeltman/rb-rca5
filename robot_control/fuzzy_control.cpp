#include "fuzzy_control.h"
#include <fl/Headers.h>

fuzzy_control::~fuzzy_control()
{

}

void fuzzy_control::process()
{
    p_engine->process();
}

void fuzzy_control::setValues()
{

}

output fuzzy_control::getValues()
{
    output out;
    out.speed = p_drivingSpeed->getValue();
    out.direction = p_drivingDir->getValue();
    return out;
}

void obstacle_avoidance::init_controller()
{
    using namespace fl;
    //Setup of the controller

    //Engine:
    Engine* engine = new Engine;
    engine->setName("obstacleAvoidance");
    engine->setDescription("");

    //Inputvariable (distance):
    InputVariable* Odist = new InputVariable;
    Odist->setName("Odist");
    Odist->setDescription("");
    Odist->setEnabled(true);
    Odist->setRange(0.08, 10);
    Odist->setLockValueInRange(false);
    Odist->addTerm(new Ramp("far", 1.5, 10));
    Odist->addTerm(new Triangle("close", 0.5, 1.2, 1.8));
    Odist->addTerm(new Triangle("veryclose", 0.2, 0.8, 1.2));
    Odist->addTerm(new Ramp("tooclose", 0.8, 0.08));
    engine->addInputVariable(Odist);

    //Inputvariable (direction):
    InputVariable* Odir = new InputVariable;
    Odir->setName("Odir");
    Odir->setDescription("");
    Odir->setEnabled(true);
    Odir->setRange(-PI/2, PI/2);
    Odir->setLockValueInRange(false);
    //Odir->addTerm(new Sigmoid("leftB", -1, -2.26));
    Odir->addTerm(new Ramp("left", 0, -PI/2));
    Odir->addTerm(new Triangle("center", -0.2, 0, 0.2));
    Odir->addTerm(new Ramp("right", 0, PI/2));
    //Odir->addTerm(new Sigmoid("rightB", 1, 2.26));
    engine->addInputVariable(Odir);

    //Outputvariable (steering direction):
    OutputVariable* Sdir = new OutputVariable;
    Sdir->setName("Sdir");
    Sdir->setDescription("");
    Sdir->setEnabled(true);
    Sdir->setRange(-0.8, 0.8);
    Sdir->setLockValueInRange(false);
    Sdir->setAggregation(new Maximum);
    Sdir->setDefuzzifier(new Centroid(100));
    Sdir->setDefaultValue(0);
    Sdir->setLockPreviousValue(false);
    Sdir->addTerm(new Ramp("hardleft", 0.4, 0.8));
    Sdir->addTerm(new Triangle("left", 0, 0.25, 0.5));
    Sdir->addTerm(new Triangle("straight", -0.1, 0, 0.1));
    Sdir->addTerm(new Triangle("right", -0.5, -0.25, 0));
    Sdir->addTerm(new Ramp("hardright", -0.4, -0.8));
    engine->addOutputVariable(Sdir);

    //Outputvariable (speed):
    OutputVariable* speed = new OutputVariable;
    speed->setName("speed");
    speed->setDescription("");
    speed->setEnabled(true);
    speed->setRange(-1.2, 1.2);
    speed->setLockValueInRange(false);
    speed->setAggregation(new Maximum);
    speed->setDefuzzifier(new Centroid(100));
    speed->setDefaultValue(0);
    speed->setLockPreviousValue(false);
    speed->addTerm(new Ramp("fast", 0.7, 1.2));
    speed->addTerm(new Triangle("medium", 0.3, 0.5, 1));
    speed->addTerm(new Triangle("slow", 0, 0.3, 0.5));
    speed->addTerm(new Ramp("backward", 0, -1.2));
    engine->addOutputVariable(speed);

    //Ruleblock:
    RuleBlock* mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new Minimum);
    mamdani->setActivation(new General);

    mamdani->addRule(Rule::parse("if Odist is far then speed is fast", engine));
    mamdani->addRule(Rule::parse("if Odist is close then speed is medium", engine));
    mamdani->addRule(Rule::parse("if Odist is veryclose then speed is slow", engine));
    mamdani->addRule(Rule::parse("if Odist is tooclose then speed is slow", engine));

    mamdani->addRule(Rule::parse("if Odist is far then Sdir is straight", engine));
    mamdani->addRule(Rule::parse("if Odir is right and Odist is veryclose then Sdir is hardleft", engine));
    mamdani->addRule(Rule::parse("if Odir is left and Odist is veryclose then Sdir is hardright", engine));
    mamdani->addRule(Rule::parse("if Odir is right and Odist is close then Sdir is left", engine));
    mamdani->addRule(Rule::parse("if Odir is left and Odist is close then Sdir is right", engine));

    mamdani->addRule(Rule::parse("if Odir is right and Odist is tooclose then Sdir is hardleft", engine));
    mamdani->addRule(Rule::parse("if Odir is left and Odist is tooclose then Sdir is hardright", engine));

    mamdani->addRule(Rule::parse("if Odir is center and Odist is close then Sdir is right", engine));
    mamdani->addRule(Rule::parse("if Odir is center and Odist is veryclose then Sdir is hardright", engine));

    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    p_engine = engine;
    p_obstacleDist = Odist;
    p_obstacleDir = Odir;
    p_drivingSpeed = speed;
    p_drivingDir = Sdir;
}

void obstacle_avoidance::setValues(float obstacleDist, float obstacleDir)
{
    p_obstacleDist->setValue(obstacleDist);
    p_obstacleDir->setValue(obstacleDir);
}

void fuzzy_goal::init_controller()
{
    using namespace fl;
    //Setup of the controller

    //Engine:
    Engine* engine = new Engine;
    engine->setName("obstacleAvoidance");
    engine->setDescription("");

    //Inputvariable (distance to goal):
    InputVariable* goalDist = new InputVariable;
    goalDist->setName("goalDist");
    goalDist->setDescription("");
    goalDist->setEnabled(true);
    goalDist->setRange(0, 200);
    goalDist->setLockValueInRange(false);
    goalDist->addTerm(new Ramp("far", 5, 20));
    goalDist->addTerm(new Triangle("near", 1, 3, 10));
    goalDist->addTerm(new Ramp("veryclose", 1.5, 0));
    engine->addInputVariable(goalDist);

    //Inputvariable (angle to goal):
    InputVariable* goalAngle = new InputVariable;
    goalAngle->setName("goalAngle");
    goalAngle->setDescription("");
    goalAngle->setEnabled(true);
    goalAngle->setRange(-PI, PI);
    goalAngle->setLockValueInRange(false);
    goalAngle->addTerm(new Ramp("hardleft", -PI/18, -PI));
    goalAngle->addTerm(new Triangle("left", -PI/12, -PI/18, 0));
    goalAngle->addTerm(new Triangle("center", -0.1, 0, 0.1));
    goalAngle->addTerm(new Triangle("right", 0, PI/18, PI/12));
    goalAngle->addTerm(new Ramp("hardright", PI/18, PI));
    engine->addInputVariable(goalAngle);

    //Outputvariable (steering direction):
    OutputVariable* Sdir = new OutputVariable;
    Sdir->setName("Sdir");
    Sdir->setDescription("");
    Sdir->setEnabled(true);
    Sdir->setRange(-0.8, 0.8);
    Sdir->setLockValueInRange(false);
    Sdir->setAggregation(new Maximum);
    Sdir->setDefuzzifier(new Centroid(100));
    Sdir->setDefaultValue(0);
    Sdir->setLockPreviousValue(false);
    Sdir->addTerm(new Ramp("hardleft", 0.4, 0.8));
    Sdir->addTerm(new Triangle("left", 0, 0.25, 0.5));
    Sdir->addTerm(new Triangle("straight", -0.1, 0, 0.1));
    Sdir->addTerm(new Triangle("right", -0.5, -0.25, 0));
    Sdir->addTerm(new Ramp("hardright", -0.4, -0.8));
    engine->addOutputVariable(Sdir);

    //Outputvariable (speed):
    OutputVariable* speed = new OutputVariable;
    speed->setName("speed");
    speed->setDescription("");
    speed->setEnabled(true);
    speed->setRange(-1.2, 1.2);
    speed->setLockValueInRange(false);
    speed->setAggregation(new Maximum);
    speed->setDefuzzifier(new Centroid(100));
    speed->setDefaultValue(0);
    speed->setLockPreviousValue(false);
    speed->addTerm(new Ramp("fast", 0.7, 1.2));
    speed->addTerm(new Triangle("medium", 0.3, 0.5, 1));
    speed->addTerm(new Triangle("slow", 0, 0.3, 0.5));
    speed->addTerm(new Ramp("backward", -1.2, 0));
    engine->addOutputVariable(speed);

    //Ruleblock:
    RuleBlock* mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new Minimum);
    mamdani->setActivation(new General);

    mamdani->addRule(Rule::parse("if goalDist is far and goalAngle is hardright then speed is slow", engine));
    mamdani->addRule(Rule::parse("if goalDist is far and goalAngle is hardleft then speed is slow", engine));

    mamdani->addRule(Rule::parse("if goalDist is far and goalAngle is left then speed is fast", engine));
    mamdani->addRule(Rule::parse("if goalDist is far and goalAngle is right then speed is fast", engine));
    mamdani->addRule(Rule::parse("if goalDist is far and goalAngle is center then speed is fast", engine));

    mamdani->addRule(Rule::parse("if goalDist is near then speed is fast", engine));
    mamdani->addRule(Rule::parse("if goalDist is veryclose then speed is slow", engine));

    mamdani->addRule(Rule::parse("if goalAngle is right then Sdir is right", engine));
    mamdani->addRule(Rule::parse("if goalAngle is left then Sdir is left", engine));
    mamdani->addRule(Rule::parse("if goalAngle is hardright then Sdir is hardright", engine));
    mamdani->addRule(Rule::parse("if goalAngle is hardleft then Sdir is hardleft", engine));


    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    p_engine = engine;
    p_drivingSpeed = speed;
    p_drivingDir = Sdir;
    p_goalDist = goalDist;
    p_goalAngle = goalAngle;
}

void fuzzy_goal::setValues(float goalDist, float goalDir)
{
    p_goalDist->setValue(goalDist);
    p_goalAngle->setValue(goalDir);
}

void take_marble::init_controller()
{
    using namespace fl;
    //Setup of the controller

    //Engine:
    Engine* engine = new Engine;
    engine->setName("catchMarbles");
    engine->setDescription("");

    //Inputvariable (distance to obstacle):
    InputVariable* Odist = new InputVariable;
    Odist->setName("Odist");
    Odist->setDescription("");
    Odist->setEnabled(true);
    Odist->setRange(0.08, 5);
    Odist->setLockValueInRange(false);
    Odist->addTerm(new Ramp("far", 1, 5));
    Odist->addTerm(new Triangle("close", 0.2, 0.8, 1.5));
    Odist->addTerm(new Ramp("veryclose", 0.5, 0));
    engine->addInputVariable(Odist);

    //Inputvariable (direction to obstacle (only left or right)):
    InputVariable* Odir = new InputVariable;
    Odir->setName("Odir");
    Odir->setDescription("");
    Odir->setEnabled(true);
    Odir->setRange(-2.26, 2.26);
    Odir->setLockValueInRange(false);
    Odir->addTerm(new Ramp("left", -1, -2.26));
    //Odir->addTerm(new Triangle("center", -0.3, 0, 0.3));
    Odir->addTerm(new Ramp("right", 1, 2.26));
    engine->addInputVariable(Odir);

    //Outputvariable (steering direction):
    OutputVariable* Sdir = new OutputVariable;
    Sdir->setName("Sdir");
    Sdir->setDescription("");
    Sdir->setEnabled(true);
    Sdir->setRange(-0.4, 0.4);
    Sdir->setLockValueInRange(false);
    Sdir->setAggregation(new Maximum);
    Sdir->setDefuzzifier(new Centroid(100));
    Sdir->setDefaultValue(0);
    Sdir->setLockPreviousValue(false);
    Sdir->addTerm(new Ramp("left", 0, 0.4));
    Sdir->addTerm(new Triangle("straight", -0.1, 0, 0.1));
    Sdir->addTerm(new Ramp("right", 0, -0.4));
    engine->addOutputVariable(Sdir);

    //Outputvariable (speed):
    OutputVariable* speed = new OutputVariable;
    speed->setName("speed");
    speed->setDescription("");
    speed->setEnabled(true);
    speed->setRange(-1.2, 1.2);
    speed->setLockValueInRange(false);
    speed->setAggregation(new Maximum);
    speed->setDefuzzifier(new Centroid(100));
    speed->setDefaultValue(0);
    speed->setLockPreviousValue(false);
    speed->addTerm(new Ramp("forward", -0.1, 1.2));
    speed->addTerm(new Ramp("backward", 0.1, -1.2));
    engine->addOutputVariable(speed);

    //Outputvariable (distance to marble center)
    InputVariable* marbleC = new InputVariable;
    marbleC->setName("marbleC");
    marbleC->setDescription("");
    marbleC->setEnabled(true);
    marbleC->setRange(-200, 200);
    marbleC->setLockValueInRange(false);
    marbleC->addTerm(new Ramp("posLong", 30, 200));
    marbleC->addTerm(new Triangle("close", -55, 0, 55));
    marbleC->addTerm(new Ramp("negLong", -30, -200));
    engine->addInputVariable(marbleC);

    //Ruleblock:
    RuleBlock* mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new Minimum);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if marbleC is close then Sdir is straight", engine));
    mamdani->addRule(Rule::parse("if Odist is far or Odist is close or Odist is veryclose then speed is forward", engine));
    mamdani->addRule(Rule::parse("if Odist is close and Odir is right then Sdir is left", engine));
    mamdani->addRule(Rule::parse("if Odist is close and Odir is left then Sdir is right", engine));
    mamdani->addRule(Rule::parse("if marbleC is posLong then Sdir is left", engine));
    mamdani->addRule(Rule::parse("if marbleC is negLong then Sdir is right", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);

    //Initialise controllable variables:
    p_engine = engine;
    p_obstacleDist = Odist;
    p_obstacleDir = Odir;
    p_drivingDir = Sdir;
    p_drivingSpeed = speed;
    p_marbleDist = marbleC;
}

void take_marble::setValues(float obstacleDist, float obstacleDir, float marbleDist)
{
    p_obstacleDist->setValue(obstacleDist);
    p_obstacleDir->setValue(obstacleDir);
    p_marbleDist->setValue(marbleDist);
}

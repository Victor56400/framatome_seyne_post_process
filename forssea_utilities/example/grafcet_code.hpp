
#ifndef FORSSEA_DRIVERGRAFCET_HPP
#define FORSSEA_DRIVERGRAFCET_HPP

#include <forssea_utilities/grafcet.hpp>
using namespace forssea_utilities;

    
class DriverS0 : public Action
{
public:
    DriverS0(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS0() {}

    virtual bool execute();
};
                    
class DriverS0S1Condition : public GrafcetCondition
{
public:
    DriverS0S1Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS0S1"), grafcet) {}
    virtual ~DriverS0S1Condition() {}

    virtual bool eval();
};
                    
class DriverS1 : public Action
{
public:
    DriverS1(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS1() {}

    virtual bool execute();
};
                    
class DriverS1S2Condition : public GrafcetCondition
{
public:
    DriverS1S2Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS1S2"), grafcet) {}
    virtual ~DriverS1S2Condition() {}

    virtual bool eval();
};
                    
class DriverS2 : public Action
{
public:
    DriverS2(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS2() {}

    virtual bool execute();
};
                    
class DriverS2S3Condition : public GrafcetCondition
{
public:
    DriverS2S3Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS2S3"), grafcet) {}
    virtual ~DriverS2S3Condition() {}

    virtual bool eval();
};
                    
class DriverS3 : public Action
{
public:
    DriverS3(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS3() {}

    virtual bool execute();
};
                    
class DriverS3S4Condition : public GrafcetCondition
{
public:
    DriverS3S4Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS3S4"), grafcet) {}
    virtual ~DriverS3S4Condition() {}

    virtual bool eval();
};
                    
class DriverS4 : public Action
{
public:
    DriverS4(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS4() {}

    virtual bool execute();
};
                    
class DriverS4S3Condition : public GrafcetCondition
{
public:
    DriverS4S3Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS4S3"), grafcet) {}
    virtual ~DriverS4S3Condition() {}

    virtual bool eval();
};
                    
class DriverS1S0Condition : public GrafcetCondition
{
public:
    DriverS1S0Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS1S0"), grafcet) {}
    virtual ~DriverS1S0Condition() {}

    virtual bool eval();
};
                    
class DriverS3S30Condition : public GrafcetCondition
{
public:
    DriverS3S30Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS3S30"), grafcet) {}
    virtual ~DriverS3S30Condition() {}

    virtual bool eval();
};
                    
class DriverS30 : public Action
{
public:
    DriverS30(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS30() {}

    virtual bool execute();
};
                    
class DriverS30S3Condition : public GrafcetCondition
{
public:
    DriverS30S3Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS30S3"), grafcet) {}
    virtual ~DriverS30S3Condition() {}

    virtual bool eval();
};
                    
class DriverS30S300Condition : public GrafcetCondition
{
public:
    DriverS30S300Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS30S300"), grafcet) {}
    virtual ~DriverS30S300Condition() {}

    virtual bool eval();
};
                    
class DriverS300 : public Action
{
public:
    DriverS300(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS300() {}

    virtual bool execute();
};
                    
class DriverS3S300Condition : public GrafcetCondition
{
public:
    DriverS3S300Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS3S300"), grafcet) {}
    virtual ~DriverS3S300Condition() {}

    virtual bool eval();
};
                    
class DriverS300S0Condition : public GrafcetCondition
{
public:
    DriverS300S0Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS300S0"), grafcet) {}
    virtual ~DriverS300S0Condition() {}

    virtual bool eval();
};
                    
class DriverS4S40Condition : public GrafcetCondition
{
public:
    DriverS4S40Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS4S40"), grafcet) {}
    virtual ~DriverS4S40Condition() {}

    virtual bool eval();
};
                    
class DriverS40 : public Action
{
public:
    DriverS40(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS40() {}

    virtual bool execute();
};
                    
class DriverS40S3Condition : public GrafcetCondition
{
public:
    DriverS40S3Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS40S3"), grafcet) {}
    virtual ~DriverS40S3Condition() {}

    virtual bool eval();
};
                    
class DriverS40S300Condition : public GrafcetCondition
{
public:
    DriverS40S300Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS40S300"), grafcet) {}
    virtual ~DriverS40S300Condition() {}

    virtual bool eval();
};
                    
class DriverS4S41Condition : public GrafcetCondition
{
public:
    DriverS4S41Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS4S41"), grafcet) {}
    virtual ~DriverS4S41Condition() {}

    virtual bool eval();
};
                    
class DriverS41 : public Action
{
public:
    DriverS41(Grafcet* grafcet) : Action(grafcet) { this->initialize(); }
    virtual ~DriverS41() {}

    virtual bool execute();
};
                    
class DriverS41S3Condition : public GrafcetCondition
{
public:
    DriverS41S3Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS41S3"), grafcet) {}
    virtual ~DriverS41S3Condition() {}

    virtual bool eval();
};
                    
class DriverS41S300Condition : public GrafcetCondition
{
public:
    DriverS41S300Condition(Grafcet* grafcet) : GrafcetCondition(string("DriverS41S300"), grafcet) {}
    virtual ~DriverS41S300Condition() {}

    virtual bool eval();
};
                    
class DriverGrafcetGrafcet : public Grafcet
{
public:
    DriverGrafcetGrafcet(ros::NodeHandle* node_handle, const float& frequency);
    virtual ~DriverGrafcetGrafcet() {}

private:
    virtual void specificSetup();
};

#endif
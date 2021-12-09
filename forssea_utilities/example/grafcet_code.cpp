
#include <grafcet_code.hpp>
    
bool DriverS0::execute()
{
    return true;
}
                    
bool DriverS0S1Condition::eval()
{
    return false;
}
                    
bool DriverS1::execute()
{
    return true;
}
                    
bool DriverS1S2Condition::eval()
{
    return false;
}
                    
bool DriverS2::execute()
{
    return true;
}
                    
bool DriverS2S3Condition::eval()
{
    return false;
}
                    
bool DriverS3::execute()
{
    return true;
}
                    
bool DriverS3S4Condition::eval()
{
    return false;
}
                    
bool DriverS4::execute()
{
    return true;
}
                    
bool DriverS4S3Condition::eval()
{
    return false;
}
                    
bool DriverS1S0Condition::eval()
{
    return false;
}
                    
bool DriverS3S30Condition::eval()
{
    return false;
}
                    
bool DriverS30::execute()
{
    return true;
}
                    
bool DriverS30S3Condition::eval()
{
    return false;
}
                    
bool DriverS30S300Condition::eval()
{
    return false;
}
                    
bool DriverS300::execute()
{
    return true;
}
                    
bool DriverS3S300Condition::eval()
{
    return false;
}
                    
bool DriverS300S0Condition::eval()
{
    return false;
}
                    
bool DriverS4S40Condition::eval()
{
    return false;
}
                    
bool DriverS40::execute()
{
    return true;
}
                    
bool DriverS40S3Condition::eval()
{
    return false;
}
                    
bool DriverS40S300Condition::eval()
{
    return false;
}
                    
bool DriverS4S41Condition::eval()
{
    return false;
}
                    
bool DriverS41::execute()
{
    return true;
}
                    
bool DriverS41S3Condition::eval()
{
    return false;
}
                    
bool DriverS41S300Condition::eval()
{
    return false;
}
                    
DriverGrafcetGrafcet::DriverGrafcetGrafcet(ros::NodeHandle* node_handle, const float& frequency)
: Grafcet(string("DriverGrafcetGrafcet"), node_handle, frequency)
{
    this->setup();
}

void DriverGrafcetGrafcet::specificSetup()
{
    list<GrafcetTransition*> transitions;
    list<int> upstream_step_ids;
    list<int> downstream_step_ids;
    Action* action = 0;
    GrafcetStep* step = 0;

    // Step 0
    transitions.clear();
    action = new DriverS0(this);
                    
    // Tansition S0 / S1 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(0);
    downstream_step_ids.push_back(1);
    transitions.push_back(new GrafcetTransition(new DriverS0S1Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(0, transitions, action);
    step->setActive(true);
    this->addStep(step);
    // Step 1
    transitions.clear();
    action = new DriverS1(this);
                    
    // Tansition S1 / S2 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(1);
    downstream_step_ids.push_back(2);
    transitions.push_back(new GrafcetTransition(new DriverS1S2Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S1 / S0 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(1);
    downstream_step_ids.push_back(0);
    transitions.push_back(new GrafcetTransition(new DriverS1S0Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(1, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 2
    transitions.clear();
    action = new DriverS2(this);
                    
    // Tansition S2 / S3 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(2);
    downstream_step_ids.push_back(3);
    transitions.push_back(new GrafcetTransition(new DriverS2S3Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(2, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 3
    transitions.clear();
    action = new DriverS3(this);
                    
    // Tansition S3 / S4 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(3);
    downstream_step_ids.push_back(4);
    transitions.push_back(new GrafcetTransition(new DriverS3S4Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S3 / S30 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(3);
    downstream_step_ids.push_back(30);
    transitions.push_back(new GrafcetTransition(new DriverS3S30Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S3 / S300 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(3);
    downstream_step_ids.push_back(300);
    transitions.push_back(new GrafcetTransition(new DriverS3S300Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(3, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 4
    transitions.clear();
    action = new DriverS4(this);
                    
    // Tansition S4 / S3 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(4);
    downstream_step_ids.push_back(3);
    transitions.push_back(new GrafcetTransition(new DriverS4S3Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S4 / S40 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(4);
    downstream_step_ids.push_back(40);
    transitions.push_back(new GrafcetTransition(new DriverS4S40Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S4 / S41 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(4);
    downstream_step_ids.push_back(41);
    transitions.push_back(new GrafcetTransition(new DriverS4S41Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(4, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 30
    transitions.clear();
    action = new DriverS30(this);
                    
    // Tansition S30 / S3 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(30);
    downstream_step_ids.push_back(3);
    transitions.push_back(new GrafcetTransition(new DriverS30S3Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S30 / S300 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(30);
    downstream_step_ids.push_back(300);
    transitions.push_back(new GrafcetTransition(new DriverS30S300Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(30, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 300
    transitions.clear();
    action = new DriverS300(this);
                    
    // Tansition S300 / S0 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(300);
    downstream_step_ids.push_back(0);
    transitions.push_back(new GrafcetTransition(new DriverS300S0Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(300, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 40
    transitions.clear();
    action = new DriverS40(this);
                    
    // Tansition S40 / S3 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(40);
    downstream_step_ids.push_back(3);
    transitions.push_back(new GrafcetTransition(new DriverS40S3Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S40 / S300 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(40);
    downstream_step_ids.push_back(300);
    transitions.push_back(new GrafcetTransition(new DriverS40S300Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(40, transitions, action);
    step->setActive(false);
    this->addStep(step);
    // Step 41
    transitions.clear();
    action = new DriverS41(this);
                    
    // Tansition S41 / S3 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(41);
    downstream_step_ids.push_back(3);
    transitions.push_back(new GrafcetTransition(new DriverS41S3Condition(), upstream_step_ids, downstream_step_ids));
                                
    // Tansition S41 / S300 Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back(41);
    downstream_step_ids.push_back(300);
    transitions.push_back(new GrafcetTransition(new DriverS41S300Condition(), upstream_step_ids, downstream_step_ids));
                                
    step = new GrafcetStep(41, transitions, action);
    step->setActive(false);
    this->addStep(step);
}

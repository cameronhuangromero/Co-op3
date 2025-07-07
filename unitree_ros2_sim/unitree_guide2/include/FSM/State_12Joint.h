#include "FSM/FSMState.h"

class State_12Joint : public FSMState{
public:
    State_12Joint(CtrlComponents *ctrlComp);
    ~State_12Joint(){}
    void enter() override;
    void run();
    void exit();
    FSMStateName checkChange();

    int _motionTime = 0;
    float _interpolationPercent = 0.0f;
    float _interpolationDuration = 1000.0f;  // Number of cycles for full interpolation (e.g., 1000 cycles at 1kHz = 1 second)
    float _targetPos[12] = {0};              // desired final joint positions (e.g., all 0)
    float _startPos[12] = {0};               // filled in at entry


private:
   
};
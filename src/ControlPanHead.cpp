#include <module_pf/ControlPanHead.hpp>

control panGovernor;

//Initialize the controller
void initControl(double kp, double kd, double bkp, double bkd, double maxvel, double bmaxvel)
{
	panGovernor.KP = kp;
	panGovernor.KD = kd;
	panGovernor.bKP = bkp;
	panGovernor.bKD = bkd;
	panGovernor.MAX_VEL = maxvel;
	panGovernor.bMAX_VEL = bmaxvel;
	panGovernor.permissionToRotate = false;
	panGovernor.newGoal = 0;
	panGovernor.last_error = 0.0;
}

//Verifies that the requested goal is within the valid range
void validateGoal(double input)
{
	double data = input + panGovernor.feedbackPos;
	if(data < 1.56 && data > -1.56)
	{
		panGovernor.goalPosition = data;	
		panGovernor.newGoal++;
	}
}

//Updates the controller's output (velocity)
void controlCycle(void)
{
	double error;
	double derror;
	double base_error;
	double base_derror;

	error = panGovernor.goalPosition - panGovernor.feedbackPos;
	derror = error - panGovernor.last_error;

	base_error = panGovernor.feedbackPos;
	base_derror = base_error - panGovernor.base_last_error;

	panGovernor.output = panGovernor.KP * error + panGovernor.KD * derror;
	panGovernor.base_output = panGovernor.bKP * base_error + panGovernor.bKD * base_derror;

	if(panGovernor.output > panGovernor.MAX_VEL) panGovernor.output = panGovernor.MAX_VEL;
	else if(panGovernor.output < (-panGovernor.MAX_VEL)) panGovernor.output = (-panGovernor.MAX_VEL);

	if(panGovernor.base_output > panGovernor.bMAX_VEL) panGovernor.base_output = panGovernor.bMAX_VEL;
	else if(panGovernor.base_output < (-panGovernor.bMAX_VEL)) panGovernor.base_output = (-panGovernor.bMAX_VEL);

	panGovernor.last_error = error;
	panGovernor.base_last_error = base_error;
}

//Checks if the current position is within the error tolerance
int checkOutput(void)
{	
	if(panGovernor.output > -0.02 && panGovernor.output < 0.02) return 1;
	else return 0;
}

#ifndef CONTROL_PAN_HEAD
#define CONTROL_PAN_HEAD

/*
	control:

	Structure used for positioning the RB1's range of view towards a given direction, by
	turning its head and base, to the right or left.
*/

typedef struct 
{
	double KP;// Proportional constant for head-turning control
	double KD;// Derivative constant for head-turning control
	double bKP;// Proportional constant for base-turning control
	double bKD;// Derivative constant for base-turning control
	double MAX_VEL;// Maximum velocity allowed for head-turning
	double bMAX_VEL;// Maximum velocity allowed for base-turning

	bool permissionToRotate;// Flag to control to enable/disable the controller

	int newGoal;// Flag for a new goal head position

	double feedbackPos;// Current head position
	double goalPosition;// Current goal head position

	//Control variables
	double output;
	double base_output;
	double last_error;
	double base_last_error;
} control;

// Declaration of a controller
extern control panGovernor;

//Controller methods
void initControl(double,double,double,double,double,double);
void validateGoal(double);
void controlCycle(void);
int checkOutput(void);

#endif

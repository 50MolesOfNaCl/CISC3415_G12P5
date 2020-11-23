/**
 * follow-plan.cc
 *
 * Group 12: Jennie Kang, Edmund Lam, Jamila Toaha
 *
 * Project 5
 *
 *
 *
 * Date: November 2020
 *
 * Original file:
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to.
 *
 * Written by: Simon Parsons
 * Date:       10th November 2011
 *
 **/


#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <queue>
using namespace PlayerCc;


/**
 * Function headers
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp, int counter);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
void localize(BumperProxy& bp, LaserProxy& sp);
bool navigate(BumperProxy& bp, LaserProxy& sp, double x, double y);
double getTan(double xPos, double yPos, double xTarget, double yTarget);
float getDistance(double xPos, double yPos, double xTarget, double yTarget);


int  readPlanLength(void);
void readPlan(double*, int);
void printPlan(double*, int);
void writePlan(double*, int);
void getNextWaypoint();
double truncate(double num);
double x, y;	            // Hold our current waypoint
std::queue<double> myqueue; // Hold all of our waypoint
double prevX;
double prevY;

// Variables
float distanceBetweenWayPoints = 0; //In order to calculate proportional control, we'll need distance between way points
	
int counter = 0;
double speed;            // How fast do we want the robot to go forwards?
double turnrate;         // How fast do we want the robot to turn?
player_pose2d_t  pose;   // For handling localization data
player_laser_data laser; // For handling laser data
int setFullRotationCounter = 0;


// Navigation
float distance; 	   // How far is robot from the target?
double targetTan;  	   // Save the tan result for our target


// === State Variables
bool end = false;
int isStuckTimer = 0;


// The set of coordinates that makes up the plan

int pLength;
double* plan;

//void getNextWaypoint(myqueue);
/**
 * main()
 *
 **/

int main(int argc, char* argv[])
{


	// Set up proxies. These are the names we will use to connect to 
	// the interface to the robot.
	PlayerClient    robot("localhost");
	BumperProxy     bp(&robot, 0);
	Position2dProxy pp(&robot, 0);
	LocalizeProxy   lp(&robot, 0);
	LaserProxy      sp(&robot, 0);

	// Allow the program to take charge of the motors (take care now)
	pp.SetMotorEnable(true);

	/**
	// Plan handling
	//
	// A plan is an integer, n, followed by n doubles (n has to be
	// even). The first and second doubles are the initial x and y
	// (respectively) coordinates of the robot, the third and fourth
	// doubles give the first location that the robot should move to, and
	// so on. The last pair of doubles give the point at which the robot
	// should stop.
	**/
	pLength = readPlanLength(); // Find out how long the plan is from plan.txt
	plan = new double[pLength]; // Create enough space to store the plan
	readPlan(plan, pLength);    // Read the plan from the file plan.txt.
	printPlan(plan, pLength);   // Print the plan on the screen
	writePlan(plan, pLength);   // Write the plan to the file plan-out.txt
	getNextWaypoint();			// Initialize X and Y

	// Main control loop
	while (true)
	{
		// Update information from the robot.
		robot.Read();
		// Read new information about position
		pose = readPosition(lp, counter);
		// Print data on the robot to the terminal
		printRobotData(bp, pose);
		// Print information about the laser. Check the counter first to stop
		// problems on startup
		

		//=== Checks if robot is stuck (2 cases: 1- bumps into noise or 2- spinning around for 
		// too long without detecting viable blob. Note: it is not considered stuck if it has arrived). 
		// If it is stuck will backup, and turn 30 degrees

		if( isStuckTimer > 0 ) {
		
		 	//** Robot backs up for 2 seconds while rotating itself, and moves forwards for 4 seconds- 
			//	takes into account that there's no bumpers on the back, hence only 2 seconds spent backing up
			// when 4 seconds is up, it sets itself as unstuck, so it can restart scanning for blobs
			// and continue with rest of while loop
			if (isStuckTimer++ < 20 ){
				speed = -.3;
				turnrate = dtor(40); // make it rotate 60 degrees
			} else if (isStuckTimer >= 20 && isStuckTimer < 80 ){
		 	 	speed = .5;	
				turnrate = 0;	
			} 
			else {
				isStuckTimer = 0;  //resets timer

			}
				

		
    			std::cout << "isStuck speed" << speed  << std::endl;
   			//std::cout << "isStuck yaw" << yawRobot  << std::endl;
  		    	std::cout << "isStuckTimer" << isStuckTimer << ","  << std::endl;

			
			pp.SetSpeed(speed, turnrate); // need to set this, so robot can skip the rest of this cycle

			// ** do not proceed with the rest of this loop since robot is stuck
			continue;
		}


		isStuckTimer = 0;
		turnrate = 0; 
		speed = 0;





		//For testing purposes, prints distance between way points
		std::cout << "Distance Between Way Points: " << distanceBetweenWayPoints << std::endl;
		distance = getDistance (pose.px, pose.py, x, y); 

		if (counter > 2) {
			printLaserData(sp);
		}
		std::cout << "Current targets X: " << x << " Y: " << y << std::endl;

		// It navigates to destination way point
		navigate(bp, sp, x, y);

		// What are we doing?
		std::cout << "Speed: " << speed << std::endl;
		std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

		// Send the commands to the robot
		pp.SetSpeed(speed, turnrate);

		//  If waypoint reached and not the last one, get the next one
		if (truncate(pose.px) == truncate(x) && truncate(pose.py) == truncate(y))
		{
			if (!myqueue.empty()) {
				getNextWaypoint();
			}
			else {
				// WE ARE DONE
				std::cout << "Arrived to Destination. Program terminated." << std::endl;
				pp.SetSpeed(0, 0);				
				break;
			
			}
		}
		// In event of perpendicular alignment to waypoint
		// If bot lines up horizontally to waypoint
		else if (truncate(pose.px) == truncate(x) && truncate(pose.py) != truncate(y)) {

		}
		// if bot lines up on the vertically to waypoint
		else if (truncate(pose.px) != truncate(x) && truncate(pose.py) == truncate(y)) {

		}
		else {

		}
		// Count how many times we do this
		counter++;


		if(bp[0] == 1 || bp[1] == 1){
			speed = 0;
			turnrate = 0;
			isStuckTimer++;

		}
	}

} // end of main()

/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy.
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose.
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp, int counter)
{

	player_localize_hypoth_t hypothesis;
	player_pose2d_t          tempPose;
	uint32_t                 hCount;

	//need some messing around to avoid a crash when the proxy is starting up
	hCount = lp.GetHypothCount();

	if (hCount > 0) {
		hypothesis = lp.GetHypoth(0);
		pose = hypothesis.mean;
	}


	return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{

	double maxRange, minLeft, minRight, range, bearing, middleScanLine, leftRange, rightRange;
	int points;

	maxRange = sp.GetMaxRange();
	minLeft = sp.MinLeft();
	minRight = sp.MinRight();
	range = sp.GetRange(5);
	bearing = sp.GetBearing(5);
	points = sp.GetCount();
	middleScanLine = sp.GetRange(180);
	leftRange = sp.GetRange(360);  // left most range
	rightRange = sp.GetRange(0);    // right most range

	//Print out useful laser data
	//std::cout << "Laser says..." << std::endl;
	//std::cout << "Maximum distance I can see: " << maxRange << std::endl;
	//std::cout << "Number of readings I return: " << points << std::endl;
	//std::cout << "Closest thing on left: " << minLeft << std::endl;
	//std::cout << "Closest thing on right: " << minRight << std::endl;
	//std::cout << "Range of a middle scan line: " << middleScanLine << std::endl;
	//std::cout << "Range of a single point: " << range << std::endl;
	//std::cout << "Bearing of a single point: " << bearing << std::endl;
	//std::cout << "Range of Left Most Point: " << leftRange << std::endl;
	//std::cout << "Range of Right Most Point: " << rightRange << std::endl;


	return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

	// Print out what the bumpers tell us:
	std::cout << "Left  bumper: " << bp[0] << " Right bumper: " << bp[1] << std::endl;

	// Print out where we are
	std::cout << "We are at" << std::endl;
	std::cout << "Absolute X: " << pose.px << "| Trunc X: " << truncate(pose.px) << std::endl;
	std::cout << "Absolute Y: " << pose.py << "| Trunc Y: " << truncate(pose.py) << std::endl;
	std::cout << "A: " << pose.pa << std::endl;


} // End of printRobotData()


//robot moves towards the destination of waypoint 
bool navigate(BumperProxy& bp, LaserProxy& sp, double x, double y) {


	// We use laser to avoid obstacles
	/* 
	if (counter > 2 && sp.GetRange(180) < .3) {
		turnrate = dtor(40);
		speed = 0;
	} else
	if (counter > 2 && sp.MinRight() < .3) {
		turnrate = dtor(40);
		speed = 0;
	} else
	if (counter > 2 && sp.MinLeft() < .3) {
		turnrate = dtor(-40);
		//speed = -.5;
	}
	*/


	// If the previous x is the same as the current x
	if (y == 1.5) {
		turnrate = dtor(5);
		speed = 0;
		if (pose.pa > 1.55) {
			turnrate = 0;
			speed = 0.1 + 0.9 * distance/distanceBetweenWayPoints;
		}
		//if robot is facing the target positon, go towards it
		if ((pose.px > x - .2 && pose.px < x + .2) && (pose.py > y - .2 && pose.py < y + .2)) {
			turnrate = 0;
			speed = 0;

			std::cout << "=========================================================" << std::endl;
			std::cout << "Success Navigation!" << std::endl;
			std::cout << "=========================================================" << std::endl;
			pose.px = x;
			pose.py = y;
		}
	}


	//turn robot counter-clockwise if the robot's current angle is less than the waypoint 
//getTan 
	else {
		distance = getDistance(pose.px, pose.py, x, y);
		targetTan = getTan(pose.px, pose.py, x, y);
		if (pose.pa < targetTan) {
			double degrees =  1 + 30 * (1- pose.pa/(targetTan+.1));
			turnrate = dtor(15);
			speed = 0;

		}
		//turn robot clockwise if the robot's current angle is more than the waypoint
	//getTan 
		if (pose.pa > targetTan) {
			//double degrees =  1 + 30 * (pose.pa/(targetTan+.1));
			turnrate = dtor(-15);
			speed = 0;

		}
		//if robot is facing the target position, go towards it
		else if (pose.pa > (getTan(pose.px, pose.py, x, y) - .3) && pose.pa < (getTan(pose.px, pose.py, x, y) + .3)) {
			speed = 0.1 + 0.9 * distance/distanceBetweenWayPoints;
			turnrate = 0;
			//robot comes to a stop around waypoint
			if (x == -2.5 && y == -6) {
				if (pose.px == x && pose.py == y) {
					turnrate = 0;
					speed = 0;
					std::cout << "=========================================================" << std::endl;
					std::cout << "Success Navigation!" << std::endl;
					std::cout << "=========================================================" << std::endl;
				}
			}
			else {
				if ((pose.px > x - .2 && pose.px < x + .2) && (pose.py > y - .2 && pose.py < y + .2)) {
					turnrate = 0;
					speed = 0;
					std::cout << "=========================================================" << std::endl;
					std::cout << "Success Navigation!" << std::endl;
					std::cout << "=========================================================" << std::endl;
					pose.px = x;
					pose.py = y;
				}

			}
		}

	}



} // end of navigate


/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
	int length;

	std::ifstream planFile;
	planFile.open("plan.txt");

	planFile >> length;
	planFile.close();

	// Some minimal error checking
	if ((length % 2) != 0) {
		std::cout << "The plan has mismatched x and y coordinates" << std::endl;
		exit(1);
	}

	return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double* plan, int length)
{
	int skip;

	std::ifstream planFile;
	planFile.open("plan.txt");

	planFile >> skip;
	for (int i = 0; i < length; i++) {
		planFile >> plan[i];
		myqueue.push(plan[i]); // Here we fill our queue
	}

	planFile.close();

} // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double* plan, int length)
{
	std::cout << std::endl;
	std::cout << "   x     y" << std::endl;
	for (int i = 0; i < length; i++) {
		std::cout.width(5);
		std::cout << plan[i] << " ";
		if ((i > 0) && ((i % 2) != 0)) {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 *
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/

void writePlan(double* plan, int length)
{
	std::ofstream planFile;
	planFile.open("plan-out.txt");

	planFile << length << " ";
	for (int i = 0; i < length; i++) {
		planFile << plan[i] << " ";
	}

	planFile.close();

} // End of writePlan

/**
* getNextWaypoint
*
* Set double x and double y to our next waypoint while popping them off the queue.
* Does nothing if queue is empty.
* Should only be called once we cleared our current waypoint OR at the start of the program before main loop.
**/

void getNextWaypoint()
{
	if (!myqueue.empty())
	{
		// Save previous point in order to get distance
		// If previous way point does not exists (we will know this if distance
		// between way points not calculated before), set value to initial robot
		// coordinate (-6, -6)
		double previousX = distanceBetweenWayPoints ? x : -6;
		double previousY = distanceBetweenWayPoints ? y : -6; 

		x = myqueue.front();
		myqueue.pop();
		y = myqueue.front();
		myqueue.pop();
		std::cout << "SETTING: X: " << x << " Y: " << y << std::endl;
		
		distanceBetweenWayPoints = getDistance (previousX, previousY, x, y);
	}
	else {
		std::cout << "QUEUE IS EMPTY" << std::endl;
	}
}

/**
* truncate
*
* takes double and returns it truncated to some decimal places.
**/
double truncate(double num)
{
	return (int)(num * 10) / 10.0;
}


/**
* getTan
*
* returns angle between current (x,y) coordinate and target (x,y) coordinate
**/
double getTan(double xPos, double yPos, double xTarget, double yTarget) {
	return tan((yTarget - yPos) / (xTarget - xPos));
}

/**
* getDistance
*
* returns the distance between current (x,y) coordinate and target (x,y) coordinate
**/
float getDistance(double xPos, double yPos, double xTarget, double yTarget) {
	return sqrt(pow(xTarget - xPos, 2) +
		pow(yTarget - yPos, 2));
}

#include <iostream>
#include <cmath>
#define M_PI 3.14159265358979323846

using namespace std;

// --------Important Variables--------

// -User inputed conditions-
double fuel; // Gallons of fuel (3.7kg per gal)
double weight; // Weight of the ship in kg (not factoring in fuel)
double wind; // Speed of wind (mph)
double windAngle; // Angle of the wind (0.0, N; 90.0, E; 180.0, S; 270.0, W)
double goalAlt; // Goal altitude to reach in m

// -Rocket controls-

// Gimble - Moves at 2.0 degrees per tick, max 35.0 degrees either direction
double gimbleX; // Angle of the gimble on the X plane
double gimbleY; // Angle of the gimble on the Y plane
double maxDelta = 2.0; // Max rate of change of the gimble

double enginePower; // Percentage of engine power being used (0.2 gal/tick and 2 m/tick^s at max power)
double thrust;
double tiltFactor; // How much the tilt of the gimble affects the upward lift as a whole

// -Positional Values-
double alt; // Altitude of the ship
double angleX; // X angle of the ship
double angleY; // Y angle of the ship
double angleVelX; // X angle velocity of the ship
double angleVelY; // Y angle velocity of the ship
double angleAccelX; // X angle acceloration of the ship
double angleAccelY; // Y angle acceloration of the ship
double vertVel; // Vertical velocity of the ship
double vertAccel; // Vertical acceleration of the ship

// -Misc Values-
double grav = 9.8; // Gravity
double xWind; // How much the x angle is affected by wind
double yWind; // How much the y angle is affected by wind

// -PID Values-
double targetAngleX = 0.0; // For gimble PID control
double targetAngleY = 0.0; // For gimble PID control
double kp = 1; // Proportion for PID
double ki = 0.1; // Integral for PID
double kd = 0.5; // Derivatve for PID
double integralX = 0.0;
double previousErrorX = 0.0;
double integralY = 0.0;
double previousErrorY = 0.0;

/**
 * hasFailed - Checks to make sure that no fail condition has been met
 * If fuel is <= 0 or vertical velocity is less than 0, then the simulation has failed.
 */
bool hasFailed() {return (fuel <= 0 || vertVel < 0 || angleX > 90 || angleX < -90 || angleY > 90 || angleY < -90);}

int main() {
	
	// -------INITIALIZATION-------

	cout << "Initialization procedure start: " << endl << endl
	 	 << "Setting engine initial conditions..." << endl;
	gimbleX = 0.0;
	gimbleY = 0.0;
	enginePower = 0.0;
	cout << "Setting positional values..." << endl;
	alt = 0.0;
	angleX = 0.0;
	angleY = 0.0;
	angleVelX = 0.0;
	angleVelY = 0.0;
	vertVel = 0.0;
	vertAccel = 0.0;
	cout << "Setting other values..." << endl;
	cout << "Setting user inputed values..." << endl
		 << "PLEASE ENTER HOW MANY GALLONS OF FUEL YOU WOULD LIKE TO USE." << endl
	  	 << "EACH GALLON IS 3.7kg: ";
	cin >> fuel;
	cout << endl;
	cout << "PLEASE ENTER THE MASS OF THE SHIP (IF UNSURE, USE _): ";
	cin >> weight;
	cout << endl;
	cout << "PLEASE ENTER THE WIND SPEED (MPH): ";
	cin >> wind;
	cout << endl;
	cout << "PLEASE ENTER THE WIND DIRECTION (0.0 = N, 90.0 = E...): ";
	cin >> windAngle;
	cout << endl;
	cout << "PLEASE ENTER THE GOAL ALTITUDE (m): ";
	cin >> goalAlt;
	cout << endl;
	cout << "Setting wind and angle variables...";

	double windRad = windAngle * M_PI / 180.0;
	xWind = (wind / 10.0) * cos(windRad); // East-West
	yWind = (wind / 10.0) * sin(windRad); // North-South

	cout << endl << "Initialization complete." << endl << endl;
	
	cout << "Beginning simulation..." << endl;



	enginePower = 1; // Set power to 100 for liftoff


	
	// -------MAIN SIMULATION LOOP-------

	int ticks = 0;
	cout << "Stats at " << (ticks / 10) << " seconds:" << endl;
	cout << "ALT: " << alt << " | ANGLE X: " << angleX << " | ANGLE Y: " << angleY << endl;
	cout << "GIMBLE X: " << gimbleX << " | GIMBLE Y: " << gimbleY << endl;
	cout << "FUEL: " << fuel << " | VERTICAL VELOCITY: " << vertVel << endl << endl;
	while (!hasFailed() && (alt < goalAlt)) {
		
		// !-UPDATE GIMBLE-!
		
		/*
		 * PRIMATIVE GIMBLE UPDATE (Fuel inefficient)
		 * 
		if (angleX > 1) {gimbleX -= 2;}
		else if (angleX < -1) {gimbleX += 2;}
		else {gimbleX = 0;}

		if (angleY > 1) {gimbleY -= 2;}
		else if (angleY < -1) {gimbleY += 2;}
		else {gimbleY = 0;}
		*/

		// Compute error
		double errorX = targetAngleX - angleX;
		double errorY = targetAngleY - angleY;

		// Integrate error
		integralX += errorX / 10;
		integralY += errorY / 10;

		// Derive rate of change
		double derivativeX = (errorX - previousErrorX) * 10;
		double derivativeY = (errorY - previousErrorY) * 10;

		// PID output
		double outputX = (kp * errorX) + (ki * integralX) + (kd * derivativeX);
		double outputY = (kp * errorY) + (ki * integralY) + (kd * derivativeY);

		double deltaX = outputX - gimbleX;
		if (deltaX > maxDelta) { deltaX = maxDelta; }
		else if (deltaX < -maxDelta) { deltaX = -maxDelta; }

		double deltaY = outputY - gimbleY;
		if (deltaY > maxDelta) { deltaY = maxDelta; }
		else if (deltaY < -maxDelta) { deltaY = -maxDelta; }

		// Apply changes
		gimbleX += deltaX;
		gimbleY += deltaY;

		// Ensure gimble does not exceed its max values
		if (gimbleX > 35.0) { gimbleX = 35.0; }
		if (gimbleX < -35.0) { gimbleX = -35.0; }
		if (gimbleY > 35.0) { gimbleY = 35.0; }
		if (gimbleY < -35.0) { gimbleY = -35.0; }

		// Save previous error
		previousErrorX = errorX;
		previousErrorY = errorY;
		
		// !-UPDATE MOVEMENTS-!
		
		// Angle movement
		angleAccelX = xWind + ((enginePower * 2) * (gimbleX / 90));
		angleAccelY = yWind + ((enginePower * 2) * (gimbleY / 90));

		// Velocity movement
		thrust = enginePower * 2;
		tiltFactor = cos(gimbleX * M_PI / 180.0) * cos(gimbleY * M_PI / 180.0);
		vertAccel = (thrust * tiltFactor) - (grav / 10);

		// Apply accelerations
		vertVel += vertAccel;
		angleVelX += angleAccelX;
		angleVelY += angleAccelY;

		// Change positions
		alt += vertVel;
		angleX += angleVelX;
		angleY += angleVelY;

		// Update fuel
		fuel -= (0.2 * enginePower);

		// !-LOG PROGRESS (Every 10 ticks)-!
		ticks++;
		if (ticks % 10 == 0) {
			cout << "Stats at " << (ticks / 10) << " seconds:" << endl;
			cout << "ALT: " << alt << " | ANGLE X: " << angleX << " | ANGLE Y: " << angleY << endl;
			cout << "GIMBLE X: " << gimbleX << " | GIMBLE Y: " << gimbleY << endl;
			cout << "FUEL: " << fuel << " | VERTICAL VELOCITY: " << vertVel << endl << endl;
		}

	}

	// -------POST SIMULATION-------
	
	cout << "End of simulation." << endl << endl;

	// Check reason for loop break
	if (hasFailed()) {
		cout << "!- SIMULATION FAILED -!" << endl << "Failure reason: ";
		if (fuel <= 0) {
			cout << "Out of Fuel!" << endl;
		}
		else if (vertVel < 0) {
			cout << "Ship stopped moving upwards!" << endl;
		}
		else {
			cout << "Ship turned upside-down!" << endl;
		}
	}
	else {
		cout << "!- SIMULATION SUCCEEDED -!" << endl;
	}
	
	cout << "Ending conditions:" << endl
		 << "ALT: " << alt << " | FUEL: " << fuel << " | VERTICAL VELOCITY: " << vertVel << endl 
		 << "ANGLE X: " << angleX << " | ANGLE Y: " << angleY << endl << endl;

	return 0;
}

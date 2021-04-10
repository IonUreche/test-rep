#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define PI 3.14159265

using namespace std;

int main()
{
	bool firstCycle = true;
	bool initStartingPoint = false;
	bool usedBoost = false;

	int maxDist = -300000; // distance to the next checkpoint
	int startX = -1, startY = -1;
	int lastX = -1, lastY = -1;
	int optimX, optimY;

	const int thresholdDistForFirstBoost = 1000;

	int x;
	int y;
	int nextCheckpointX; // x position of the next check point
	int nextCheckpointY; // y position of the next check point
	int nextCheckpointDist; // distance to the next checkpoint
	int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
	int opponentX;
	int opponentY;

	int thrust;
	bool boost = false;
	double angleFactor;

	// game loop
	while (1) {

		cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
		cin >> opponentX >> opponentY; cin.ignore();

		// determine the first time we head towards a checkpoints we already visited
		if (lastX != nextCheckpointX && lastY != nextCheckpointY
			&& startX == nextCheckpointX && startY == nextCheckpointY)
		{
			// we finished full cycle
			firstCycle = false;
		}
		lastX = nextCheckpointX;
		lastY = nextCheckpointY;

		// initializing startX so we can determine later that the cycle of checkpoints has finished
		if (!initStartingPoint)
		{
			startX = nextCheckpointX;
			startY = nextCheckpointY;
			initStartingPoint = true;
		}

		// during the first cycle also determine the longest distance between two checkpoints
		// and save their position so that we know when it's optimal to use BOOST
		if (nextCheckpointDist > maxDist)
		{
			maxDist = nextCheckpointDist;
			optimX = nextCheckpointX;
			optimY = nextCheckpointY;
		}

		// Write an action using cout. DON'T FORGET THE "<< endl"
		// To debug: cerr << "Debug messages..." << endl;

		double cosine = cos(nextCheckpointAngle * PI / 180.0);
		// slow down before reaching a checkpoint in order to avoid big curves
		if (nextCheckpointDist < 1000)
		{
			angleFactor = 0.5;
		}
		else
			if (nextCheckpointAngle <= 90 && nextCheckpointAngle >= -90)
			{
				angleFactor = 1.0;
			}
			else
			{
				angleFactor = 0.0;
			}

		// try to use boost
		// we want to boost when the angle between pod orientation and the next checkpoint is as small as possible
		double thresholdAngleFactor = 0.95; // plus epsilon just in case
		cerr << usedBoost << " " << firstCycle << " " << optimX << " " << optimY << " " << cosine << " " << nextCheckpointDist << " " << angleFactor << endl;
		if (!usedBoost && !firstCycle && optimX == nextCheckpointX && optimY == nextCheckpointY && cosine >= thresholdAngleFactor)
		{
			boost = true;
			usedBoost = true;
		}
		else
		{
			thrust = static_cast<int>(100 * angleFactor);
		}

		// You have to output the target position
		// followed by the power (0 <= thrust <= 100)
		// i.e.: "x y thrust"
		cout << nextCheckpointX << " " << nextCheckpointY << " ";
		if (boost)
		{
			cout << "BOOST" << endl;
			boost = false;
		}
		else
		{
			cout << thrust << endl;
		}
	}
}
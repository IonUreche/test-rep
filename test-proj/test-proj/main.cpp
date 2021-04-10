#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define PI 3.14159265

using namespace std;

// ====== heplers ============
double clamp(double x)
{
	if (x > 1.0) return 1.0;
	if (x < 0.0) return 0.0;
	return x;
}

// Helper class used to keep track of the checkpoint sequence
// so we can do clever tricks with information gathered after the first Lap / cycle.
class CheckpointManager
{
public:
	CheckpointManager()
	{
		m_cycleFinished = false;
		m_initStartingPoint = false;
		m_previousCheckpointX = -1;
		m_previousCheckpointY = -1;
		m_startCheckpointX = -1;
		m_startCheckpointY = -1;
		m_maxDist = -300000; // init max with a very small number so we guarantee to update on first check
		m_currentCheckPointIndex = 0;
	}

	void UpdateCheckpoints(int nextX, int nextY, int nextDist)
	{
		bool newCheckPoint = m_previousCheckpointX != nextX && m_previousCheckpointY != nextY;
		if (newCheckPoint)
		{
			++m_currentCheckPointIndex;
			if (!m_cycleFinished)
			{
				m_posX.push_back(nextX);
				m_posY.push_back(nextY);
			}
		}

		m_previousCheckpointX = nextX;
		m_previousCheckpointY = nextY;

		if (m_cycleFinished) return;

		// determine the first time we head towards a checkpoints we already visited
		if (newCheckPoint && m_startCheckpointX == nextX && m_startCheckpointY == nextY)
		{
			// we finished full cycle
			m_cycleFinished = true;
			ComputeWindingAngles();
		}

		// initializing startX so we can determine later that the cycle of checkpoints has finished
		if (!m_initStartingPoint)
		{
			m_startCheckpointX = nextX;
			m_startCheckpointY = nextY;
			m_initStartingPoint = true;
		}

		// during the first cycle also determine the longest distance between two checkpoints
		// and save their position so that we know when it's optimal to use BOOST
		if (nextDist > m_maxDist)
		{
			m_maxDist = nextDist;
			m_optimCheckpointX = nextX;
			m_optimCheckpointY = nextY;
		}
	}

	// computing winding angles between previous, current and next checkpoint
	// the idea is to have a linear slowdown factor with starting distance as a function of winding angle
	// For example: if the next checkpoint and and the one after are on the same line as your current moving direction, you dont want to slow down at all
	// as oposed to the case when the winding angle is big and if you dont slow down early, the moving trajectory will be a long curve
	void ComputeWindingAngles()
	{
	}

	bool IsOptimalToUseBoost(int nextX, int nextY)
	{
		return m_cycleFinished && m_optimCheckpointX == nextX && m_optimCheckpointY == nextY;
	}

	bool m_cycleFinished;
	bool m_initStartingPoint;
	int m_previousCheckpointX;
	int m_previousCheckpointY;
	int m_startCheckpointX;
	int m_startCheckpointY;
	int m_optimCheckpointX; // best next checkpoint to use boost towards
	int m_optimCheckpointY;
	int m_maxDist;

	int m_currentCheckPointIndex;
	std::vector<int> m_angles; // stores the initial angle 
	std::vector<int> m_posX; // stores the initial angle 
	std::vector<int> m_posY; // stores the initial angle 
};

int main()
{
	const double k_CheckpointRadius = 600;

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

	CheckpointManager ckManager;

	// game loop
	while (1) {

		cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
		cin >> opponentX >> opponentY; cin.ignore();

		ckManager.UpdateCheckpoints(nextCheckpointX, nextCheckpointY, nextCheckpointDist);

		// Write an action using cout. DON'T FORGET THE "<< endl"
		// To debug: cerr << "Debug messages..." << endl;

		double cosine = cos(nextCheckpointAngle * PI / 180.0);
		// slow down before reaching a checkpoint in order to avoid big curves

		double slowDownStartDistance = 1.5 * k_CheckpointRadius; // twice the radius of the checkpoint should be okay to start slowing down
		double slowDownFactor = clamp(nextCheckpointDist / slowDownStartDistance);

		if (nextCheckpointAngle <= 30 && nextCheckpointAngle >= -30)
		{
			angleFactor = 1.0;
		}
		else
			if (nextCheckpointAngle <= 90 && nextCheckpointAngle >= -90)
			{
				angleFactor = 1.0 - ((abs(nextCheckpointAngle) - 30.0) / 60.0);
			}
			else
			{
				angleFactor = 0.0;
			}

		angleFactor *= slowDownFactor;

		// try to use boost
		// we want to boost when the angle between pod orientation and the next checkpoint is as small as possible
		double thresholdAngleFactor = 0.95; // plus epsilon just in case
		cerr << usedBoost << " " << ckManager.m_cycleFinished << " " << ckManager.m_optimCheckpointX << " " << ckManager.m_optimCheckpointY << " " << cosine << " " << nextCheckpointDist << " " << angleFactor << endl;
		if (!usedBoost && cosine >= thresholdAngleFactor && ckManager.IsOptimalToUseBoost(nextCheckpointX, nextCheckpointY))
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
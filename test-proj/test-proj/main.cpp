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

double vec_length(double x, double y)
{
	return std::sqrt(x * x + y * y);
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
		m_arraySize = 0;
	}

	void UpdateCheckpoints(int nextX, int nextY, int nextDist)
	{
		bool newCheckPoint = m_previousCheckpointX != nextX && m_previousCheckpointY != nextY;
		if (newCheckPoint)
		{
			++m_currentCheckPointIndex;
		}

		m_previousCheckpointX = nextX;
		m_previousCheckpointY = nextY;

		if (m_cycleFinished) return;

		// determine the first time we head towards a checkpoints we already visited
		if (newCheckPoint)
		{
			if (m_startCheckpointX == nextX && m_startCheckpointY == nextY)
			{
				// we finished full cycle
				m_cycleFinished = true;
				ComputeWindingAngles();
			}

			if (!m_cycleFinished)
			{
				m_posX.push_back(nextX);
				m_posY.push_back(nextY);
			}
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

	// computing cosine of the angle between vectors forme by (previous, current) and (current, next) checkpoints
	// the idea is to have a linear slowdown factor with starting distance as a function angle cosine
	// For example: if the next checkpoint and and the one after are on the same line as your current moving direction, you dont want to slow down at all
	// as oposed to the case when the winding angle is big and if you dont slow down early, the moving trajectory will be a long curve
	void ComputeWindingAngles()
	{
		m_arraySize = m_posX.size();
		m_angles.resize(m_arraySize);
		cerr << m_arraySize << endl;
		for (int i = 0; i < m_arraySize; ++i)
		{
			int j = (i + 1) % m_arraySize;
			int k = (i + 2) % m_arraySize;

			double ab_x = m_posX[j] - m_posX[i]; // vector AB
			double ab_y = m_posY[j] - m_posY[i];

			double bc_x = m_posX[k] - m_posX[j]; // vector BC
			double bc_y = m_posY[k] - m_posY[j];

			double ab_len = vec_length(ab_x, ab_y);
			double bc_len = vec_length(bc_x, bc_y);
			double cos = (ab_x * bc_x + ab_y * bc_y);
			cos /= ab_len;
			cos /= bc_len;
			m_angles[i] = abs(cos);

			//cerr << ab_x << " " << ab_y << " " << bc_x << " " << bc_y << " " << ab_len << " " << bc_len << " " << cos << " " << m_angles[i] << endl;
		}

	}

	double GetSlowDownFactor()
	{
		if (!m_cycleFinished) return 2.0;
		int idx = (m_currentCheckPointIndex + m_arraySize - 1) % m_arraySize;
		return 0.1 + 2 * m_angles[idx];
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
	int m_arraySize;

	int m_currentCheckPointIndex;
	std::vector<double> m_angles; // stores the initial angle 
	std::vector<int> m_posX; // stores the initial angle 
	std::vector<int> m_posY; // stores the initial angle 
};

int main()
{
	const double k_CheckpointRadius = 600;
	bool usedBoost = false;
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

	int prevX = -1, prevY = -1;

	CheckpointManager ckManager;

	// game loop
	while (1) {

		cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
		cin >> opponentX >> opponentY; cin.ignore();

		if (prevX < 0)
		{
			prevX = x;
			prevY = y;
		}

		ckManager.UpdateCheckpoints(nextCheckpointX, nextCheckpointY, nextCheckpointDist);

		// Write an action using cout. DON'T FORGET THE "<< endl"
		// To debug: cerr << "Debug messages..." << endl;

		double cosine = cos(nextCheckpointAngle * PI / 180.0);
		// slow down before reaching a checkpoint in order to avoid big curves

		double slowDownStartDistance = (ckManager.GetSlowDownFactor()) * k_CheckpointRadius; // twice the radius of the checkpoint should be okay to start slowing down
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
		double angle = acos(ckManager.GetSlowDownFactor()) * 180.0 / PI;
		//cerr << angle << " " << slowDownStartDistance << " " << ckManager.GetSlowDownFactor() << " " << usedBoost << " " << ckManager.m_cycleFinished << " " << ckManager.m_optimCheckpointX << " " << ckManager.m_optimCheckpointY << " " << cosine  << " " << nextCheckpointDist << " " << angleFactor << endl;
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

		// in order to try to avoid curve drifting, we can send modified NextCheckPoint coordinates
		// we want it to be speed dependent, because on high speed we will drift anyway, while on slow speed we dont want to change coordinate too much

		// speed is the derivate of the position so we can take the delta pos between current and last frame
		double deltaSpeedX = x - prevX;
		double deltaSpeedY = y - prevY;
		double distScale = 4.0; // empiric value, seems to work fine at 4.0

		prevX = x;
		prevY = y;

		cout << nextCheckpointX - deltaSpeedX * distScale << " " << nextCheckpointY - deltaSpeedY * distScale << " ";
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
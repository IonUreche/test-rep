#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define PI 3.14159265

using namespace std;

// ====== constants ============
const int k_nrPods = 4;
const int k_maxCheckpoints = 8;
const double k_CheckpointRadius = 600;
const int k_PodRadius = 400;
const double k_frictionfactor = 0.85;
const int k_maxDepth = 14;
const int k_maxNumberOfNodes = 400000;

// ====== helpers ============
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

int g_nrNodes = 0;

struct Node
{
	Node()
	{
		parentIdx = -1;
		nodeIdx = g_nrNodes++;
		firstChildIdx = lastChildIdx = -1;
		nextSiblingIdx = -1;
		score = 0;
		nrTimesVisited = 0;
		thrust = 0;
		angle = 0;
		shieldTurn = 0;
		usedBoost = false;
	}

	Node(Node& parent)
	{
		nodeIdx = g_nrNodes++;
		parentIdx = parent.parentIdx;
		firstChildIdx = -1;
		nextSiblingIdx = parent.lastChildIdx;
		parent.lastChildIdx = nodeIdx;
		if (parent.firstChildIdx == -1)
		{
			parent.firstChildIdx = nodeIdx;
		}
		score = 0;
		nrTimesVisited = 0;
		thrust = 0;
		angle = 0;
		shieldTurn = 0;
		usedBoost = false;
	}

	// tree specific vars
	int parentIdx;
	int nodeIdx;
	int firstChildIdx;
	int lastChildIdx;
	int nextSiblingIdx;

	// scoring vars
	double score;
	int nrTimesVisited;

	// simulation vars
	int thrust;
	int angle;
	int shieldTurn;
	bool usedBoost;
};

//struct Move
//{
//	int angle;
//};

double g_angles[k_maxCheckpoints]; // stores the initial angle 
int g_checkPointPosX[k_maxCheckpoints];
int g_checkPointPosY[k_maxCheckpoints];
double g_normDirVecX[k_maxCheckpoints]; // stores the normalized direction vector between i'th and i+1'th checkpoints
double g_normDirVecY[k_maxCheckpoints];

int g_optimalCheckpointId = -1;
int g_nrLaps = 0;
int g_nrCheckpoints = 0;
int g_turn = -1;
int g_currentLap = 0;

int SquaredIntDistance(int x0, int y0, int x1, int y1)
{
	int dx = (x0 - x1);
	int dy = (y0 - y1);
	return dx * dx + dy * dy;
}

double vec_dot(int x0, int y0, int x1, int y1)
{
	double l1 = sqrt(x0 * x0 + y0 * y0);
	double l2 = sqrt(x1 * x1 + y1 * y1);
	double cos = (x0 * x1 + y0 * y1);
	cos /= l1;
	cos /= l2;
	return abs(cos);
}

void ComputeOptimalBoostCheckpointId()
{
	g_optimalCheckpointId = 0;
	int maxDist = SquaredIntDistance(g_checkPointPosX[0], g_checkPointPosY[0], g_checkPointPosX[g_nrCheckpoints - 1], g_checkPointPosY[g_nrCheckpoints - 1]);
	for (int i = 0; i < g_nrCheckpoints - 1; ++i)
	{
		int dist = SquaredIntDistance(g_checkPointPosX[i], g_checkPointPosY[i], g_checkPointPosX[i + 1], g_checkPointPosY[i + 1]);
		if (dist > maxDist)
		{
			maxDist = dist;
			g_optimalCheckpointId = i;
		}
	}
}

// computing cosine of the angle between vectors formed by (previous, current) and (current, next) checkpoints
// the idea is to have a linear slowdown factor with starting distance as a function of cos(angle)
// For example: if the next checkpoint and and the one after are on the same line as your current moving direction, you dont want to slow down at all
// as oposed to the case when the winding angle is big and if you dont slow down early, the moving trajectory will be a long curve
void ComputeAngles()
{
	for (int i = 0; i < g_nrCheckpoints; ++i)
	{
		int j = (i + 1) % g_nrCheckpoints;
		int k = (i + 2) % g_nrCheckpoints;

		double ab_x = g_checkPointPosX[j] - g_checkPointPosX[i]; // vector AB
		double ab_y = g_checkPointPosY[j] - g_checkPointPosY[i];

		double bc_x = g_checkPointPosX[k] - g_checkPointPosX[j]; // vector BC
		double bc_y = g_checkPointPosY[k] - g_checkPointPosY[j];

		double ab_len = vec_length(ab_x, ab_y);
		double bc_len = vec_length(bc_x, bc_y);
		double cos = (ab_x * bc_x + ab_y * bc_y);
		cos /= ab_len;
		cos /= bc_len;
		g_angles[i] = abs(cos);

		g_normDirVecX[i] = ab_x / ab_len;
		g_normDirVecY[i] = ab_y / ab_len;

		//cerr << ab_x << " " << ab_y << " " << bc_x << " " << bc_y << " " << ab_len << " " << bc_len << " " << cos << " " << m_angles[i] << endl;
	}
}

struct Pod
{
	void ReadTurnInput(std::istream& in)
	{
		in >> x >> y >> vx >> vy >> angle >> nextCheckPointId;
	}

	void WriteTurnOutput(std::ostream& out)
	{
		// in order to try to avoid curve drifting, we can send modified NextCheckPoint coordinates
		// we want it to be speed dependent, because on high speed we will drift anyway, while on slow speed we dont want to change coordinate too much
		out << GetAdjustedCheckpoint(0) << ' ' << GetAdjustedCheckpoint(1) << ' ';
		if (shieldEnabled && (g_turn - turnShieldWasEnabled) > 3)
		{
			out << "SHIELD" << endl;
			turnShieldWasEnabled = g_turn;
			shieldEnabled = false;
		}
		else
			if (useBoost)
			{
				out << "BOOST" << endl;
				useBoost = false;
				boostUsed = true;
			}
			else
			{
				out << thrust << endl;
			}
	}

	void Update()
	{
		UpdateCurrentLap();
		UpdateAngleFactor();
		UpdateThrust();
		TryUseBoost();
	}

	void UpdateCurrentLap()
	{
		if (previousCheckPointId == g_nrCheckpoints - 1 && nextCheckPointId == 0)
		{
			++currentLap;
		}
		previousCheckPointId = nextCheckPointId;
	}

	void UpdateAngleFactor()
	{
		if (g_turn == 0)
		{
			angleFactor = 1.0;
			return;
		}
		if (angle > 180) angle = angle - 360;
		//cerr << angle << " ";
		int nextChpX = g_checkPointPosX[nextCheckPointId] - x;
		int nextChpY = g_checkPointPosY[nextCheckPointId] - y;
		double dirVecLen = sqrt(nextChpX * nextChpX + nextChpY * nextChpY);
		double normProjX = nextChpX / dirVecLen;
		double arc = acos(normProjX);
		if (nextChpY < 0.0) arc = -arc;
		//cerr << arc << " ";
		nextCheckpointAngle = (arc * 180.0 / PI) - angle;
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

		// slow down before reaching a checkpoint in order to avoid big curves
		// the radius of the checkpoint multiplied by a function cosine of angle between the target and the next checkpoint should be okay to start slowing down
		double slowDownStartDistance = GetSlowDownFactor() * k_CheckpointRadius;
		int nextCheckpointDist = sqrt(SquaredIntDistance(x, y, g_checkPointPosX[nextCheckPointId], g_checkPointPosY[nextCheckPointId]));
		double slowDownFactor = clamp(nextCheckpointDist / slowDownStartDistance);

		angleFactor *= slowDownFactor;
		//cerr << slowDownStartDistance << " __ " << slowDownFactor << '\n';
		//cerr << nextCheckpointAngle << " $ \n";
	}

	void UpdateThrust()
	{
		thrust = 100 * angleFactor;
	}

	void PredictCollision(const Pod pods[k_nrPods])
	{
		// Collision prediction for the next 2 frames
		// also checking if it's optimal to shield a collision based on the dot product between speed vectors of the colliding pods

		for (int i = 2; i < 4; ++i) // traverse only the enemy pods
		{
			// compute the distance between the centers of the two pods
			int x0 = x + (i - 1) * vx * k_frictionfactor;
			int y0 = y + (i - 1) * vy * k_frictionfactor;
			int x1 = pods[i].x + (i - 1) * pods[i].vx * k_frictionfactor;
			int y1 = pods[i].y + (i - 1) * pods[i].vy * k_frictionfactor;
			int dx = x1 - x0;
			int dy = y1 - y0;
			int distSq = dx * dx + dy * dy;
			//cerr << distSq << " ... " << 4 * k_PodRadius * k_PodRadius << endl;
			double cosVal = vec_dot(vx, vy, pods[i].vx, pods[i].vy);
			//cerr << cosVal << ' ';
			if (dx * dx + dy * dy <= 4 * k_PodRadius * k_PodRadius && cosVal > 0.8)
				// if cosine of the velocities are > 0.8 we want to shield to prevent deviation in a wrong direction
			{
				shieldEnabled = true;
			}
		}
		//cerr << " #\n";
	}

	int GetAdjustedCheckpoint(int coord)
	{
		const int distScale = 3;
		if (coord == 0) return g_checkPointPosX[nextCheckPointId] + distScale * (g_normDirVecX[nextCheckPointId] - vx);
		return g_checkPointPosY[nextCheckPointId] + distScale * (g_normDirVecY[nextCheckPointId] - vy);
	}

	double GetSlowDownFactor()
	{
		//cerr << g_angles[nextCheckPointId] << " # ";
		return 0.1 + 2.9 * (1 - g_angles[nextCheckPointId]);
	}

	void TryUseBoost()
	{
		if (!boostUsed && currentLap > 0 && nextCheckPointId == g_optimalCheckpointId
			&& nextCheckpointAngle >= -10 && nextCheckpointAngle <= 10) // we want pod to be aligned in order to boost when moving in almost straight line
		{
			useBoost = true;
		}
	}

	int x, y;
	int vx, vy;
	int angle;
	int nextCheckPointId;
	int nextCheckpointAngle;

	bool shieldEnabled = false;
	bool boostUsed = false;
	bool useBoost = false;
	int thrust;
	int turnShieldWasEnabled = -1;
	double angleFactor;
	int previousCheckPointId = -1;
	int currentLap = 0;
};

int main()
{
	bool usedBoost = false;
	bool boost = false;

	Pod pods[k_nrPods];
	Pod s_pods[k_nrPods];

	cin >> g_nrLaps >> g_nrCheckpoints;
	for (int i = 0; i < g_nrCheckpoints; ++i)
	{
		cin >> g_checkPointPosX[i] >> g_checkPointPosY[i];
	}

	ComputeOptimalBoostCheckpointId();
	ComputeAngles();

	using namespace std::chrono;
	high_resolution_clock::time_point t0, t1, t2;
	duration<double> time0, time1;
	double totalAllowedSimulationTime = 0.06;

	// game loop
	while (1) {
		++g_turn;

		for (int i = 0; i < k_nrPods; ++i)
		{
			pods[i].ReadTurnInput(cin);
		}

		t0 = high_resolution_clock::now();

		do
		{
			t1 = high_resolution_clock::now();

			// simulate

			t2 = high_resolution_clock::now();
			time0 = duration_cast<duration<double>>(t2 - t0);
			time1 = duration_cast<duration<double>>(t2 - t1);
		} while (time0.count() + time1.count() < totalAllowedSimulationTime);
		//cerr << time0.count() << '\n';

		for (int i = 0; i < 2; ++i)
		{
			pods[i].Update();
			pods[i].PredictCollision(pods);
			pods[i].WriteTurnOutput(cout);
		}
	}
}
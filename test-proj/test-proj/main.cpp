#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define PI 3.14159265

using namespace std;

// ====== constants ============
const int k_nrPods = 4;
const int k_maxCheckpoints = 4;
const double k_CheckpointRadius = 600;
const int k_PodRadius = 400;

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

double g_angles[k_maxCheckpoints]; // stores the initial angle 
int g_checkPointPosX[k_maxCheckpoints];
int g_checkPointPosY[k_maxCheckpoints];
double g_normDirVecX[k_maxCheckpoints]; // stores the normalized direction vector between i'th and i+1'th checkpoints
double g_normDirVecY[k_maxCheckpoints];

int g_optimalCheckpointId = -1;
int g_nrLaps = 0;
int g_nrCheckpoints = 0;

int SquaredIntDistance(int x0, int y0, int x1, int y1)
{
	int dx = (x0 - x1);
	int dy = (y0 - y1);
	return dx * dx + dy * dy;
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
		out << g_checkPointPosX[nextCheckPointId] << ' ' << g_checkPointPosY[nextCheckPointId] << ' ';
		//if(enableShield && (turn - turnShieldEnabled) > 3)
		//{
		//    out << "SHIELD" << endl;
		//    turnShieldEnabled = turn;
		//}
		//else
		//if(!boostUsed && boost)
		//{
		//    out << "BOOST" << endl;
		//    boost = false;
		//} 
		//else
		{
			out << thrust << endl;
		}
	}

	void UpdateThrust()
	{
		angleFactor = 1.0;
		thrust = 100 * angleFactor;
	}

	double GetSlowDownFactor()
	{
		//int idx = (m_currentCheckPointIndex + m_arraySize - 1) % m_arraySize;
		return 0.1 + 2 * g_angles[nextCheckPointId];
	}

	bool IsOptimalToUseBoost(int nextX, int nextY)
	{
		return nextCheckPointId == g_optimalCheckpointId;
	}

	int x, y;
	int vx, vy;
	int angle;
	int nextCheckPointId;

	bool shieldEnabled = false;
	bool boostUsed = false;
	int thrust;

	double angleFactor;
};

int main()
{
	bool usedBoost = false;
	const int thresholdDistForFirstBoost = 1000;

	int thrust;
	bool boost = false;
	double angleFactor;

	Pod pods[k_nrPods];

	int turnShieldEnabled = -5;
	int turn = -1;

	//int Nrlaps, nrCkp, c_x, c_y;
	cin >> g_nrLaps >> g_nrCheckpoints;
	//ckManager.InitCheckpoints(Nrlaps, nrCkp);
	for (int i = 0; i < g_nrCheckpoints; ++i)
	{
		cin >> g_checkPointPosX[i] >> g_checkPointPosY[i];
		//ckManager.AddCheckpoint(g_checkPointPosX[i], g_checkPointPosY[i]);
	}

	ComputeOptimalBoostCheckpointId();
	ComputeAngles();

	// game loop
	while (1) {
		++turn;
		//cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
		//cin >> opponentX >> opponentY; cin.ignore();
		for (int i = 0; i < k_nrPods; ++i)
		{
			pods[i].ReadTurnInput(cin);
		}

		//double cosine = cos ( nextCheckpointAngle * PI / 180.0 );
		//// slow down before reaching a checkpoint in order to avoid big curves
		//
		//double slowDownStartDistance = (ckManager.GetSlowDownFactor()) * k_CheckpointRadius; // twice the radius of the checkpoint should be okay to start slowing down
		//double slowDownFactor = clamp(nextCheckpointDist / slowDownStartDistance);

		//if(nextCheckpointAngle <= 30 && nextCheckpointAngle >= -30)
		//{
		//    angleFactor = 1.0;
		//}
		//else
		//if(nextCheckpointAngle <= 90 && nextCheckpointAngle >= -90)
		//{
		//    angleFactor = 1.0 - ((abs(nextCheckpointAngle) - 30.0) / 60.0);
		//}
		//else
		//{
		//    angleFactor = 0.0;
		//}
//
		//angleFactor *= slowDownFactor;

		// try to use boost
		// we want to boost when the angle between pod orientation and the next checkpoint is as small as possible
		//double thresholdAngleFactor = 0.95; // plus epsilon just in case
		//double angle = acos(ckManager.GetSlowDownFactor()) * 180.0 / PI;
		////cerr << angle << " " << slowDownStartDistance << " " << ckManager.GetSlowDownFactor() << " " << usedBoost << " " << ckManager.m_cycleFinished << " " << ckManager.m_optimCheckpointX << " " << ckManager.m_optimCheckpointY << " " << cosine  << " " << nextCheckpointDist << " " << angleFactor << endl;
		//if(!usedBoost && cosine >= thresholdAngleFactor && ckManager.IsOptimalToUseBoost(nextCheckpointX, nextCheckpointY))
		//{
		//    boost = true;
		//    usedBoost = true;
		//}
		//else
		//{
		//    thrust = static_cast<int>(100 * angleFactor);
		//}

		// You have to output the target position
		// followed by the power (0 <= thrust <= 100)
		// i.e.: "x y thrust"

		// in order to try to avoid curve drifting, we can send modified NextCheckPoint coordinates
		// we want it to be speed dependent, because on high speed we will drift anyway, while on slow speed we dont want to change coordinate too much

		// speed is the derivative of the position so we can take the delta pos between current and last frame
		//int deltaSpeedX = x - prevX;
		//int deltaSpeedY = y - prevY;
		//int distScale = 4;
//
		//prevX = x;
		//prevY = y;
//
		//int offX = nextCheckpointX - deltaSpeedX * distScale + distScale * ckManager.GetCurrentNormDirX();
		//int offY = nextCheckpointY - deltaSpeedY * distScale + distScale * ckManager.GetCurrentNormDirY();

		//cerr << nextCheckpointX << " " << nextCheckpointY << " " ;
		//cout << offX << " " << offY << " ";

		// Collision prediction for the next 2 frames
		// for now just naive shielding before collision
		// next upgrade would be to determine when it makes sense to shield and when it's okay to take extra velocity from collision
		//int enemyDeltaSpeedX = opponentX - enemyPrevX;
		//int enemyDeltaSpeedY = opponentY - enemyPrevY;
		//enemyPrevX = opponentX;
		//enemyPrevY = opponentY;
		//bool enableShield = false;
		//for(int i = 0; i < 2; ++i)
		//{
		//    int x0 = x + (i+1) * deltaSpeedX;
		//    int y0 = y + (i+1) * deltaSpeedY;
		//    int x1 = opponentX + (i+1) * enemyDeltaSpeedX;
		//    int y1 = opponentY + (i+1) * enemyDeltaSpeedY;
		//    int dx = x1 - x0;
		//    int dy = y1 - y0;
		//    int distSq = dx * dx + dy * dy;
		//    cerr << distSq << " ... " << 4 * k_PodRadius * k_PodRadius << endl;
		//    if(dx * dx + dy * dy <= 4 * k_PodRadius * k_PodRadius)
		//    {
		//        enableShield = true;
		//    }
		//}

		for (int i = 0; i < 2; ++i)
		{
			pods[i].UpdateThrust();
			pods[i].WriteTurnOutput(cout);
		}

		//if(enableShield && (turn - turnShieldEnabled) > 3)
		//{
		//    cout << "SHIELD" << endl;
		//    turnShieldEnabled = turn;
		//}
		//else
		//if(boost)
		//{
		//    cout << "BOOST" << endl;
		//    boost = false;
		//} 
		//else
		//{
		//    cout << thrust << endl;
		//}
	}
}
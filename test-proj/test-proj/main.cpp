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
const int k_maxDepth = 15;
const int k_maxNumberOfNodes = 400000;

// ====== helpers ============
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
// random number generator
static unsigned long x = 123456789, y = 362436069, z = 521288629;

unsigned long xorshf96(void) {          //period 2^96-1
	unsigned long t;
	x ^= x << 16;
	x ^= x >> 5;
	x ^= x << 1;

	t = x;
	x = y;
	y = z;
	z = t ^ x ^ y;

	return z;
}

int GetRand(int mod)
{
	return xorshf96() % mod;
}

double dist(int x0, int y0, int x1, int y1)
{
	int dx = x0 - x1;
	int dy = y0 - y1;
	return sqrt(dx * dx + dy * dy);
}

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
	// will use this for root nodes
	void InitRoot()
	{
		parentIdx = -1;
		nodeIdx = g_nrNodes++;
		firstChildIdx = lastChildIdx = -1;
		nextSiblingIdx = -1;
		score = 0;
		nrTimesVisited = 0;
		//thrust = 0;
		//angle = 0;
		//shieldTurn = 0;
		//usedBoost = false;
	}

	// will use this for child nodes
	void InitChild(Node& parent)
	{
		nodeIdx = g_nrNodes++;
		parentIdx = parent.nodeIdx;
		firstChildIdx = -1;
		lastChildIdx = -1;
		nextSiblingIdx = parent.lastChildIdx;
		parent.lastChildIdx = nodeIdx;
		if (parent.firstChildIdx == -1)
		{
			parent.firstChildIdx = nodeIdx;
		}
		score = 0;
		nrTimesVisited = 0;
		//thrust = 0;
		//angle = 0;
		//shieldTurn = 0;
		//usedBoost = false;
	}

	void ApplyRandomMove()
	{
		move.thrust = GetRand(101);
		move.useBoost = GetRand(100) > 10;
		// shieldTurn;
		// usedBoost;
	}

	struct Move
	{
		int thrust = 0;
		//int angle;
		//int shieldTurn;
		bool useBoost = false;
	};

	// tree specific vars
	int parentIdx;
	int nodeIdx;
	int firstChildIdx;
	int lastChildIdx;
	int nextSiblingIdx;

	// scoring vars
	double score;
	int nrTimesVisited;
	int bestScoreChildIdx;

	// simulation vars
	Move move;

	double traveledDistToNextCheckPoint = 0.0;
	int nrReachedCheckpoints = 0;
};

Node g_nodes[k_maxNumberOfNodes];

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

	double GetTargetPos(int coord)
	{
		if (coord == 0) return g_checkPointPosX[nextCheckPointId];
		return g_checkPointPosY[nextCheckPointId];
	}

	int GetAdjustedCheckpoint(int coord, int nextId = -1, int velX = -1, int velY = -1)
	{
		if (nextId == -1) nextId = nextCheckPointId;
		if (velX == -1) velX = vx;
		if (velY == -1) velY = vy;
		const int distScale = 3;
		if (coord == 0) return g_checkPointPosX[nextId] + distScale * (g_normDirVecX[nextId] - velX);
		return g_checkPointPosY[nextId] + distScale * (g_normDirVecY[nextId] - velY);
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

	void ApplyMove(Node::Move& move)
	{
		thrust = move.thrust;
		useBoost = (boostUsed) ? false : move.useBoost;
	}

	void ApplyMove(Node& node)
	{
		// simulating pod's movement based on expert rules
		// 1. rotation: the facing vector is the normalized speed vector
		double vlen = vec_length(vx, vy);
		vlen = std::max(1e-3, vlen);
		double fx = vx / vlen;
		double fy = vy / vlen;
		//cerr << "vxy vec " << vx << ' ' << vy << " $ ";
		//cerr << "f vec " << fx << ' ' << fy << " $ ";

		// determine the angle between the target position and facing direction
		double tx = GetTargetPos(0) - x;
		double ty = GetTargetPos(1) - y;
		double vlen2 = vec_length(tx, ty);
		tx /= vlen2; // normalize the target vector
		ty /= vlen2;
		double cosine = vec_dot(fx, fy, tx, ty);
		double alpha = acos(cosine);
		if (ty < 0.0) alpha = -alpha;
		//cerr << arc << " ";
		// clamp to [-18, 18] degrees
		alpha = std::min(18.0, std::max(-18.0, (alpha * 180.0 / PI)));
		angle = (angle + (int)(alpha)+360) % 360; // update the pod's absolute angle
		//cerr << "t vec " << tx << ' ' << ty << ' ' << alpha << " $ ";
		// determine the new facing vector
		double rad_angle = angle * PI / 180.0;
		double fx2 = cos(rad_angle);
		double fy2 = sin(rad_angle);
		//cerr << "new facing: " << fx2 << " " << fy2 << '\n';

		// 2. Acceleration
		vx += fx2 * node.move.thrust;
		vy += fy2 * node.move.thrust;

		//cerr << "acc : " << node.move.thrust << '\n';

		// 3. Movement
		x = (int)std::round(x + vx);
		y = (int)std::round(y + vy);

		//cerr << "vxy before : " << vx << ' ' << vy << '\n';

		//cerr << "vxy after : " << vx << ' ' << vy << '\n';

		// 4. Check if we reached the target checkpoint
		double destX = GetAdjustedCheckpoint(0, nextCheckPointId, vx, vy);
		double destY = GetAdjustedCheckpoint(1, nextCheckPointId, vx, vy);
		double distToCheckPoint = dist(x, y, destX, destY);
		//cerr << "dist to check : " << distToCheckPoint << '\n';
		if (distToCheckPoint < k_CheckpointRadius)
		{
			++nrReachedCheckpoints;
			nextCheckPointId = (nextCheckPointId + 1) % g_nrCheckpoints;
			traveledDistToNextCheckPoint = distToCheckPoint;
		}
		else
		{
			traveledDistToNextCheckPoint += vec_length(vx, vy);
		}

		// 5. friction
		vx = vx * k_frictionfactor;
		vy = vy * k_frictionfactor;

		node.traveledDistToNextCheckPoint = traveledDistToNextCheckPoint;
		node.nrReachedCheckpoints = nrReachedCheckpoints;
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

	int nrReachedCheckpoints = 0;
	double traveledDistToNextCheckPoint = 0.0;
};

Pod g_pods[k_nrPods];
Pod s_pods[k_nrPods];

const int k_childsPerNode = 10;
const int k_maxVisitsWhenExploring = 5;
const int k_scoreForReachingCheckpoint = 18358; // diagonal of the viewport, we want reaching checkpoint to be worth more than just distance traveled

void InitSimulation()
{
	// create all Tree roots
	for (int i = 0; i < k_nrPods; ++i)
	{
		g_nodes[i].InitRoot();
	}
}

void GatherSimulationResults()
{
	// apply best score moves to the friendly pods
	for (int i = 0; i < 2; ++i)
	{
		Node::Move best_move = g_nodes[g_nodes[i].bestScoreChildIdx].move;

		//cerr << "Best " << g_nodes[i].bestScoreChildIdx << ' ' << g_nodes[i].score << ' ' << best_move.thrust << "\n"; 
		g_pods[i].ApplyMove(best_move);
	}
}


void Simulate()
{
	// reset nodes
	for (int i = 0; i < k_nrPods; ++i)
		s_pods[i] = g_pods[i];

	Node* cNode[k_nrPods]; // pointers to current nodes

	for (int i = 0; i < k_nrPods; ++i)
	{
		cNode[i] = &g_nodes[i];
	}

	//cerr << " sim " << g_nrNodes << "\n";
	// explore until maxDepth for each root
	int depth = 0;
	while (depth < k_maxDepth)
	{
		for (int i = 0; i < k_nrPods; ++i)
		{
			cNode[i]->nrTimesVisited++;

			if (cNode[i]->firstChildIdx == -1)
			{
				// current node has no children so we want to allocate some in order to explore the random move possibilities
				for (int k = 0; k < k_childsPerNode; ++k)
				{
					int nodeidx = g_nrNodes;
					g_nodes[nodeidx].InitChild(*cNode[i]);
					g_nodes[nodeidx].ApplyRandomMove();
				}
			}

			// for further exploration in the first steps of the simulation, we'll take random nodes to explore
			// after which we can start selecting them based on score computed in this 'Random phase'
			if (cNode[i]->nrTimesVisited < k_maxVisitsWhenExploring)
			{
				int rnd = xorshf96() % k_childsPerNode;
				int childIdx = cNode[i]->firstChildIdx + rnd;
				cNode[i] = &g_nodes[childIdx];
			}
			else
			{
				// take the best score child, in order to exploit the more promising moves
				int bestIdx = 0;
				int bestScore = -1;
				int firstChildIdx = cNode[i]->firstChildIdx;
				for (int j = 0; j < k_childsPerNode; ++j)
				{
					int idx = firstChildIdx + j;
					if (g_nodes[idx].score > bestScore)
					{
						bestScore = g_nodes[idx].score;
						bestIdx = idx;
					}
				}
				cNode[i] = &g_nodes[bestIdx];
			}
			s_pods[i].ApplyMove(*cNode[i]);
		}
		++depth;
	}

	for (int i = 0; i < k_nrPods; ++i)
	{
		// bottom-up update for scores
		int score = cNode[i]->nrReachedCheckpoints * k_scoreForReachingCheckpoint + cNode[i]->traveledDistToNextCheckPoint;
		//cerr << score << "__";//cNode[i]->nrReachedCheckpoints << " " << cNode[i]->traveledDistToNextCheckPoint << '\n';
		int idx = cNode[i]->nodeIdx;
		//cerr << idx << "__";
		while (cNode[i]->parentIdx >= 0)
		{
			cNode[i] = &g_nodes[cNode[i]->parentIdx];
			if (cNode[i]->score < score)
			{
				cNode[i]->score = score;
				cNode[i]->bestScoreChildIdx = idx;
			}
			idx = cNode[i]->nodeIdx;
			//cerr << cNode[i]->nodeIdx << "_" << cNode[i]->score << '\n';
		}
	}
}

int main()
{
	cin >> g_nrLaps >> g_nrCheckpoints;
	for (int i = 0; i < g_nrCheckpoints; ++i)
	{
		cin >> g_checkPointPosX[i] >> g_checkPointPosY[i];
	}

	//ComputeOptimalBoostCheckpointId();
	//ComputeAngles();

	using namespace std::chrono;
	high_resolution_clock::time_point t0, t1, t2;
	duration<double> time0, time1;
	double totalAllowedSimulationTime = 0.05;

	//for (int i = 0; i < 100; ++i)
	//{
	//	cout << xorshf96() << ' ';
	//}

	// game loop
	while (1) {
		++g_turn;
		g_nrNodes = 0;
		for (int i = 0; i < k_nrPods; ++i)
		{
			g_pods[i].ReadTurnInput(cin);
		}

		// first turn only
		if (g_turn == 1)
		{
			for (int i = 0; i < 2; ++i)
				cout << g_checkPointPosX[0] << ' ' << g_checkPointPosY[0] << ' ' << 100 << '\n';
			continue;
		}

		t0 = high_resolution_clock::now();

		InitSimulation();

		do
		{
			t1 = high_resolution_clock::now();

			// simulate
			Simulate();

			t2 = high_resolution_clock::now();
			time0 = duration_cast<duration<double>>(t2 - t0);
			time1 = duration_cast<duration<double>>(t2 - t1);
		} while (time0.count() + time1.count() < totalAllowedSimulationTime);
		//cerr << time0.count() << '\n';

		GatherSimulationResults();

		for (int i = 0; i < 2; ++i)
		{
			//g_pods[i].Update();
			//pods[i].PredictCollision(pods);
			g_pods[i].WriteTurnOutput(cout);
		}
	}

	return 0;
}


//
//void ComputeOptimalBoostCheckpointId()
//{
//	g_optimalCheckpointId = 0;
//	int maxDist = SquaredIntDistance(g_checkPointPosX[0], g_checkPointPosY[0], g_checkPointPosX[g_nrCheckpoints - 1], g_checkPointPosY[g_nrCheckpoints - 1]);
//	for (int i = 0; i < g_nrCheckpoints - 1; ++i)
//	{
//		int dist = SquaredIntDistance(g_checkPointPosX[i], g_checkPointPosY[i], g_checkPointPosX[i + 1], g_checkPointPosY[i + 1]);
//		if (dist > maxDist)
//		{
//			maxDist = dist;
//			g_optimalCheckpointId = i;
//		}
//	}
//}
//
//// computing cosine of the angle between vectors formed by (previous, current) and (current, next) checkpoints
//// the idea is to have a linear slowdown factor with starting distance as a function of cos(angle)
//// For example: if the next checkpoint and and the one after are on the same line as your current moving direction, you dont want to slow down at all
//// as oposed to the case when the winding angle is big and if you dont slow down early, the moving trajectory will be a long curve
//void ComputeAngles()
//{
//	for (int i = 0; i < g_nrCheckpoints; ++i)
//	{
//		int j = (i + 1) % g_nrCheckpoints;
//		int k = (i + 2) % g_nrCheckpoints;
//
//		double ab_x = g_checkPointPosX[j] - g_checkPointPosX[i]; // vector AB
//		double ab_y = g_checkPointPosY[j] - g_checkPointPosY[i];
//
//		double bc_x = g_checkPointPosX[k] - g_checkPointPosX[j]; // vector BC
//		double bc_y = g_checkPointPosY[k] - g_checkPointPosY[j];
//
//		double ab_len = vec_length(ab_x, ab_y);
//		double bc_len = vec_length(bc_x, bc_y);
//		double cos = (ab_x * bc_x + ab_y * bc_y);
//		cos /= ab_len;
//		cos /= bc_len;
//		g_angles[i] = abs(cos);
//
//		g_normDirVecX[i] = ab_x / ab_len;
//		g_normDirVecY[i] = ab_y / ab_len;
//
//		//cerr << ab_x << " " << ab_y << " " << bc_x << " " << bc_y << " " << ab_len << " " << bc_len << " " << cos << " " << m_angles[i] << endl;
//	}
//}
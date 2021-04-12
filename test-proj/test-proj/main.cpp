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
const int k_maxDepth = 8;
const int k_maxNumberOfNodes = 400000;
const int k_childrenPerNode = 100;
const int k_maxVisitsWhenExploring = 40;
const int k_scoreForReachingCheckpoint = 18358; // diagonal of the viewport, we want reaching checkpoint to be worth more than just distance traveled

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
	return cos;//abs(cos);
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
// ====== random number generator ============
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

// ====== global vars ============
int g_nrNodes = 0;
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

struct Node
{
	// will use this for root nodes
	void InitRoot()
	{
		parentIdx = -1;
		nodeIdx = g_nrNodes++;
		firstChildIdx = -1;
		nrChildren = 0;
		score = -k_scoreForReachingCheckpoint;
		nrTimesVisited = 0;
	}

	// will use this for child nodes
	void InitChild(Node& parent)
	{
		nodeIdx = g_nrNodes++;
		parentIdx = parent.nodeIdx;
		firstChildIdx = -1;
		nrChildren = 0;
		if (parent.firstChildIdx == -1)
		{
			parent.firstChildIdx = nodeIdx;
		}
		score = -k_scoreForReachingCheckpoint;
		nrTimesVisited = 0;
	}

	void ApplyRandomMove(int turn)
	{
		move.thrust = GetRand(101);
		move.useBoost = GetRand(100) < 10;
		move.shieldEnabled = GetRand(100) < 15;
		move.shieldTurn = move.shieldEnabled ? turn : -5;
		move.targetOffsetX = GetRand(600) - 300;
		move.targetOffsetY = GetRand(600) - 300;
	}

	void UpdateScore(int depth)
	{
		score = (nrReachedCheckpoints * k_scoreForReachingCheckpoint - distToNextCheckPoint);
	}

	struct Move
	{
		int thrust = 0;
		//int angle;
		int targetOffsetX = 0;
		int targetOffsetY = 0;
		int shieldTurn = -5;
		int shieldEnabled = false;
		bool useBoost = false;
	};

	// tree specific vars
	int parentIdx;
	int nodeIdx;
	int firstChildIdx;
	int nrChildren;

	// scoring vars
	double score;
	int nrTimesVisited;
	int bestScoreChildIdx;

	// simulation vars
	Move move;

	double distToNextCheckPoint = 0.0;
	int nrReachedCheckpoints = 0;
};

Node g_nodes[k_maxNumberOfNodes];

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
		//out << GetAdjustedCheckpoint(0) << ' ' << GetAdjustedCheckpoint(1) << ' ';
		out << g_checkPointPosX[nextCheckPointId] + destOffsetX << ' ' << g_checkPointPosY[nextCheckPointId] + destOffsetY << ' ';
		if (false && shieldEnabled && (g_turn - turnShieldWasEnabled) > 3)
		{
			out << "SHIELD" << endl;
			turnShieldWasEnabled = g_turn;
			shieldEnabled = false;
		}
		else if (useBoost)
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
		//if (nextCheckpointAngle <= 30 && nextCheckpointAngle >= -30)
		//{
		//	angleFactor = 1.0;
		//}
		//else if (nextCheckpointAngle <= 90 && nextCheckpointAngle >= -90)
		//{
		//    angleFactor = 1.0 - ((abs(nextCheckpointAngle) - 30.0) / 60.0);
		//}
		//else
		//{
		//    angleFactor = 0.0;
		//}

		// slow down before reaching a checkpoint in order to avoid big curves
		// the radius of the checkpoint multiplied by a function cosine of angle between the target and the next checkpoint should be okay to start slowing down
		//double slowDownStartDistance = GetSlowDownFactor() * k_CheckpointRadius;
		//int nextCheckpointDist = sqrt(SquaredIntDistance(x, y, g_checkPointPosX[nextCheckPointId], g_checkPointPosY[nextCheckPointId]));
		//double slowDownFactor = clamp(nextCheckpointDist / slowDownStartDistance);

		//angleFactor *= slowDownFactor;
		//cerr << slowDownStartDistance << " __ " << slowDownFactor << '\n';
		//cerr << nextCheckpointAngle << " $ \n";
	}

	bool WillCollideNextNFrames(int N, const Pod pods[k_nrPods], int idx, int simulationTurn)
	{
		// Collision prediction for the next 2 frames
		// also checking if it's optimal to shield a collision based on the dot product between speed vectors of the colliding pods
		double attenuation_factor = 1.0;
		for (int f = 1; f <= N; ++f)
		{
			for (int i = 0; i < N; ++i) // traverse all pods
			{
				if (i == idx) continue; // skip checking collision with itself

				// compute the distance between the centers of the two pods
				int x0 = x + f * vx * attenuation_factor;
				int y0 = y + f * vy * attenuation_factor;
				int x1 = pods[i].x + f * pods[i].vx * attenuation_factor;
				int y1 = pods[i].y + f * pods[i].vy * attenuation_factor;
				int dx = x1 - x0;
				int dy = y1 - y0;
				int distSq = dx * dx + dy * dy;
				//cerr << distSq << " ... " << 4 * k_PodRadius * k_PodRadius << endl;
				double cosVal = vec_dot(vx, vy, pods[i].vx, pods[i].vy);
				//cerr << cosVal << ' ';
				if (dx * dx + dy * dy <= 4 * k_PodRadius * k_PodRadius/* && cosVal < 0*/)
					// if cosine of the velocities are > 0.8 we want to shield to prevent deviation in a wrong direction
				{
					//shieldEnabled = true;
					//turnShieldWasEnabled = simulationTurn;
					return true;
				}
			}
			attenuation_factor *= k_frictionfactor;
		}
		return false;
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

	void ApplyMove(Node::Move& move)
	{
		thrust = move.thrust;
		useBoost = !boostUsed ? move.useBoost : false;
		shieldEnabled = move.shieldEnabled;
		destOffsetX = move.targetOffsetX;
		destOffsetY = move.targetOffsetY;
		//turnShieldWasEnabled = std::max(turnShieldWasEnabled, move.shieldTurn);
	}

	void ApplyMove(Node& node, int podIdx, const Pod pods[k_nrPods], int simulationTurn)
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
		int actualThrust = useBoost ? 650 : node.move.thrust;
		bool shieldCooldownOn = (simulationTurn - turnShieldWasEnabled) > 3;
		if (!shieldCooldownOn && node.move.shieldEnabled)
		{
			turnShieldWasEnabled = simulationTurn;
		}
		actualThrust = shieldCooldownOn ? actualThrust : 0;

		vx += fx2 * actualThrust;
		vy += fy2 * actualThrust;

		//cerr << "acc : " << node.move.thrust << '\n';

		// 3. Movement
		x = (int)std::round(x + vx);
		y = (int)std::round(y + vy);

		//cerr << "vxy before : " << vx << ' ' << vy << '\n';

		//cerr << "vxy after : " << vx << ' ' << vy << '\n';

		// 4. Check if we reached the target checkpoint
		double destX = g_checkPointPosX[nextCheckPointId] + node.move.targetOffsetX;//GetAdjustedCheckpoint(0, nextCheckPointId, vx, vy);
		double destY = g_checkPointPosY[nextCheckPointId] + node.move.targetOffsetY;//GetAdjustedCheckpoint(1, nextCheckPointId, vx, vy);
		double distToCheckPoint = dist(x, y, destX, destY);
		//cerr << "dist to check : " << distToCheckPoint << '\n';
		if (distToCheckPoint < k_CheckpointRadius)
		{
			++nrReachedCheckpoints;
			nextCheckPointId = (nextCheckPointId + 1) % g_nrCheckpoints;
			distToNextCheckPoint = dist(x, y, g_checkPointPosX[nextCheckPointId], g_checkPointPosY[nextCheckPointId]);//distToCheckPoint;
		}
		else
		{
			distToNextCheckPoint = distToCheckPoint; //vec_length(vx, vy);
		}

		// 5. friction
		vx = vx * k_frictionfactor;
		vy = vy * k_frictionfactor;

		node.distToNextCheckPoint = distToNextCheckPoint;
		node.nrReachedCheckpoints = nrReachedCheckpoints;

		// update node score
		node.UpdateScore(simulationTurn - g_turn);

		//UpdateAngleFactor();
	}

	int x, y;
	int vx, vy;
	int angle;
	int nextCheckPointId;
	int nextCheckpointAngle;

	int destOffsetX;
	int destOffsetY;

	bool shieldEnabled = false;
	bool boostUsed = false;
	bool useBoost = false;
	int thrust;
	int turnShieldWasEnabled = -1;
	double angleFactor;
	int previousCheckPointId = -1;
	int currentLap = 0;

	int nrReachedCheckpoints = 0;
	double distToNextCheckPoint = 0.0;
};

Pod g_pods[k_nrPods];
Pod s_pods[k_nrPods];

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
		//double bestScore = -1;
		//int bestIdx = 0;
		//int nrChildren = g_nodes[i].nrChildren;
		//int firstChildrenIdx = g_nodes[i].firstChildIdx;
		//for(int j = 0; j < nrChildren; ++j)
		//{
		//    if(g_nodes[firstChildrenIdx + j].score > bestScore)
		//    {
		//        bestScore = g_nodes[firstChildrenIdx + j].score;
		//        bestIdx = j;
		//    }
		//}
		Node::Move best_move = g_nodes[g_nodes[i].bestScoreChildIdx].move;
		//Node::Move best_move = g_nodes[bestIdx].move;
		////cerr << "Best " << g_nodes[i].bestScoreChildIdx << ' ' << g_nodes[i].score << ' ' << best_move.thrust << "\n"; 
		//g_pods[i].ApplyMove(best_move);
		cerr << g_nodes[g_nodes[i].bestScoreChildIdx].score << '\n';
		g_pods[i].ApplyMove(best_move);
	}
}

void SimulateCollision(Pod pods[k_nrPods], int simulationTurn)
{
	// PredictCollision and apply elastic collision effect
	bool collided[k_nrPods];
	for (int i = 0; i < k_nrPods; ++i)
	{
		collided[i] = false;
	}

	for (int i = 0; i < k_nrPods / 2; ++i)
	{
		if (collided[i]) continue; // skip 
		for (int j = k_nrPods / 2; j < k_nrPods; ++j)
		{
			if (collided[j]) continue; // skip 

			// compute the distance between the centers of the two pods
			int x0 = pods[i].x + pods[i].vx;
			int y0 = pods[i].y + pods[i].vy;
			int x1 = pods[j].x + pods[j].vx;
			int y1 = pods[j].y + pods[j].vy;
			int dx = x1 - x0;
			int dy = y1 - y0;
			int distSq = dx * dx + dy * dy;
			//cerr << distSq << " ... " << 4 * k_PodRadius * k_PodRadius << endl;
			double cosVal = vec_dot(pods[i].vx, pods[i].vy, pods[j].vx, pods[j].vy);
			//cerr << cosVal << ' ';
			if (dx * dx + dy * dy <= 4 * k_PodRadius * k_PodRadius)
			{
				int m1 = (pods[i].turnShieldWasEnabled - simulationTurn > 3) ? 1 : 10;
				int m2 = (pods[j].turnShieldWasEnabled - simulationTurn > 3) ? 1 : 10;
				//cerr << "cosval: " << cosVal << '\n';
				//if (cosVal < 0)
				//{
				//	shieldEnabled = true;
				//	turnShieldWasEnabled = simulationTurn;
				//}

				double v1x = pods[i].vx - pods[j].vx;
				double v1y = pods[i].vy - pods[j].vy;
				double massFactor = (2.0 * m2 / (m1 + m2));
				double dotFactor1 = vec_dot(v1x, v1y, dx, dy) / distSq;
				double dotFactor2 = vec_dot(-v1x, -v1y, -dx, -dy) / distSq;

				pods[i].vx = pods[i].vx - massFactor * dotFactor1 * dx;
				pods[i].vy = pods[i].vy - massFactor * dotFactor1 * dy;

				pods[j].vx = pods[j].vx - massFactor * dotFactor2 * (-dx);
				pods[j].vy = pods[j].vy - massFactor * dotFactor2 * (-dy);

				collided[i] = collided[j] = true;
			}
		}
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
				int nrChildren = k_childrenPerNode / (depth + 1);
				cNode[i]->nrChildren = nrChildren;
				for (int k = 0; k < nrChildren; ++k)
				{
					int nodeidx = g_nrNodes;
					g_nodes[nodeidx].InitChild(*cNode[i]);
					g_nodes[nodeidx].ApplyRandomMove(g_turn + depth + 1);
				}
			}

			// for further exploration in the first steps of the simulation, we'll take random nodes to explore
			// after which we can start selecting them based on score computed in this 'Random phase'
			if (cNode[i]->nrTimesVisited < k_maxVisitsWhenExploring / (depth + 1))
			{
				int rnd = xorshf96() % cNode[i]->nrChildren;
				int childIdx = cNode[i]->firstChildIdx + rnd;
				cNode[i] = &g_nodes[childIdx];
			}
			else
			{
				// take the best score child, in order to exploit the more promising moves
				int bestIdx = 0;
				double bestScore = -k_scoreForReachingCheckpoint;
				int firstChildIdx = cNode[i]->firstChildIdx;
				for (int j = 0; j < cNode[i]->nrChildren; ++j)
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
			s_pods[i].ApplyMove(*cNode[i], i, s_pods, g_turn + depth + 1);
		}
		SimulateCollision(s_pods, g_turn + depth + 1);
		++depth;
	}

	for (int i = 0; i < k_nrPods; ++i)
	{
		// bottom-up update for scores
		int score = cNode[i]->score;
		//cerr << score << "__";//cNode[i]->nrReachedCheckpoints << " " << cNode[i]->traveledDistToNextCheckPoint << '\n';
		int idx = cNode[i]->nodeIdx;
		//cerr << idx << "__";
		double f = 1.0f + 0.1 * k_maxDepth;
		while (cNode[i]->parentIdx >= 0)
		{
			cNode[i] = &g_nodes[cNode[i]->parentIdx];
			if (cNode[i]->score < score)
			{
				cNode[i]->score = score;
				cNode[i]->bestScoreChildIdx = idx;
			}
			//cNode[i]->score += score;
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

	ComputeOptimalBoostCheckpointId();
	//ComputeAngles();

	using namespace std::chrono;
	high_resolution_clock::time_point t0, t1, t2;
	duration<double> time0, time1;
	double totalAllowedSimulationTime = 0.05;

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
			g_pods[i].WriteTurnOutput(cout);
		}
		cerr << g_nrNodes << '\n';
	}

	return 0;
}
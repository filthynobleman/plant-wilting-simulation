#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Simulation/Simulation.h"
#include <pwd/pwd.hpp>

#define _USE_MATH_DEFINES
#include "math.h"

#include <chrono>


// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void initParameters();
void timeStep ();
void buildModel ();
void createModel();
void render ();
void reset();

void start_timer();
double stop_timer();

double WModelTime = 0.0;
double PBDTime = 0.0;
size_t NFrames = 0;

DemoBase *base;

// bunny rod scene
const int numberOfBodies = 10;
const Real width = static_cast<Real>(1.0);
const Real height = static_cast<Real>(0.1);
const Real depth = static_cast<Real>(0.1);
const Real density = 7800.;

const Real bunnyDensity = 500.;


std::string PlantFile = "/resources/plants/plant000.txt";
pwd::Graph* Graph = nullptr;
pwd::WaterModel* WaterModel = nullptr;
std::vector<int> CNode1;
std::vector<int> CNode2;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Plant demo");

	
	if (argc > 1)
		PlantFile = argv[1];
	else
		PlantFile = FileSystem::normalizePath(base->getExePath() + PlantFile);
	std::cout << PlantFile << std::endl;
	try
	{
		Graph = new pwd::Graph(PlantFile);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.0025));
	int grav_id = Simulation::getCurrent()->GRAVITATION;
	Vector3r gravity(0.0, 0.0, -9.81);
	Simulation::getCurrent()->setVecValue<Real>(grav_id, gravity.data());
	buildModel();

	base->createParameterGUI();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	MiniGL::mainLoop();	

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	
	std::cout << "Plant,Nodes,WaterModel,PBD,Frames" << std::endl;
	std::cout << PlantFile << ',' << Graph->NumNodes() << ',';
	std::cout << WModelTime / NFrames << ',' << PBDTime / NFrames << ',' << NFrames << std::endl;

	delete Simulation::getCurrent();
	delete base;
	delete model;
	delete Graph;


	return 0;
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	buildModel();
}

void timeStep ()
{
	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	// Simulation code
	start_timer();
	SimulationModel *model = Simulation::getCurrent()->getModel();
	auto& rbvec = model->getRigidBodies();
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;

		for (auto& rb : rbvec)
		{
			rb->setVelocity(base->getDampingForce() * rb->getVelocity());
			rb->setAngularVelocity(base->getDampingForce() * rb->getAngularVelocity());
		}

		base->step();
	}
	PBDTime += stop_timer();

	start_timer();
	WaterModel->Evaluate(base->getTime());
	WModelTime += stop_timer();

	start_timer();
	for (int i = 0; i < Graph->NumNodes(); ++i)
	{
		const pwd::Node* N = Graph->GetNode(i);
		Real Mass = N->Volume();
		if (N->IsOnLeaf())
			Mass *= base->getLeafDensity();
		else
			Mass *= base->getDensity();
		Mass += WaterModel->Water(i);
		Mass *= base->getMassScale();
		if (N == Graph->Root())
			rbvec[i]->setMass(0.0);
		else
			rbvec[i]->setMass(Mass);
	}

	// Update Young's modulus on constraints
	auto CVec = model->getConstraints();
	double MaxYoungs = 0.0;
	for (size_t cidx = 0; cidx < CVec.size(); ++cidx)
	{
		Constraint* c = CVec[cidx];
		StretchBendingTwistingConstraint* cc;
		cc = static_cast<StretchBendingTwistingConstraint*>(c);
		if (cc != nullptr)
		{
			int IDN = CNode1[cidx];
			int IDC = CNode2[cidx];
			double yN = WaterModel->Water(IDN);
			double yC = WaterModel->Water(IDC);
			Real youngsModulus = 0.5 * (yN + yC);
			youngsModulus = 1 + std::exp(-base->getYoungsSteep() * (youngsModulus - base->getYoungsMidpoint()));
			youngsModulus = base->getYoungsMin() + base->getYoungsMax() / youngsModulus;
			Real torsionModulus = youngsModulus;
			Real secondMomentOfArea(static_cast<Real>(M_PI_4) * std::pow(cc->m_averageRadius, static_cast<Real>(4.0)));
			Real bendingStiffness(youngsModulus * secondMomentOfArea);
			Real torsionStiffness(static_cast<Real>(2.0) * torsionModulus * secondMomentOfArea);
			cc->m_stiffnessCoefficientK = Vector3r(bendingStiffness, torsionStiffness, bendingStiffness);
		}
	}

	PBDTime += stop_timer();
	NFrames++;
}

void buildModel ()
{
	createModel();
	base->resetWaterModel();
}

void render ()
{
	base->render();
}

void createModel()
{
	if (WaterModel == nullptr)
		WaterModel = new pwd::WaterModel(Graph, base->getLossRate(), base->getInitialWater());
	else
		WaterModel->Initialize(base->getLossRate(), base->getInitialWater());
	WaterModel->Build();

	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cylinder_equisize.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(1, 1, 1));
	mesh.setFlatShading(true);


	SimulationModel* model = Simulation::getCurrent()->getModel();
	model->cleanup();
	const std::vector<pwd::Node*>& Nodes = Graph->GetNodes();

	SimulationModel::RigidBodyVector &rbvec = model->getRigidBodies();
	for (int i = 0; i < Graph->NumNodes(); ++i)
	{
		const pwd::Node *N = Graph->GetNode(i);
		Vector3r Pos = 0.5 * (N->Head() + N->Tail());
		Quaternionr Rot = N->Rotation();
		// std::cout << (*it)->ID() << '\t' << Rot << std::endl;
		Real Len = N->Length();
		Real Rad = N->Radius();
		Vector3r Scale = Vector3r(Rad, Len, Rad);
		rbvec.push_back(new RigidBody());
		rbvec[rbvec.size() - 1]->initBody(1, Pos, Rot, vd, mesh, Scale);
		if (N == Graph->Root())
			rbvec[rbvec.size() - 1]->setMass(0.0);
		else
			rbvec[rbvec.size() - 1]->setMass(base->getDensity() * N->Volume() + WaterModel->Water0(i));
	}
	
	pwd::Queue<const pwd::Node*> Queue;
	std::vector<bool> Visited;
	Visited.resize(Graph->NumNodes());
	Queue.Enqueue(Graph->Root());
	CNode1.clear();
	CNode2.clear();
	while(!Queue.IsEmpty())
	{
		const pwd::Node* N = Queue.Dequeue();
		Visited[Graph->GetNodeID(N)] = true;
		for (int i = 0; i < N->Degree(); ++i)
		{
			const pwd::Node* ch = N->GetAdjacent(i);
			if (Visited[Graph->GetNodeID(ch)])
				continue;

			Vector3r JPos = ch->Head();

			Real LN = N->Length();
			Real RN = LN * N->Radius();

			Real LC = ch->Length();
			Real RC = LC * N->Radius();

			Real AvgRad = (RN * LN + RC * LC) / (LN + LC);
			Real AvgLen = 0.5 * (LN + LC);

			Vector2r DFromY(JPos[0], JPos[2]);


			int IDN = Graph->GetNodeID(N);
			int IDC = Graph->GetNodeID(ch);
			model->addStretchBendingTwistingConstraint(IDN, IDC, 
													   JPos, 
													   AvgRad, 
													   AvgLen, 
													   base->getYoungsMax(), 
													   base->getYoungsMax());

			CNode1.push_back(IDN);
			CNode2.push_back(IDC);

			Queue.Enqueue(ch);
		}
	}



}




std::chrono::system_clock::time_point timer;
void start_timer()
{
	timer = std::chrono::system_clock::now();
}

double stop_timer()
{
	std::chrono::system_clock::time_point tend;
	tend = std::chrono::system_clock::now();
	std::chrono::system_clock::duration eta = tend - timer;
	size_t etaus = std::chrono::duration_cast<std::chrono::microseconds>(eta).count();
	return etaus * 1.0e-3;
}
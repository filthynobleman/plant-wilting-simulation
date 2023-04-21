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
void parse_args(int argc, const char* const argv[]);
void export_state(std::ofstream& Stream, bool CreateNewFile);

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


std::ofstream Output;
int CurOutFrame = 0;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Plant demo");

	
	parse_args(argc, argv);
	std::cout << "Loading plant file " << PlantFile << "... ";
	try
	{
		Graph = new pwd::Graph(PlantFile);
	}
	catch(const std::exception& e)
	{
		std::cerr << '\n' << e.what() << '\n';
	}
	std::cout << "Done." << std::endl;
	std::cout << "Water model solution will be computed ";
	if (base->isExactSolution())
		std::cout << "exactly.";
	else
		std::cout << "approximatedly.";
	std::cout << std::endl;

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


	Output << ']';
	Output.close();
	
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
	CurOutFrame++;
	if (CurOutFrame >= base->getOutputEvery())
	{
		export_state(Output, false);
		CurOutFrame = 0;
	}
}

void buildModel ()
{
	createModel();
	base->resetWaterModel();

	CurOutFrame = 0;
	export_state(Output, true);
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
	if (base->isExactSolution())
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

void parse_args(int argc, const char* const argv[])
{
	base->useExactSolution(false);
	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--exact") == 0)
		{
			base->useExactSolution(true);
			continue;
		}
		PlantFile = std::string(argv[i]);
	}
}

































void export_state(std::ofstream& Stream, bool CreateNewFile)
{
	static std::vector<Quaternionr> Rot0;
	Matrix3r DefaultRot;
	DefaultRot.row(0) = Vector3r{ 0, -1, 0 };
	DefaultRot.row(1) = Vector3r{ -1, 0, 0 };
	DefaultRot.row(2) = Vector3r{ 0, 0, 1 };
	if (CreateNewFile)
	{
		// On file creation, export topology, positions and roll
		if (Stream.is_open())
			Stream.close();

		Stream.open("output.wilt", std::ios::out);
		assert(Stream.is_open());
		Stream << '[' << '\n';

		Stream << '[' << '\n';

		auto& rbvec = Simulation::getCurrent()->getModel()->getRigidBodies();
		pwd::Queue<const pwd::Node*> Queue;
		Queue.Enqueue(Graph->Root());
		std::vector<int> ParentID;
		ParentID.resize(Graph->NumNodes());
		std::vector<bool> Visited;
		Visited.resize(Graph->NumNodes());
		std::vector<int> VisitOrder;
		VisitOrder.resize(Graph->NumNodes());
		int CurID = 0;
		while (!Queue.IsEmpty())
		{
			const pwd::Node* N = Queue.Dequeue();
			Visited[Graph->GetNodeID(N)] = true;
			VisitOrder[Graph->GetNodeID(N)] = CurID;
			for (int i = 0; i < N->Degree(); ++i)
			{
				if (Visited[Graph->GetNodeID(N->GetAdjacent(i))])
					continue;
				Queue.Enqueue(N->GetAdjacent(i));
				ParentID[Graph->GetNodeID(N->GetAdjacent(i))] = Graph->GetNodeID(N);
			}

			auto& rb = rbvec[Graph->GetNodeID(N)];
			Quaternionr rot0 = rb->getRotation0();
			Quaternionr rot = rb->getRotation();
			Matrix3r rot0m = rot0.toRotationMatrix();
			Matrix3r rotm  =  rot.toRotationMatrix();

			Vector3r head = N->Head();
			Vector3r tail = N->Tail();
			// Convert to blender units
			head *= 1e-2;
			tail *= 1e-2;
			Real len = (tail - head).norm();

			Stream << "{\n";


			Stream << "\"id\" : " << CurID << ',' << '\n';
			
			if (Graph->GetNodeID(N) != Graph->GetNodeID(Graph->Root()))
				Stream << "\"parent\" : " << VisitOrder[ParentID[Graph->GetNodeID(N)]] << ',' << '\n';
			else
				Stream << "\"parent\" : " << -1 << ',' << '\n';
			
			Stream << "\"head_abs\" : ";
			Stream << '(' << head[0] << ',' << head[1] << ',' << head[2] << ")," << '\n';
			
			Stream << "\"tail_abs\" : ";
			Stream << '(' << tail[0] << ',' << tail[1] << ',' << tail[2] << ")," << '\n';

			Stream << "\"len\" : " << len << ',' << '\n';

			Stream << "\"rot0_abs\" : [";
			for (int i = 0; i < 3; ++i)
			{
				Stream << '[';
				for (int j = 0; j < 3; ++j)
				{
					Stream << rot0m.row(i)[j];
					if (j < 2) Stream << ',';
				}
				Stream << ']';
				if (i < 2) Stream << ',';
			}
			Stream << "]," << '\n';

			Stream << "\"rot_abs\" : [";
			for (int i = 0; i < 3; ++i)
			{
				Stream << '[';
				for (int j = 0; j < 3; ++j)
				{
					Stream << rotm.row(i)[j];
					if (j < 2) Stream << ',';
				}
				Stream << ']';
				if (i < 2) Stream << ',';
			}
			Stream << "]," << '\n';



			Stream << "}," << '\n';

			CurID++;
		}

		Stream << "]," << std::endl;
	}

	Stream << '[' << '\n';

	auto& rbvec = Simulation::getCurrent()->getModel()->getRigidBodies();
	pwd::Queue<const pwd::Node*> Queue;
	Queue.Enqueue(Graph->Root());
	std::vector<bool> Visited;
	Visited.resize(Graph->NumNodes());
	int CurID = 0;
	while (!Queue.IsEmpty())
	{
		const pwd::Node* N = Queue.Dequeue();
		Visited[Graph->GetNodeID(N)] = true;
		for (int i = 0; i < N->Degree(); ++i)
		{
			if (Visited[Graph->GetNodeID(N->GetAdjacent(i))])
				continue;
			Queue.Enqueue(N->GetAdjacent(i));
		}

		auto& rb = rbvec[Graph->GetNodeID(N)];
		Quaternionr rot0 = rb->getRotation0();
		Quaternionr rot = rb->getRotation();
		Matrix3r rot0m = rot0.toRotationMatrix();
		Matrix3r rotm  =  rot.toRotationMatrix();



		Stream << "{\n";

		Stream << "\"rot0_abs\" : [";
		for (int i = 0; i < 3; ++i)
		{
			Stream << '[';
			for (int j = 0; j < 3; ++j)
			{
				Stream << rot0m.row(i)[j];
				if (j < 2) Stream << ',';
			}
			Stream << ']';
			if (i < 2) Stream << ',';
		}
		Stream << "]," << '\n';

		Stream << "\"rot_abs\" : [";
		for (int i = 0; i < 3; ++i)
		{
			Stream << '[';
			for (int j = 0; j < 3; ++j)
			{
				Stream << rotm.row(i)[j];
				if (j < 2) Stream << ',';
			}
			Stream << ']';
			if (i < 2) Stream << ',';
		}
		Stream << "]," << '\n';



		Stream << "}," << '\n';
	}

	Stream << "]," << std::endl;
}
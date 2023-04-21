#ifndef __DemoBase_h__
#define __DemoBase_h__

#include "Common/Common.h"
#include "Utils/SceneLoader.h"
#include "Demos/Visualization/Shader.h"
#include "Simulation/TimeStep.h"
#include "Simulation/SimulationModel.h"
#include "ParameterObject.h"
#include "Simulator_GUI_imgui.h"

namespace PBD
{
	class DemoBase : public GenParam::ParameterObject
	{
	protected:
		unsigned int m_numberOfStepsPerRenderUpdate;
		std::string m_exePath;
		std::string m_outputPath;
		std::string m_sceneFile;
		std::string m_sceneName;
		bool m_useCache;
		GLint m_context_major_version;
		GLint m_context_minor_version;
		Shader m_shader;
		Shader m_shaderFlat;
		Shader m_shaderTex;
		bool m_doPause;
		Real m_pauseAt;
		bool m_renderTets;
		bool m_renderRefTets;
		bool m_renderContacts;
		bool m_renderAABB;
		bool m_renderSDF;
		int m_renderBVHDepth;
		int m_renderBVHDepthTets;
		Vector3r m_oldMousePos;
		std::vector<unsigned int> m_selectedBodies;
		std::vector<unsigned int> m_selectedParticles;
		Utilities::SceneLoader *m_sceneLoader;	
		Utilities::SceneLoader::SceneData m_scene;
		float m_jointColor[4] = { 0.0f, 0.6f, 0.2f, 1 };
		Simulator_GUI_imgui *m_gui;
		bool m_enableExportOBJ;
		bool m_enableExportPLY;
		unsigned int m_exportFPS;
		Real m_nextFrameTime;
		unsigned int m_frameCounter;

		// Plant simulation
		Real m_damping = 0.99;
		Real m_density = 1;
		Real m_leafdensity = 2;
		Real m_massscale = 1e-3;
		Real m_yngmin = 1;
		Real m_yngmax = 1e10;
		Real m_yngsteep = 1e3;
		Real m_yngmid = 1.5e-2;
		Real m_lossrate = 6e-3;
		Real m_water0 = 8;
		Real m_time = 0.0;
		Real m_timestep = 1e1;
		bool m_exactsol = false;
		int m_outevery = 100;


		virtual void initParameters();

		void initShaders();

		static void selection(const Vector2i &start, const Vector2i &end, void *clientData);
		static void mouseMove(int x, int y, void *clientData);

		void renderTriangleModels();
		void renderTetModels();
		void renderAABB(AABB &aabb);
		void renderSDF(CollisionDetection::CollisionObject* co);
		void renderBallJoint(BallJoint &bj);
		void renderRigidBodyParticleBallJoint(RigidBodyParticleBallJoint &bj);
		void renderBallOnLineJoint(BallOnLineJoint &bj);
		void renderHingeJoint(HingeJoint &hj);
		void renderUniversalJoint(UniversalJoint &uj);
		void renderSliderJoint(SliderJoint &joint);
		void renderTargetPositionMotorSliderJoint(TargetPositionMotorSliderJoint &joint);
		void renderTargetVelocityMotorSliderJoint(TargetVelocityMotorSliderJoint &joint);
		void renderTargetAngleMotorHingeJoint(TargetAngleMotorHingeJoint &hj);
		void renderTargetVelocityMotorHingeJoint(TargetVelocityMotorHingeJoint &hj);
		void renderRigidBodyContact(RigidBodyContactConstraint &cc);
		void renderParticleRigidBodyContact(ParticleRigidBodyContactConstraint &cc);
		void renderSpring(RigidBodySpring &s);
		void renderDistanceJoint(DistanceJoint &j);
		void renderDamperJoint(DamperJoint &j);

		void exportMeshOBJ(const std::string& exportFileName, const unsigned int nVert, const Vector3r* pos, const unsigned int nTri, const unsigned int* faces);
		void exportMeshPLY(const std::string& exportFileName, const unsigned int nVert, const Vector3r* pos, const unsigned int nTri, const unsigned int* faces);
		void exportOBJ();
		void exportPLY();

	public:
		static int PAUSE;
		static int PAUSE_AT;
		static int NUM_STEPS_PER_RENDER;
		static int RENDER_TETS;
		static int RENDER_TETS0;
		static int RENDER_CONTACTS;
		static int RENDER_AABB;
		static int RENDER_SDF;
		static int RENDER_BVH;
		static int RENDER_BVH_TETS;
		static int EXPORT_OBJ;
		static int EXPORT_PLY;
		static int EXPORT_FPS;

		// Plant simulation
		static int DAMPING;
		static int YNGMIN;
		static int YNGMAX;
		static int YNGSTEEP;
		static int YNGMID;
		static int DENSITY;
		static int LEAFDENSITY;
		static int MASSSCALE;
		static int LOSSRATE;
		static int WATER0;
		static int TIME;
		static int TIMESTEP;
		static int EXACTSOL;
		static int OUTEVERY;
		

		DemoBase();
		virtual ~DemoBase();

		void init(int argc, char **argv, const char *demoName);

		void createParameterGUI();

		void render();
		void cleanup();

		void readParameters();
		void readScene();
		void reset();
		void step();

		Utilities::SceneLoader *getSceneLoader() { return m_sceneLoader; }
		void setSceneLoader(Utilities::SceneLoader *sceneLoader) { m_sceneLoader = sceneLoader; }

		const std::string& getExePath() const { return m_exePath; }
		const std::string& getSceneFile() const { return m_sceneFile; }
		const std::string& getSceneName() const { return m_sceneName; }

		GLint getContextMajorVersion() const { return m_context_major_version; }
		GLint getContextMinorVersion() const { return m_context_minor_version; }
		Shader& getShader() { return m_shader; }
		Shader& getShaderTex() { return m_shaderTex; }
		Shader& getShaderFlat() { return m_shaderFlat; }
		void shaderTexBegin(const float *col);
		void shaderTexEnd();
		void shaderBegin(const float *col);
		void shaderEnd();
		void shaderFlatBegin(const float* col);
		void shaderFlatEnd();

		std::vector<unsigned int>& getSelectedParticles() { return m_selectedParticles; }
		std::vector<unsigned int>& getSelectedRigidBodies() { return m_selectedBodies; }
		bool getUseCache() const { return m_useCache; }
		void setUseCache(bool val) { m_useCache = val; }
		std::string getOutputPath() const { return m_outputPath; }

		Utilities::SceneLoader::SceneData& getSceneData() { return m_scene; }

		static void loadMesh(const std::string& filename, VertexData& vd, Utilities::IndexedFaceMesh& mesh, const Vector3r& translation = Vector3r::Zero(),
			const Matrix3r& rotation = Matrix3r::Identity(), const Vector3r& scale = Vector3r::Ones());



		// plant simulation
		Real getDampingForce() const { return m_damping; }
		Real getYoungsMin() const { return m_yngmin; }
		Real getYoungsMax() const { return m_yngmax; }
		Real getYoungsSteep() const { return m_yngsteep; }
		Real getYoungsMidpoint() const { return m_yngmid; }
		Real getDensity() const { return m_density; }
		Real getLeafDensity() const { return m_leafdensity; }
		Real getMassScale() const { return m_massscale; }
		Real getLossRate() const { return m_lossrate; }
		Real getInitialWater() const { return m_water0; }
		Real getTime() const { return m_time; }
		Real getTimeStep() const { return m_timestep; }
		bool isExactSolution() const { return m_exactsol; }
		void useExactSolution(bool useExact) { m_exactsol = useExact; }
		int getOutputEvery() const { return m_outevery; }
		void resetWaterModel();
	};
}
 
#endif
// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\ricca\Desktop\comau_racer_obstacle\UR5e\ASSIEME_finale_UR5e__env.SLDASM


#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "UR5e_env_1.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

std::string shapes_dir = "CAD_export3/UR5e_env_1_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("industrial_shelf-2");
body_1->SetPos(chrono::ChVector3d(0.208845590889814,1.31063702099535e-18,1.10509717924786));
body_1->SetRot(chrono::ChQuaternion<>(0.707106781186548,-3.00498981059771e-18,0.707106781186547,3.0049898105977e-18));
body_1->SetMass(186.498321285612);
body_1->SetInertiaXX(chrono::ChVector3d(75.429390265546,69.1546438739121,12.2640798323733));
body_1->SetInertiaXY(chrono::ChVector3d(-0.0022025681557986,2.54475096269195e-16,1.13821528599999e-14));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.62721857882186,-0.809509774202697),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,-2.83111701824924E-32,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,-1.90667064494336E-32,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,-1.90667064494336E-32,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478576,0.44,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.44,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_6.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.4,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,-2.83111701824924E-32,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478576,0.44,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_9.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.44,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.44,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_6.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.84,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Collision Model
body_1->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_1 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_1;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,0.195,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,0.195,-0.628845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.195,-0.628845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0250971792478576,0.635,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.635,-0.988845590889814), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=2.9274135154686E-32; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=-2.9274135154686E-32;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,2.03725750708509,0.435989192526648,0.0520000000000001);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.426,-0.808845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.195,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0250971792478576,0.635,-0.628845590889814), mr));
mr(0,0)=-1; mr(1,0)=-1.083073822851E-16; mr(2,0)=-3.17060494726427E-48;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=2.9274135154686E-32;
mr(0,2)=0; mr(1,2)=2.9274135154686E-32; mr(2,2)=-1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0640666753965284,0.38,0.424);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.645,-0.820845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0517981681477977,0.0512392141064311,0.39);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.635,-0.628845590889814), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=2.9274135154686E-32; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=-2.9274135154686E-32;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,2.03725750708509,0.435989192526648,0.0520000000000001);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.866,-0.808845590889814), mr));
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("industrial_shelf-1");
body_2->SetPos(chrono::ChVector3d(1.40884559088981,-1.65590618024033e-17,0.505097179247858));
body_2->SetRot(chrono::ChQuaternion<>(0.707106781186548,-3.00498981059771e-18,0.707106781186547,3.0049898105977e-18));
body_2->SetMass(186.498321285612);
body_2->SetInertiaXX(chrono::ChVector3d(75.429390265546,69.1546438739121,12.2640798323733));
body_2->SetInertiaXY(chrono::ChVector3d(-0.0022025681557986,2.54475096269195e-16,1.13814372059702e-14));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.62721857882186,-0.809509774202697),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,-3.08148791101958E-32,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,-2.46519032881566E-32,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,-2.46519032881566E-32,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478575,0.44,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.44,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_6.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.4,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,-3.08148791101958E-32,-0.988845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0.0250971792478575,0.44,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_9.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.44,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.44,-0.628845590889814), chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_6.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.84,-0.808845590889814), chrono::ChQuaternion<>(0.707106781186547,-0.707106781186547,0,0)));

// Collision Model
body_2->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_2 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_2;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,0.195,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.0250971792478571,0.195,-0.628845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.195,-0.628845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.0250971792478575,0.635,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.635,-0.988845590889814), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=2.9274135154686E-32; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=-2.9274135154686E-32;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,2.03725750708509,0.435989192526648,0.0520000000000001);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.426,-0.808845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.195,-0.988845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.0250971792478575,0.635,-0.628845590889814), mr));
mr(0,0)=-1; mr(1,0)=-1.083073822851E-16; mr(2,0)=-3.17060494726427E-48;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=2.9274135154686E-32;
mr(0,2)=0; mr(1,2)=2.9274135154686E-32; mr(2,2)=-1;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0640666753965284,0.38,0.424);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.645,-0.820845590889814), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.0517981681477977,0.0512392141064311,0.39);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.98509717924786,0.635,-0.628845590889814), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=2.9274135154686E-32; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=-2.9274135154686E-32;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,2.03725750708509,0.435989192526648,0.0520000000000001);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(1.00509717924786,0.866,-0.808845590889814), mr));
body_2->EnableCollision(true);

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("Link4_UR5_STEP_1000515-1");
body_3->SetPos(chrono::ChVector3d(0.508789924723371,0.881299999999981,4.40619762898109e-16));
body_3->SetRot(chrono::ChQuaternion<>(0.500000000000016,8.85256044027966e-16,2.09952851344052e-16,0.86602540378443));
body_3->SetMass(1.98715438806532);
body_3->SetInertiaXX(chrono::ChVector3d(0.00374494513363848,0.00405290094593909,0.00333128550628577));
body_3->SetInertiaXY(chrono::ChVector3d(-0.000266854238789509,-8.58909661690962e-05,-4.98247245716833e-05));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.332131640132098,0.395740681286961,0.12723374477465),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_3->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_3 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_3;
mr(0,0)=-4.86759381965911E-16; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=-0.499999999999968; mr(1,1)=-0.866025403784457; mr(2,1)=0;
mr(0,2)=-0.866025403784457; mr(1,2)=0.499999999999968; mr(2,2)=4.21545990312901E-16;
collshape_3 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_3,0.114042283082579,0.100474309500103,0.116);
body_3->GetCollisionModel()->AddShape(collshape_3,chrono::ChFramed(chrono::ChVector3d(0.329522666139989,0.397250000000015,0.1333), mr));
body_3->EnableCollision(true);

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("Link2_UR5_STEP-1");
body_4->SetPos(chrono::ChVector3d(0,0,0));
body_4->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_4->SetMass(13.4869578178459);
body_4->SetInertiaXX(chrono::ChVector3d(0.446831193472711,0.0363743847510672,0.440722635300511));
body_4->SetInertiaXY(chrono::ChVector3d(-1.77024123615327e-06,2.28317030040725e-07,0.000235444714445938));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-6.15152218940188e-07,0.366921489597504,0.14411017605433),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_4->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_4 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_4;
mr(0,0)=0; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=0;
mr(0,2)=-1; mr(1,2)=0; mr(2,2)=0;
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_4,0.141822171420531,0.58787819687177,0.12);
body_4->GetCollisionModel()->AddShape(collshape_4,chrono::ChFramed(chrono::ChVector3d(0,0.364109885978337,0.146268056886561), mr));
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("Base_UR5_STEP-1");
body_5->SetPos(chrono::ChVector3d(0,0,0));
body_5->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_5->SetMass(4.80160121153642);
body_5->SetInertiaXX(chrono::ChVector3d(0.0275494062842208,0.0386615054177464,0.0199299189225059));
body_5->SetInertiaXY(chrono::ChVector3d(5.14955818071587e-07,6.4511125414673e-08,-0.00127937473743534));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.27760013272622e-06,0.0531074916233878,-0.00611136468161377),chrono::ChQuaternion<>(1,0,0,0)));
body_5->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_5->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_5 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_5;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.209088419740169,0.18379402801915,0.105);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(0,0.0525,0), mr));
body_5->EnableCollision(true);

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("Link1_UR5_STEP-1");
body_6->SetPos(chrono::ChVector3d(0,0,0));
body_6->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_6->SetMass(5.30876614321518);
body_6->SetInertiaXX(chrono::ChVector3d(0.0307324141847837,0.0119449603656888,0.0303338138457506));
body_6->SetInertiaXY(chrono::ChVector3d(-0.00047153765263149,-0.000167981577263375,-0.000469095473076001));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.00957883477798284,0.141272822478325,0.00330414209848852),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_6_1.obj");
body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_6->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_6 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_6;
chrono::ChVector3d p1_body_6(-0.0137401657281089,0.264,0);
chrono::ChVector3d p2_body_6(-0.0137401657281089,0,0);
body_6->GetCollisionModel()->AddCylinder(mat_6,0.0668003634838783,p1_body_6,p2_body_6);
body_6->EnableCollision(true);

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("Link3_UR5_STEP-1");
body_7->SetPos(chrono::ChVector3d(2.43149693754354e-15,0.9355,0.00659999999999983));
body_7->SetRot(chrono::ChQuaternion<>(0.500000000000001,0.499999999999999,-0.499999999999999,-0.500000000000001));
body_7->SetMass(10.2485241510182);
body_7->SetInertiaXX(chrono::ChVector3d(0.216745059208228,0.21803542984154,0.0177569102851551));
body_7->SetInertiaXY(chrono::ChVector3d(-1.14520746177216e-05,-0.00419453180122824,-1.13634753356105e-05));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.164536440839439,0.00184603725784859,-4.13236819091921e-06),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_7_1.obj");
body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_7->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_7 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_7;
mr(0,0)=0; mr(1,0)=1.26161707343768E-16; mr(2,0)=-1;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=-1.20676415720126E-16;
mr(0,2)=-1; mr(1,2)=0; mr(2,2)=0;
collshape_7 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_7,0.11,0.115,0.513);
body_7->GetCollisionModel()->AddShape(collshape_7,chrono::ChFramed(chrono::ChVector3d(0.1615,1.38777878078145E-17,-6.93889390390723E-18), mr));
body_7->EnableCollision(true);

bodylist.push_back(body_7);



// Rigid body part
auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_8->SetName("Link5_UR5_STEP-1");
body_8->SetPos(chrono::ChVector3d(0.508789924723371,0.881299999999981,1.25561459484425e-15));
body_8->SetRot(chrono::ChQuaternion<>(0.500000000000016,4.32083215997192e-17,-1.4123247291576e-16,0.86602540378443));
body_8->SetMass(1.26653751294046);
body_8->SetInertiaXX(chrono::ChVector3d(0.00173387706481413,0.00179202686878704,0.00129124808161356));
body_8->SetInertiaXY(chrono::ChVector3d(-5.07900554045803e-05,-1.60832333990906e-05,-9.38652107154482e-06));
body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.423155408864203,0.343195767732578,0.127539242186587),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_8_1.obj");
body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_8->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_8 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_8;
chrono::ChVector3d p1_body_8(0.425997896121578,0.341550000000018,0.0716);
chrono::ChVector3d p2_body_8(0.425997896121578,0.341550000000018,0.1796);
body_8->GetCollisionModel()->AddCylinder(mat_8,0.0462646642673956,p1_body_8,p2_body_8);
body_8->EnableCollision(true);

bodylist.push_back(body_8);



// Rigid body part
auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_9->SetName("Link6_UR5_STEP-1");
body_9->SetPos(chrono::ChVector3d(-8.44661016281028e-16,1.07945,0.232900000000001));
body_9->SetRot(chrono::ChQuaternion<>(6.08546238857962e-17,0.741156669705821,0.671332101832303,9.36224982858403e-17));
body_9->SetMass(0.531572472308725);
body_9->SetInertiaXX(chrono::ChVector3d(0.000345921264373931,0.000345224158878349,0.000469325826357456));
body_9->SetInertiaXY(chrono::ChVector3d(-6.97567041937565e-08,1.69646723943244e-07,-1.68194479350155e-08));
body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(4.93732275760031e-10,7.40815735846451e-06,0.0260552428320294),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_9_1.obj");
body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_9->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_9 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_9;
chrono::ChVector3d p1_body_9(4.08006961549779E-18,-9.59302082215214E-18,0.049);
chrono::ChVector3d p2_body_9(0,0,0);
body_9->GetCollisionModel()->AddCylinder(mat_9,0.0450659241943092,p1_body_9,p2_body_9);
body_9->EnableCollision(true);

bodylist.push_back(body_9);




// Mate constraint: Concentrico1 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: Base_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_6 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-5.8418494793746e-16,0.0991,-1.05344719486481e-16);
dA = chrono::ChVector3d(-1.88622596932378e-16,-1,-1.52137599783086e-16);
cB = chrono::ChVector3d(-5.95768274398936e-16,0.0991,-1.16349079764615e-16);
dB = chrono::ChVector3d(0,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-5.8418494793746e-16,0.0991,-1.05344719486481e-16);
cB = chrono::ChVector3d(-5.95768274398936e-16,0.0991,-1.16349079764615e-16);
dA = chrono::ChVector3d(-1.88622596932378e-16,-1,-1.52137599783086e-16);
dB = chrono::ChVector3d(0,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico1");
linklist.push_back(link);


// Mate constraint: Coincidente1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: Base_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(1.15833264614759e-17,0.0991,1.10023101740352e-17);
cB = chrono::ChVector3d(-5.95768274398936e-16,0.0991,0.058);
dA = chrono::ChVector3d(1.1688523170006e-16,1,1.11022302462515e-16);
dB = chrono::ChVector3d(0,-1,2.06872260413245e-20);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(1.15833264614759e-17,0.0991,1.10023101740352e-17);
dA = chrono::ChVector3d(1.1688523170006e-16,1,1.11022302462515e-16);
cB = chrono::ChVector3d(-5.95768274398936e-16,0.0991,0.058);
dB = chrono::ChVector3d(0,-1,2.06872260413245e-20);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente1");
linklist.push_back(link);


// Mate constraint: Concentrico2 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_6 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(4.85779486057579e-16,0.1625,0.0744);
dA = chrono::ChVector3d(-2.40763165915719e-16,1.83109907586861e-17,-1);
cB = chrono::ChVector3d(-3.78020694416034e-16,0.1625,0.0744);
dB = chrono::ChVector3d(-1.51407237036752e-16,2.39272203583008e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_6,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(4.85779486057579e-16,0.1625,0.0744);
cB = chrono::ChVector3d(-3.78020694416034e-16,0.1625,0.0744);
dA = chrono::ChVector3d(-2.40763165915719e-16,1.83109907586861e-17,-1);
dB = chrono::ChVector3d(-1.51407237036752e-16,2.39272203583008e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_6,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico2");
linklist.push_back(link);


// Mate constraint: Coincidente2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_6 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(4.6400711319521e-16,0.1625,0.0744);
cB = chrono::ChVector3d(-2.81833747120072e-16,0.2205,0.0744);
dA = chrono::ChVector3d(0,0,1);
dB = chrono::ChVector3d(-1.51407237036752e-16,2.52283844176854e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_6,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(4.6400711319521e-16,0.1625,0.0744);
dA = chrono::ChVector3d(0,0,1);
cB = chrono::ChVector3d(-2.81833747120072e-16,0.2205,0.0744);
dB = chrono::ChVector3d(-1.51407237036752e-16,2.52283844176854e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_6,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente2");
linklist.push_back(link);


// Mate constraint: Concentrico4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_7 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(2.38977494309316e-15,0.5875,0.0744);
dA = chrono::ChVector3d(1.18996360814429e-16,-1.28809687132212e-31,-1);
cB = chrono::ChVector3d(2.42730596854457e-15,0.5875,0.0743999999999999);
dB = chrono::ChVector3d(-7.6258746745367e-17,4.8572257327351e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(2.38977494309316e-15,0.5875,0.0744);
cB = chrono::ChVector3d(2.42730596854457e-15,0.5875,0.0743999999999999);
dA = chrono::ChVector3d(1.18996360814429e-16,-1.28809687132212e-31,-1);
dB = chrono::ChVector3d(-7.6258746745367e-17,4.8572257327351e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico4");
linklist.push_back(link);


// Mate constraint: Coincidente9 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(2.3269918310506e-15,0.5295,0.0744);
cB = chrono::ChVector3d(-0.0579999999999973,0.5875,0.0744000000000001);
dA = chrono::ChVector3d(1.18996360814429e-16,-1.7680142122614e-17,-1);
dB = chrono::ChVector3d(-7.6258746745367e-17,9.07701672551621e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_7,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente9");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(2.3269918310506e-15,0.5295,0.0744);
dA = chrono::ChVector3d(1.18996360814429e-16,-1.7680142122614e-17,-1);
cB = chrono::ChVector3d(-0.0579999999999973,0.5875,0.0744000000000001);
dB = chrono::ChVector3d(-7.6258746745367e-17,9.07701672551621e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente9");
linklist.push_back(link);


// Mate constraint: Concentrico6 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_7 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-2.15862312334607e-15,0.97975,0.0529000000000013);
dA = chrono::ChVector3d(-1.24849607005467e-16,-8.05424942890538e-17,1);
cB = chrono::ChVector3d(-1.99840144432528e-15,0.97975,0.0534153120765819);
dB = chrono::ChVector3d(-1.23259716914287e-16,1.23822748943089e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-2.15862312334607e-15,0.97975,0.0529000000000013);
cB = chrono::ChVector3d(-1.99840144432528e-15,0.97975,0.0534153120765819);
dA = chrono::ChVector3d(-1.24849607005467e-16,-8.05424942890538e-17,1);
dB = chrono::ChVector3d(-1.23259716914287e-16,1.23822748943089e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_7,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico6");
linklist.push_back(link);


// Mate constraint: Coincidente12 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_7 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-2.15862312334607e-15,0.97975,0.0529000000000013);
cB = chrono::ChVector3d(-0.000240457452144183,0.980809818190146,0.0529000000000013);
dA = chrono::ChVector3d(-6.23809589375928e-17,-8.05424942890538e-17,1);
dB = chrono::ChVector3d(8.46699792424206e-17,-4.21237780994433e-17,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_7,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente12");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-2.15862312334607e-15,0.97975,0.0529000000000013);
dA = chrono::ChVector3d(-6.23809589375928e-17,-8.05424942890538e-17,1);
cB = chrono::ChVector3d(-0.000240457452144183,0.980809818190146,0.0529000000000013);
dB = chrono::ChVector3d(8.46699792424206e-17,-4.21237780994433e-17,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente12");
linklist.push_back(link);


// Mate constraint: Concentrico7 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_3 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_8 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-8.88178419700125e-16,1.02605,0.133300000000001);
dA = chrono::ChVector3d(-5.55111512312589e-17,1,-2.00188469437666e-17);
cB = chrono::ChVector3d(-8.88178419700125e-16,1.02656531207658,0.133300000000001);
dB = chrono::ChVector3d(5.55111512312578e-17,1,2.69801922852095e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico7");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-8.88178419700125e-16,1.02605,0.133300000000001);
cB = chrono::ChVector3d(-8.88178419700125e-16,1.02656531207658,0.133300000000001);
dA = chrono::ChVector3d(-5.55111512312589e-17,1,-2.00188469437666e-17);
dB = chrono::ChVector3d(5.55111512312578e-17,1,2.69801922852095e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_3,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico7");
linklist.push_back(link);


// Mate constraint: Coincidente14 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_3 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-8.88178419700125e-16,1.02605,0.133300000000001);
cB = chrono::ChVector3d(-0.0271252030758693,1.02605,0.107842911437228);
dA = chrono::ChVector3d(-5.55111512312589e-17,1,-2.00188469437666e-17);
dB = chrono::ChVector3d(7.77156117237609e-16,-1,6.35226551947206e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_3,body_8,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente14");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-8.88178419700125e-16,1.02605,0.133300000000001);
dA = chrono::ChVector3d(-5.55111512312589e-17,1,-2.00188469437666e-17);
cB = chrono::ChVector3d(-0.0271252030758693,1.02605,0.107842911437228);
dB = chrono::ChVector3d(7.77156117237609e-16,-1,6.35226551947206e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente14");
linklist.push_back(link);


// Mate constraint: Concentrico8 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_8 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_9 , SW name: Link6_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-7.7715611723761e-16,1.07945,0.179600000000001);
dA = chrono::ChVector3d(-2.26810111548179e-17,-1.75546173734874e-16,1);
cB = chrono::ChVector3d(-8.31458141281698e-16,1.07945,0.179600000000001);
dB = chrono::ChVector3d(-1.5095949961703e-17,-1.92780258353003e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_8,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-7.7715611723761e-16,1.07945,0.179600000000001);
cB = chrono::ChVector3d(-8.31458141281698e-16,1.07945,0.179600000000001);
dA = chrono::ChVector3d(-2.26810111548179e-17,-1.75546173734874e-16,1);
dB = chrono::ChVector3d(-1.5095949961703e-17,-1.92780258353003e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_8,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico8");
linklist.push_back(link);


// Mate constraint: Coincidente15 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_8 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: Link6_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-7.7715611723761e-16,1.07945,0.179600000000001);
cB = chrono::ChVector3d(-8.31648637210829e-16,1.07945,0.179600000000001);
dA = chrono::ChVector3d(-2.26810111548179e-17,-1.75546173734874e-16,1);
dB = chrono::ChVector3d(1.75922117220293e-16,-3.60947120866755e-17,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_8,body_9,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente15");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-7.7715611723761e-16,1.07945,0.179600000000001);
dA = chrono::ChVector3d(-2.26810111548179e-17,-1.75546173734874e-16,1);
cB = chrono::ChVector3d(-8.31648637210829e-16,1.07945,0.179600000000001);
dB = chrono::ChVector3d(1.75922117220293e-16,-3.60947120866755e-17,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_8,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente15");
linklist.push_back(link);


// Mate constraint: Coincidente16 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: Base_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: industrial_shelf-1/table_leg-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-5.87830463590729e-19,0,0);
cB = chrono::ChVector3d(0.42,-2.4963650766724e-17,0.480000000000001);
dA = chrono::ChVector3d(0,-1,0);
dB = chrono::ChVector3d(8.49939468988047e-18,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente16");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-5.87830463590729e-19,0,0);
dA = chrono::ChVector3d(0,-1,0);
cB = chrono::ChVector3d(0.42,-2.4963650766724e-17,0.480000000000001);
dB = chrono::ChVector3d(8.49939468988047e-18,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente16");
linklist.push_back(link);


// Mate constraint: Distanza1 [MateDistanceDim] type:5 align:0 flip:True
//   Entity 0: C::E name: body_2 , SW name: industrial_shelf-1/table_leg-6 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_0 , SW name: ASSIEME_finale_UR5e__env ,  SW ref.type:4 (4)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.440000000000001,0.84,0.5);
cB = chrono::ChVector3d(0,0,0);
dA = chrono::ChVector3d(0,0,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_0,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.5);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.440000000000001,0.84,0.5);
dA = chrono::ChVector3d(0,0,1);
cB = chrono::ChVector3d(0,0,0);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_0,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza1");
linklist.push_back(link);


// Mate constraint: Distanza2 [MateDistanceDim] type:5 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: industrial_shelf-1/table_leg-6 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_0 , SW name: ASSIEME_finale_UR5e__env ,  SW ref.type:4 (4)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.4,0.84,0.5);
cB = chrono::ChVector3d(0,0,0);
dA = chrono::ChVector3d(-1,-8.49939468988049e-18,0);
dB = chrono::ChVector3d(1,0,0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_0,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.4);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.4,0.84,0.5);
dA = chrono::ChVector3d(-1,-8.49939468988049e-18,0);
cB = chrono::ChVector3d(0,0,0);
dB = chrono::ChVector3d(1,0,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_0,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza2");
linklist.push_back(link);


// Mate constraint: Coincidente19 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: Base_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: industrial_shelf-2/table_leg-4 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-5.87830463590729e-19,0,0);
cB = chrono::ChVector3d(-0.42,-4.0341698549683e-18,1.08);
dA = chrono::ChVector3d(0,-1,0);
dB = chrono::ChVector3d(8.49939468988047e-18,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente19");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-5.87830463590729e-19,0,0);
dA = chrono::ChVector3d(0,-1,0);
cB = chrono::ChVector3d(-0.42,-4.0341698549683e-18,1.08);
dB = chrono::ChVector3d(8.49939468988047e-18,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente19");
linklist.push_back(link);


// Mate constraint: Distanza3 [MateDistanceDim] type:5 align:0 flip:True
//   Entity 0: C::E name: body_1 , SW name: industrial_shelf-2/table_leg-7 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: industrial_shelf-1/table_leg-6 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.4,0.84,1.1);
cB = chrono::ChVector3d(0.440000000000001,0.84,0.5);
dA = chrono::ChVector3d(0,0,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.6);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.4,0.84,1.1);
dA = chrono::ChVector3d(0,0,1);
cB = chrono::ChVector3d(0.440000000000001,0.84,0.5);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza3");
linklist.push_back(link);


// Mate constraint: Distanza4 [MateDistanceDim] type:5 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: industrial_shelf-2/shelf_divider-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_0 , SW name: ASSIEME_finale_UR5e__env ,  SW ref.type:4 (4)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.4,0.84,0.120000000000001);
cB = chrono::ChVector3d(0,0,0);
dA = chrono::ChVector3d(1,8.49939468988049e-18,0);
dB = chrono::ChVector3d(1,0,0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_0,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(-0.4);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.4,0.84,0.120000000000001);
dA = chrono::ChVector3d(1,8.49939468988049e-18,0);
cB = chrono::ChVector3d(0,0,0);
dB = chrono::ChVector3d(1,0,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_0,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("MARKER_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-5.8418494793746E-16,0.0991,-1.05344719486481E-16),chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,4.13251699778365E-17,-4.13251699778365E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("MARKER_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(4.64239139559248E-16,0.1625,0.0728162915038653),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("MARKER_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(2.38977494309316E-15,0.5875,0.0744),chrono::ChQuaternion<>(5.94981804072146E-17,5.25968142818241E-34,1,-8.840071061307E-18)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("MARKER_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-2.15317939407874E-15,0.97975,0.0523000000000013),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("MARKER_5");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-8.88178419700125E-16,1.02605,0.133300000000001),chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,-1.96261557335476E-17,1.96261557335476E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("MARKER_6");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-7.7715611723761E-16,1.07945,0.179600000000001),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("MARKER_TCP");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-8.44661016281028E-16,1.07945,0.232900000000001),chrono::ChQuaternion<>(1,0,0,0)));


} // end function

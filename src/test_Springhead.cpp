/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <test_Springhead.h>

MyApp::MyApp() : FWApp()
{
  bDrawInfo = false;
}

MyApp::~MyApp()
{
}

void MyApp::Init(int ac, char **av)
{
  FWApp::Init(ac, av);
  InitCameraView();
  CreateObjects();
  CreateTimer();
}

void MyApp::TimerFunc(int id)
{
//  FWApp::TimerFunc(id); // skip default
  GetSdk()->Step();
  PostRedisplay();
}

void MyApp::Display()
{
  FWApp::Display(); // skip default
  FWSceneIf *fwScene = GetSdk()->GetScene();
  fwScene->EnableRenderAxis(bDrawInfo);
  fwScene->EnableRenderForce(bDrawInfo);
  fwScene->EnableRenderContact(bDrawInfo);
/*
//  GetSdk()->SetDebugMode(true);
  GetSdk()->GetRender()->SetViewMatrix(
    GetCurrentWin()->GetTrackball()->GetAffine().inv());
  GetSdk()->Draw();
  GetSdk()->GetRender()->SwapBuffers();
*/
}

void MyApp::Keyboard(int key, int x, int y)
{
//  FWApp::Keyboard(key, x, y); // skip default
  switch(key){
  case 0x1b: // ESC
  case 'q':
    FWApp::Clear();
    exit(0);
    break;
  case 'r':
    Reset();
    break;
  case 'd':
    bDrawInfo = !bDrawInfo;
    break;
/*
  case '1':
    DSTR << "box" << std::endl;
    CreateBox(GetSdk());
    break;
  case '2':
    DSTR << "sphere" << std::endl;
    CreateSphere(GetSdk());
    break;
  case '3':
    DSTR << "capsule" << std::endl;
    CreateCapsule(GetSdk());
    break;
  case '4':
    DSTR << "roundcone" << std::endl;
    CreateRoundCone(GetSdk());
    break;
*/
  default: break;
  }
}

void MyApp::InitCameraView()
{
//  FWApp::InitCameraView(); // no method
}

void MyApp::CreateObjects()
{
//  FWApp::CreateObjects(); // no method
//  PHSdkIf *phSdk = PHSdkIf::CreateSdk(); // singleton created by Framework
  PHSdkIf *phSdk = GetSdk()->GetPHSdk();
//  PHSceneIf *phScene = phSdk->CreateScene(); // create multi instance OK
  PHSceneIf *phScene = GetSdk()->GetScene()->GetPHScene();
/**/
  Vec3d gr = phScene->GetGravity(); // 0.0, -9.8, 0.0
  fprintf(stdout, "Scene gravity: %20.17f, %20.17f, %20.17f\n",
    gr.x, gr.y, gr.z);
//  double air = phScene->GetAirResistanceRate(); // no method
//  fprintf(stdout, "Scene airResistanceRate: %20.17f\n", air);
  fprintf(stdout, "Scene numIteration: %d\n", phScene->GetNumIteration());
  fprintf(stdout, "Scene timeStep: %20.17f\n", phScene->GetTimeStep());
fprintf(stdout, "%20.17f sec\n", phScene->GetTimeStep() * phScene->GetCount());
/**/
  phScene->SetTimeStep(0.001); // default == 0.005
  GetSdk()->SetDebugMode(true); // true works without camera light etc

/*
//  GRSdkIf *grSdk = GRSdkIf::CreateSdk();
  GRSdkIf *grSdk = GetSdk()->GetGRSdk();
//  GRSceneIf *grScene = grSdk->CreateScene();
//  GRSceneIf *grScene = GetSdk()->GetScene()->GetGRScene();
  GRSceneIf *grScene = grSdk->GetScene(0);
  GRFrameIf *frm = grScene->GetWorld();
//  GRFrameDesc frmd;
//  frm->GetDesc(&frmd);
  frm->SetTransform(Affinef::Trn(3.0, 0.0, 0.0));
  GRCameraIf *cam = grScene->GetCamera(); // null
  GRCameraDesc camd;
  cam->GetDesc(&camd);
  camd.front = 3.0f;
  grScene->SetCamera(camd);
*/

/*
//  FWSdkIf *fwSdk = FWSdkIf::CreateSdk();
  FWSdkIf *fwSdk = GetSdk();
  FWSceneIf *fwScene = fwSdk->GetScene(0);
  GRFrameIf *frm = fwScene->GetGRScene()->GetWorld();
*/

  Quaterniond q = Quaterniond::Rot(Rad(45.0), Vec3d(1, 1, 1));
//  Matrix3d rot;
//  q.ToMatrix(rot);
  CDBoxDesc bd;
  PHSolidDesc sd;
  sd.mass = 1.0;

  PHSolidIf *floor = phScene->CreateSolid();
  floor->SetDynamical(false);
  floor->SetMass(1000.0);
  bd.boxsize = Vec3f(5.0f, 1.0f, 5.0f);
  floor->AddShape(phSdk->CreateShape(bd));
  floor->SetFramePosition(Vec3d(0, -1.0, 0));

  PHSolidIf *box = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  box->AddShape(phSdk->CreateShape(bd));
  box->SetCenterPosition(Vec3d(0.0, 1.0, 0.0));
  box->SetVelocity(Vec3d(0.0, 0.7, 0.0));
  box->SetOrientation(q);
//  box->AddTorque(-Vec3d(1.0, 1.0, 5.0));
  box->AddForce(-Vec3d(0.0, 0.0, -5.0), Vec3d(0.15, 0.85, 0.0));

  PHSolidIf *sol = phScene->CreateSolid(sd); // sol->SetMass(5.0); // etc
  bd.boxsize = Vec3f(0.5f, 0.3f, 0.3f);
  sol->AddShape(phSdk->CreateShape(bd));
  sol->SetCenterPosition(Vec3d(0.0, 1.5, 0.0));
  sol->SetVelocity(Vec3d(0.0, 0.7, 0.0));
  sol->SetAngularVelocity(-Vec3d(0.5, 0.5, 0.5));

  PHSolidIf *sol0 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol0->AddShape(phSdk->CreateShape(bd));
  sol0->SetCenterPosition(Vec3d(0.5, 0.5, -0.5));
  PHSolidIf *sol1 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol1->AddShape(phSdk->CreateShape(bd));
  sol1->SetCenterPosition(Vec3d(0.5, 0.5, 0.5));
  PHHingeJointDesc hjd;
  hjd.poseSocket.Pos() = Vec3d(1.0, 0.0, 0.0);
  hjd.posePlug.Pos() = Vec3d(-1.0, 0.0, 0.0);
  PHHingeJointIf *joint = phScene->CreateJoint(sol0, sol1, hjd)->Cast();
}

void MyApp::Reset()
{
//  FWApp::Reset(); // skip default
  GetSdk()->GetScene()->GetPHScene()->Clear();
  CreateObjects();
}

int main(int ac, char **av)
{
  fprintf(stdout, "sizeof(size_t): %zd\n", sizeof(size_t));
  fprintf(stdout, "%s\n", TEST_WORD);
  MyApp app; // not catch exception AtExit() when declared at global
  app.Init(ac, av);
  app.StartMainLoop();
  return 0;
}

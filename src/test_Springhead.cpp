/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <test_Springhead.h>

float PNR[][2] = {
  {    15.0f, 0.800f}, // 0.000f}, //
  {14+1/2.0f, 1.870f}, //
  {    14.0f, 2.472f}, //
  {13+1/2.0f, 2.547f},
  {12+5/8.0f, 2.406f},
  {11+3/4.0f, 2.094f},
  {10+7/8.0f, 1.870f},
  {    10.0f, 1.797f},
  { 9+3/8.0f, 1.965f},
  { 8+5/8.0f, 2.472f},
  { 7+1/4.0f, 3.703f},
  { 5+7/8.0f, 4.563f},
  { 4+1/2.0f, 4.766f},
  { 3+3/8.0f, 4.510f},
  { 2+1/4.0f, 3.906f},
  {   3/4.0f, 2.828f},
  {     0.0f, 2.250f}};
int PNI[][2] = {{0, 4}, {4, 9}, {9, 13}, {13, 16}};

PHSolidIf *CreateConvexMeshPin(FWSdkIf *fwSdk)
{
  PHSceneIf *phScene = fwSdk->GetScene()->GetPHScene();
  float pi = 3.14159265358979323846264338327950288419716939937510f;
  float r = 0.2f;
  int t = 18; // 12;
  const int m = sizeof(PNR) / sizeof(PNR[0]);
  const int l = sizeof(PNI) / sizeof(PNI[0]);
  PHJointIf *jois[l - 1];
  PHSolidIf *cvxs[l];
  for(int i = 0; i < l; ++i){
    int n = PNI[i][1] - PNI[i][0] + 1;
    std::vector<Vec3f> vertices(n * t);
    float prev, o[2], q[2];
    for(int j = 0; j < n; ++j){
      float *p = &PNR[m - (j + PNI[i][0]) - 1][0];
      float p0 = p[0] * r, p1 = p[1] * r / 2.0f;
      if(!j) o[0] = p0, q[0] = p1; else if(j == n - 1) o[1] = p0, q[1] = p1;
      for(int k = 0; k < t; ++k){
        float th = 2.0f * pi * k / t;
        vertices[j * t + k].x = p1 * cos(th);
        vertices[j * t + k].y = p0 - o[0];
        vertices[j * t + k].z = p1 * sin(th);
      }
    }
    PHSolidDesc desc;
    desc.mass = (o[1] - o[0]) * (q[0] + q[1]) / 2.0f;
    desc.inertia *= 0.033;
    desc.center = Vec3d(0, 0, 0);
    desc.dynamical = true;
    // desc.velocity = Vec3d(0, 0, 0);
    // desc.angVelocity = Vec3d(0, 0, 0);
    // desc.pose = Posed();
    // desc.pose.Pos() = Vec3d(0, 0, 0); // relation from mass center ?
    // desc.pose.Ori() = Quaterniond::Rot(Rad(360.0), 'y');
    cvxs[i] = phScene->CreateSolid(desc);
    CDConvexMeshDesc cmd;
    cmd.vertices = vertices;
    cmd.material.density = 1.0;
    // cmd.material.mu0 = ;
    // cmd.material.mu = ;
    cmd.material.e = 1.0f;
    // cmd.material.reflexSpring = ;
    // cmd.material.reflexDamper = ;
    // cmd.material.frictionSpring = ;
    // cmd.material.frictionDamper = ;
    CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
    cvxs[i]->SetMass(shapeCvx->CalcVolume() * shapeCvx->GetDensity());
    cvxs[i]->SetCenterOfMass(shapeCvx->CalcCenterOfMass());
    cvxs[i]->SetInertia(shapeCvx->CalcMomentOfInertia());
    cvxs[i]->AddShape(shapeCvx);
    cvxs[i]->SetFramePosition(Vec3d(0, 5 + o[0], 0));
//    DispVertices(shapeCvx);
    if(i == 0) prev = o[0];
    else{
      PHHingeJointDesc jd; // PHJoint1D PHJoint PHConstraint
      // PHSliderJointDesc jd; // PHJoint1D PHJoint PHConstraint
      // PHPathJointDesc jd; // PHJoint1D PHJoint PHConstraint
      // PHBallJoint jd; // PHJoint PHConstraint
      // PHSpring jd; // PHJoint PHConstraint
      // PHContactPoint jd; // PHConstraint (auto generated)
      // F = springK(targetPosition - p) + damperD(targetVelocity - v) + offset
      jd.bEnabled = true; // bool (PHConstraint)
      // jd.range = Vec2d(0, 0); // Vec2d (PH1DJointLimit)
      jd.spring = 1000; // Vec3d (PHSpring) double (otherwise)
      jd.damper = 1000; // Vec3d (PHSpring) double (otherwise)
      // jd.springOri = ; // double (PHSpring)
      // jd.damperOri = ; // double (PHSpring)
      // jd.targetPosition = ; // Quaterniond (PHBallJoint) double (PHJoint1D)
      // jd.targetVelocity = ; // Vec3d (PHBallJoint) double (PHJoint1D)
      // jd.offsetForce = ; // Vec3d (PHBallJoint) double (PHJoint1D)
      jd.fMax = 1000; // Vec3d (PHBallJoint) double (PHJoint1D)
      // jd.limitSwing = ; // Vec2d (PHBallJointConeLimit)
      // jd.limitTwist = ; // Vec2d (PHBallJointConeLimit)
      jd.poseSocket.Pos() = Vec3d(0, o[0] - prev, 0);
      jd.poseSocket.Ori() = Quaterniond::Rot(Rad(0.0), 'y');
      jd.posePlug.Pos() = Vec3d(0, 0, 0);
      jd.posePlug.Ori() = Quaterniond::Rot(Rad(0.0), 'y');
      jois[i - 1] = phScene->CreateJoint(cvxs[i - 1], cvxs[i], jd)->Cast();
      prev = o[0];
      // PHIKEngineIf *ike = phScene->GetIKEngine();
      // ike->Enable(true);
      // ike->SetNumIter(20);
      // PHIKHingeActuatorDesc ad;
      // PHIKHingeActuatorIf *ika = phScene->CreateIKActuator(ad);
      // ika->AddChildObject(jois[i - 1]);
      // PHIKEndEffectorDesc ed;
      // PHIKEndEffectorIf *ikee = phScene->CreateIKEndEffector(ed);
      // ikee->AddChildObject(cvxs[i - 1]);
      // ika->AddChildObject(ikee);
    }
  }
  return cvxs[0];
}

void DispVertices(CDShapeIf *shapeCvx)
{
  CDConvexMeshIf *mesh = shapeCvx->Cast();
  fprintf(stdout, "ConvexMesh: Vertex %d\n", mesh->NVertex());
  fprintf(stdout, "ConvexMesh: Faces %d\n", mesh->NFace());
  for(int i = 0; i < mesh->NFace(); ++i){
    CDFaceIf *face = mesh->GetFace(i);
    fprintf(stdout, "ConvexMesh: Face[%d] Index %d\n", i, face->NIndex());
    int *indices = face->GetIndices();
    for(int j = 0; j < face->NIndex(); ++j){
      Vec3f v = mesh->GetVertices()[indices[j]];
      fprintf(stdout, "ConvexMesh: Face[%d] Vertex[%d] (%5.3f %5.3f %5.3f)\n",
        i, j, v.x, v.y, v.z);
    }
  }
}

PHSolidIf *CreateConvexMeshTetra(FWSdkIf *fwSdk)
{
  std::vector<Vec3f> vertices = {
    Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1)};
//  int indices[4][3] = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 2, 3}};
//  CDFaceDesc fd[4]; // not defined
//  for(int i = 0; i < 4; ++i) fd[i].indices = &indices[i];
//  CDFaceIf *faces[4] = new CDFaceIf *[4];
//  for(int i = 0; i < 4; ++i) faces[i] = fwSdk->GetPHSdk()->CreateShape(fd[i]);
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 0.033;
  PHSolidIf *cvx = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDConvexMeshDesc cmd;
  cmd.vertices = vertices;
//  cmd.faces = faces; // not defined
  cmd.material.density = 1.0;
  cmd.material.e = 1.0f;
  CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
  cvx->AddShape(shapeCvx);
  cvx->SetFramePosition(Vec3d(0, 2, 0));
  DispVertices(shapeCvx); // 4 - 4 - 3
  // 100 000 010, 100 010 001, 010 000 001, 000 100 001
  return cvx;
}

PHSolidIf *CreateConvexMeshCube(FWSdkIf *fwSdk)
{
  std::vector<Vec3f> vertices = {
    Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1),
    Vec3f(1, 1, 0), Vec3f(1, 0, 1), Vec3f(0, 1, 1), Vec3f(1, 1, 1)};
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 0.033;
  PHSolidIf *cvx = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDConvexMeshDesc cmd;
  cmd.vertices = vertices;
  cmd.material.density = 1.0;
  cmd.material.e = 1.0f;
  CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
  cvx->AddShape(shapeCvx);
  cvx->SetFramePosition(Vec3d(0, 2, 0));
  DispVertices(shapeCvx); // 8 - 12 - 3
  // 001 100 101, 110 000 010, 000 011 010, 101 110 111,
  // 110 011 111, 011 101 111, 011 000 001, 000 100 001,
  // 100 000 110, 011 001 101, 100 110 101, 011 110 010
  return cvx;
}

PHSolidIf *CreateBox(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 0.033;
  PHSolidIf *soBox = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDBoxDesc bd;
  bd.boxsize = Vec3f(2, 2, 2);
  CDShapeIf *shapeBox = fwSdk->GetPHSdk()->CreateShape(bd);
  soBox->AddShape(shapeBox);
  soBox->SetFramePosition(Vec3d(0, 10, 0));
  return soBox;
}

PHSolidIf *CreateSphere(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 0.03;
  PHSolidIf *soSphere = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDSphereDesc sd;
  sd.radius = 2.0;
  CDShapeIf *shapeSphere = fwSdk->GetPHSdk()->CreateShape(sd);
  soSphere->AddShape(shapeSphere);
  soSphere->SetFramePosition(Vec3d(0, 15, 0));
  return soSphere;
}

PHSolidIf *CreateCapsule(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 1;
  PHSolidIf *soCapsule = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDCapsuleDesc cd;
  cd.length = 3.0;
  cd.radius = 1.0;
  CDShapeIf *shapeCapsule = fwSdk->GetPHSdk()->CreateShape(cd);
  soCapsule->AddShape(shapeCapsule);
  soCapsule->SetFramePosition(Vec3d(0, 15, 0));
  return soCapsule;
}

PHSolidIf *CreateRoundCone(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 1;
  PHSolidIf *soRCone = fwSdk->GetScene()->GetPHScene()->CreateSolid(desc);
  CDRoundConeDesc rd;
  rd.length = 3.0;
  rd.radius[0] = 1.5;
  rd.radius[1] = 0.5;
  CDShapeIf *shapeRCone = fwSdk->GetPHSdk()->CreateShape(rd);
  soRCone->AddShape(shapeRCone);
  soRCone->SetFramePosition(Vec3d(0, 15, 0));
  return soRCone;
}

MyApp::MyApp() : FWApp()
{
  bDrawInfo = false;
}

MyApp::~MyApp()
{
}

void MyApp::Init(int ac, char **av)
{
//  FWApp::Init(ac, av); // skip default
/**/
  CreateSdk();
//  PHSdkIf *phSdk = GetSdk()->GetPHSdk();
  GetSdk()->CreateScene(); // phSdk->CreateScene(); // same ?
//  PHSceneIf *phScene = GetSdk()->GetScene()->GetPHScene(); // null pointer ?
  // SetGRAdaptee(TypeGLUT);
  GRInit(ac, av);
  FWWinDesc wd;
  wd.title = TEST_WORD;
  CreateWin(wd);
/**/
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
  FWApp::Display(); // use default
  FWSceneIf *fwScene = GetSdk()->GetScene();
  fwScene->EnableRenderAxis(bDrawInfo);
  fwScene->EnableRenderForce(bDrawInfo);
  fwScene->EnableRenderContact(bDrawInfo);
/*
//  GetSdk()->SetDebugMode(true);
  GetSdk()->GetRender()->SetViewMatrix(
    GetCurrentWin()->GetTrackball()->GetAffine().inv());
  // ??? // must clear renderer background here ?
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
  case '.':
    DSTR << "convexmeshpin" << std::endl;
    CreateConvexMeshPin(GetSdk());
    break;
  case '-':
    DSTR << "convexmeshcube" << std::endl;
    CreateConvexMeshCube(GetSdk());
    break;
  case '0':
    DSTR << "convexmeshtetra" << std::endl;
    CreateConvexMeshTetra(GetSdk());
    break;
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
  default: break;
  }
}

void MyApp::InitCameraView()
{
//  FWApp::InitCameraView(); // no method
  HITrackballIf *tb = GetCurrentWin()->GetTrackball();
/*
  std::istringstream issView(
    "((0.9996 0.0107463 -0.0261432 -0.389004)"
    "(-6.55577e-010 0.924909 0.380188 5.65711)"
    "(0.0282657 -0.380037 0.92454 13.7569)"
    "(     0      0      0      1))"
  );
  // tb->SetAffine(issView); // no method
  tb->xxx(issView.Rot());
  tb->xxx(issView.Trn());
*/
  tb->SetTarget(Vec3f(0.0f, 0.0f, 0.0f));
  tb->SetAngle(0.78f, 0.35f); // move camera by angle: look left pi/4 down pi/9
  tb->SetDistance(30.0f);
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
  phScene->SetTimeStep(0.050); // default == 0.005
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
  bd.boxsize = Vec3f(20.0f, 0.1f, 20.0f);
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

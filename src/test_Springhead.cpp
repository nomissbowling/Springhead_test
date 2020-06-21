/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <test_Springhead.h>

bool DBG = false; // true works without camera light etc
int W = 6; // number of windows
float PI = 3.14159265358979323846264338327950288419716939937510f;
float PNS = 0.2f; // scale
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
PHSolidIf *soBall_ref = NULL;

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

void DispSolidInf(PHSolidIf *sol)
{
  double m = sol->GetMass();
  Vec3d c = sol->GetCenterOfMass();
  Matrix3d i = sol->GetInertia();
  fprintf(stdout, "PHSolid: mass %7.3lf\n", m);
  fprintf(stdout, "PHSolid: center (%7.3lf, %7.3lf, %7.3lf)\n", c.x, c.y, c.z);
  fprintf(stdout, "PHSolid: inertia\n");
  fprintf(stdout, " (%7.3lf, %7.3lf, %7.3lf)\n", i[0][0], i[0][1], i[0][2]);
  fprintf(stdout, " (%7.3lf, %7.3lf, %7.3lf)\n", i[1][0], i[1][1], i[1][2]);
  fprintf(stdout, " (%7.3lf, %7.3lf, %7.3lf)\n", i[2][0], i[2][1], i[2][2]);
}

PHSolidIf **RotAllParts(PHSolidIf **so, int n, Quaterniond qt)
{
  Vec3d q;
  for(int i = 0; i < n; ++i){
    Vec3d p = so[i]->GetFramePosition();
    Vec3d np = qt * p;
    if(!i) q = p - np;
    so[i]->SetFramePosition(np); // rot (center is origin)
    so[i]->SetOrientation(qt); // rot (center is local center)
    so[i]->SetFramePosition(np + q); // relative point from so[0]
  }
  return so;
}

PHSolidIf *CreateConvexMeshPin(FWSdkIf *fwSdk, int c, Vec3d pos, float r)
{
  FWSceneIf *fwScene = fwSdk->GetScene(0);
  PHSceneIf *phScene = fwScene->GetPHScene();
  int t = 24; // 18; // 12;
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
        float th = 2.0f * PI * k / t;
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
    cmd.material.mu0 = 0.5f;
    cmd.material.mu = 0.3f;
    cmd.material.e = 0.3f; // 1.0f;
    // cmd.material.reflexSpring = ;
    // cmd.material.reflexDamper = ;
    // cmd.material.frictionSpring = ;
    // cmd.material.frictionDamper = ;
    CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
    cvxs[i]->SetMass(shapeCvx->CalcVolume() * shapeCvx->GetDensity());
    cvxs[i]->SetCenterOfMass(shapeCvx->CalcCenterOfMass());
    cvxs[i]->SetInertia(shapeCvx->CalcMomentOfInertia());
    cvxs[i]->AddShape(shapeCvx);
    cvxs[i]->SetFramePosition(pos + Vec3d(0.0, o[0], 0.0));
    fwScene->SetSolidMaterial(c, cvxs[i]);
    fwScene->SetWireMaterial(c, cvxs[i]);
//    DispVertices(shapeCvx);
//    DispSolidInf(cvxs[i]);
    // default
    //   0.050     0.152     0.361     0.316        // mass calclated cvxs[3-0]
    //   (0, 0, 0) (0, 0, 0) (0, 0, 0) (0, 0, 0)    // center of mass cvxs[3-0]
    //   3-0 all (0.033, 0, 0) (0, 0.033, 0) (0, 0, 0.033) // inertia cvxs[3-0]
    // after SetXXX()
    // 0.040 (0, 0.112, 0) (0.001, 0, 0) (0, 0.001, 0) (0, 0, 0.001) // cvxs[3]
    // 0.106 (0, 0.391, 0) (0.005, 0, 0) (0, 0.003, 0) (0, 0, 0.005) // cvxs[2]
    // 0.467 (0, 0.400, 0) (0.055, 0, 0) (0, 0.039, 0) (0, 0, 0.055) // cvxs[1]
    // 0.411 (0, 0.548, 0) (0.040, 0, 0) (0, 0.034, 0) (0, 0, 0.040) // cvxs[0]
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
      PH1DJointLimitDesc jld;
      jld.spring = 1000;
      jld.damper = 1000;
      jld.range = Vec2d(0, 0.001); // Vec2d (PH1DJointLimit)
      PHHingeJointIf *joint = jois[i - 1]->Cast();
      joint->CreateLimit(jld);
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
  phScene->SetContactMode(cvxs, l, PHSceneDesc::MODE_NONE); // parts of solids
//  RotAllParts(cvxs, l, Quaterniond::Rot(Rad(-90.0), 'x'));
  return cvxs[0];
}

PHSolidIf *CreatePinsTriangle(FWSdkIf *fwSdk, int c, Vec3d pos, float r)
{
/*
    0      j = 0  i = 0             0
   2 1         1      1, 2          0 1
  5 4 3        2      3, 4, 5       0 1 2
 9 8 7 6       3      6, 7, 8, 9    0 1 2 3
*/
  PHSolidIf *so = NULL;
  float pnp = 12.0f * r; // distance of each pin center point = 12 inch
  float pnq = pnp * sqrt(3.0f) / 2.0f;
  for(int k = 0, j = 0; j < 4; ++j)
    for(int i = 0; ++k, i < j + 1; ++i)
      so = CreateConvexMeshPin(fwSdk, c,
        pos * r + Vec3d(j * pnq, 0.0, i * pnp - j * pnp / 2.0), r);
  return so;
}

PHSolidIf *CreateBall(FWSdkIf *fwSdk, int c, Vec3d pos, float rad, float r)
{
  PHSolidDesc desc;
  desc.mass = 1.0;
  desc.inertia *= 0.03;
  PHSolidIf *so = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDSphereDesc sd;
  sd.radius = rad * r;
  sd.material.mu0 = 0.5f;
  sd.material.mu = 0.4f;
  sd.material.e = 0.4f;
  CDShapeIf *shapeSphere = fwSdk->GetPHSdk()->CreateShape(sd);
  so->AddShape(shapeSphere);
  so->SetCenterPosition(pos * r + Vec3d(0.0, sd.radius, 0.0));
  fwSdk->GetScene(0)->SetSolidMaterial(c, so);
  fwSdk->GetScene(0)->SetWireMaterial(c, so);
  return so;
}

PHSolidIf *CreatePlane(FWSdkIf *fwSdk, int c, Vec3d pos, Vec3f sz, float r,
  bool dyn=false)
{
  PHSolidDesc desc;
  desc.mass = 1000.0;
  desc.inertia *= 0.033;
  PHSolidIf *soPlane = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  soPlane->SetDynamical(dyn);
  soPlane->SetMass(1000.0);
  CDBoxDesc bd;
  bd.boxsize = sz * r;
  bd.material.mu0 = 0.5f;
  bd.material.mu = 0.2f;
  bd.material.e = 0.2f;
  CDShapeIf *shapePlane = fwSdk->GetPHSdk()->CreateShape(bd);
  soPlane->AddShape(shapePlane);
  soPlane->SetCenterPosition(pos * r);
  fwSdk->GetScene(0)->SetSolidMaterial(c, soPlane);
  fwSdk->GetScene(0)->SetWireMaterial(c, soPlane);
  return soPlane;
}

PHSolidIf *CreateHalfPipe(FWSdkIf *fwSdk, int c, Vec3d pos, Vec3f si, float r)
{
  PHSolidIf *so = NULL;
  // si.x (length) si.y (thickness) si.z (radius)
  float sr = si.z * r;
  for(int i = -1; i <= 1; ++i){
    so = CreatePlane(fwSdk, c, pos, si, r);
    Vec3d p = so->GetCenterPosition();
    double a = i * 60.0; // 60.0: -1 -> 1, 36.0: -2 -> 2
    double th = a * PI / 180.0;
    so->SetOrientation(Quaterniond::Rot(Rad(a), 'x'));
    so->SetCenterPosition(Vec3d(p.x, p.y - sr * cos(th), p.z - sr * sin(th)));
  }
  return so;
}

PHSolidIf *CreateLane(FWSdkIf *fwSdk, Vec3d pos, float r)
{
/*
pdr = 7.5 + 0.17 feet
last = 34+3/16 inch /12 -> 2.849 feet
triangle = 12 inch * 3 (center of pin axis) /12 -> 3 feet
triangle lane = 31+3/16 inch /12 -> 2.600 feet = psd
psd = 2.600 feet < 2.9949 feet < (lnw x root(3) / 2)
apd = 15 + 0.35 feet
lnd = 60 feet
lnw = 41.5 inch /12 -> 3.4583... feet
gtw = 9.25 inch /12 -> 0.77083... feet
pindrop (lnd, -h, 0) pdr lnw+gtw*2
pinspot (lnd, 0, 0) psd lnw
lane (0, 0, 0) lnd lnw
approach (-apd, 0, 0) apd lnw+gtw*2
ball r = 4.25 inch /12 -> 0.35416... feet diameter 8.5 inch 5-16 pounds
*/
  float feetinch = 12.0f;
  float ballr = 8.5f / 2.0f;
  float apd = 15.35f * feetinch;
  float psd = 2.6f * feetinch, lst = 2.849f * feetinch, pdr = 7.67f * feetinch;
  float aph = apd / 2.0f, lsh = lst / 2.0f;
  float lnh = 1.0f, lnw = 41.5f, lnd = 60.0f * feetinch;
  Vec3f gtsi = Vec3f(lnd + lst, lnh, (ballr * 2.0f + 0.75f) / 2.0f);
  float gtp = lnw / 2.0f + gtsi.z;
  CreateHalfPipe(fwSdk, GRRenderBaseIf::LIMEGREEN,
    Vec3d(pos.x + lsh, pos.y, pos.z - gtp), gtsi, r);
  CreateHalfPipe(fwSdk, GRRenderBaseIf::LIMEGREEN,
    Vec3d(pos.x + lsh, pos.y, pos.z + gtp), gtsi, r);
  PHSolidIf *soLast = CreatePlane(fwSdk, GRRenderBaseIf::DARKSALMON,
    Vec3d(pos.x + lnd / 2.0f + lsh, pos.y, pos.z), Vec3f(lst, lnh, lnw), r);
  PHSolidIf *soAppr = CreatePlane(fwSdk, GRRenderBaseIf::SALMON,
    Vec3d(pos.x - lnd / 2.0f - aph, pos.y, pos.z), Vec3f(apd, lnh, lnw), r);
  PHSolidIf *soLane = CreatePlane(fwSdk, GRRenderBaseIf::LIGHTSALMON,
    pos, Vec3f(lnd, lnh, lnw), r);
  PHSolidIf *soPins = CreatePinsTriangle(fwSdk, GRRenderBaseIf::GOLD,
    pos + Vec3d(lnd / 2.0f, lnh / 2.0f, 0.0), r);
/*
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(-lnd / 2.0f, lnh / 2.0f, lnw / 39.0f * 3.2f), ballr, r);
  soBall->SetMass(0.9);
  soBall->SetVelocity(Vec3d(lnd * r / 2.4, 0.0, 0.0));
  soBall->SetAngularVelocity(Vec3d(-5.0, 2.0, -5.0));
*/
/* // hooking point 50ft ? 7pin only or left gutter
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.9);
  soBall->SetVelocity(Vec3d(lnd * r / 6.0 / 2.4, 0.0, -38.0 * r / 2.4));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -17.5 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? 10pin tap
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.0 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? just ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.4 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? 9pin tap
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.5 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
// hooking point 40ft ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 6.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), ballr, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 4.8, 0.0, 3.0 * -19.3 * r / 4.8));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));

  soBall_ref = soBall;
  return soLane;
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
  PHSolidIf *cvx = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDConvexMeshDesc cmd;
  cmd.vertices = vertices;
//  cmd.faces = faces; // not defined
  cmd.material.density = 1.0;
  cmd.material.e = 1.0f;
  CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
  cvx->AddShape(shapeCvx);
  cvx->SetFramePosition(Vec3d(0, 2, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::PLUM, cvx);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::PLUM, cvx);
//  DispVertices(shapeCvx); // 4 - 4 - 3
  // 100 000 010, 100 010 001, 010 000 001, 000 100 001

  GRSceneIf *grScene = fwSdk->GetScene(0)->GetGRScene();
  GRFrameDesc frmd;
  //frmd.transform = Affinef();
#if 0
  GRFrameIf *frm = grScene->CreateVisual(frmd, grScene->GetWorld())->Cast();
#else
  GRFrameIf *frm = grScene->CreateVisual(frmd)->Cast(); // parent = world
#endif
  GRMeshDesc meshd; // SprGRMesh.h SprGRFrame.h SprCDShape.h
  meshd.vertices = vertices;
/*
  meshd.faces = vector<GRMeshFace>{}; // {int nVertices=(3or4), int indices[4]}
  //meshd.normals = vector<Vec3f>{};
  //meshd.faceNormals = vector<GRMeshFace>{};
  meshd.colors = vector<Vec4f>{};
  meshd.texCoords = vector<Vec2f>{};
  meshd.materialList = vector<int>{};
*/
  GRMeshIf *mesh = grScene->CreateVisual(meshd, frm)->Cast();
  GRMaterialDesc matd(
    Vec4f(0.9f, 0.8f, 0.2f, 1.0f), // ambient
    Vec4f(0.6f, 0.6f, 0.6f, 1.0f), // diffuse
    Vec4f(0.2f, 0.2f, 0.2f, 1.0f), // specular
    Vec4f(0.8f, 0.6f, 0.4f, 1.0f), // emissive
    10.0); // power
  matd.texname = "texname";
  GRMaterialIf *mat = grScene->CreateVisual(matd, frm)->Cast();
  mesh->AddChildObject(mat); // 0 -> meshd.materialList[0]
//  mesh->AddChildObject(mat); // 1 -> meshd.materialList[1]
//  mesh->AddChildObject(mat); // 2 -> meshd.materialList[2]
//  grRender->SetMaterial(mat);
  FWObjectIf *fwObj = fwSdk->GetScene(0)->CreateFWObject();
  fwObj->SetPHSolid(cvx);
  fwObj->SetGRFrame(frm);
  fwSdk->GetScene(0)->Sync(); // can not set true ?

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
  PHSolidIf *cvx = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDConvexMeshDesc cmd;
  cmd.vertices = vertices;
  cmd.material.density = 1.0;
  cmd.material.e = 1.0f;
  CDShapeIf *shapeCvx = fwSdk->GetPHSdk()->CreateShape(cmd);
  cvx->AddShape(shapeCvx);
  cvx->SetFramePosition(Vec3d(0, 2, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::DEEPSKYBLUE, cvx);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::DEEPSKYBLUE, cvx);
//  DispVertices(shapeCvx); // 8 - 12 - 3
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
  PHSolidIf *soBox = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDBoxDesc bd;
  bd.boxsize = Vec3f(2, 2, 2);
  CDShapeIf *shapeBox = fwSdk->GetPHSdk()->CreateShape(bd);
  soBox->AddShape(shapeBox);
  soBox->SetFramePosition(Vec3d(0, 10, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::DODGERBLUE, soBox);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::DODGERBLUE, soBox);
  return soBox;
}

PHSolidIf *CreateSphere(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 0.03;
  PHSolidIf *soSphere = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDSphereDesc sd;
  sd.radius = 2.0;
  CDShapeIf *shapeSphere = fwSdk->GetPHSdk()->CreateShape(sd);
  soSphere->AddShape(shapeSphere);
  soSphere->SetFramePosition(Vec3d(0, 15, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::SEAGREEN, soSphere);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::SEAGREEN, soSphere);
  return soSphere;
}

PHSolidIf *CreateCapsule(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 1;
  PHSolidIf *soCapsule = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDCapsuleDesc cd;
  cd.length = 3.0;
  cd.radius = 1.0;
  CDShapeIf *shapeCapsule = fwSdk->GetPHSdk()->CreateShape(cd);
  soCapsule->AddShape(shapeCapsule);
  soCapsule->SetFramePosition(Vec3d(0, 15, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::SPRINGGREEN, soCapsule);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::SPRINGGREEN, soCapsule);
  return soCapsule;
}

PHSolidIf *CreateRoundCone(FWSdkIf *fwSdk)
{
  PHSolidDesc desc;
  desc.mass = 0.05;
  desc.inertia *= 1;
  PHSolidIf *soRCone = fwSdk->GetScene(0)->GetPHScene()->CreateSolid(desc);
  CDRoundConeDesc rd;
  rd.length = 3.0;
  rd.radius[0] = 1.5;
  rd.radius[1] = 0.5;
  CDShapeIf *shapeRCone = fwSdk->GetPHSdk()->CreateShape(rd);
  soRCone->AddShape(shapeRCone);
  soRCone->SetFramePosition(Vec3d(0, 15, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::PALEGREEN, soRCone);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::PALEGREEN, soRCone);
  return soRCone;
}

MyApp::MyApp() : FWApp()
{
  bDrawInfo = false;
  tick = 0;
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
  // SetGRAdaptee(TypeGLUT); // VirtualHuman::TypeGLUT
  GRInit(ac, av);

  GetSdk()->CreateScene(); // phSdk->CreateScene(); // same ?
//  PHSceneIf *phScene = GetSdk()->GetScene(0)->GetPHScene(); // null pointer ?

/*
//  GRSdkIf *grSdk = GRSdkIf::CreateSdk();
  GRSdkIf *grSdk = GetSdk()->GetGRSdk();
//  GRSceneIf *grScene = grSdk->CreateScene();
//  GRSceneIf *grScene = GetSdk()->GetScene(0)->GetGRScene();
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

  MyWinDescPart wdp[] = { // (title bar height = 32 depends on system env)
    {640, 480, 120, 480 + 32 + 4, WIN_TITLE, false, DBG},
    {640, 480, 120, 0, WIN_UP, false, DBG},
    {640, 480, 120 + 640 + 4, 0, WIN_PINTOP, false, DBG},
    {640, 480, 120 + 640 + 4, 480 + 32 + 4, WIN_SIDE, false, DBG},
    {480, 360, 120 + (640 + 4) * 2, 0, WIN_BALL, false, DBG},
    {480, 360, 120 + (640 + 4) * 2, 360 + 32 + 4, WIN_DEBUG, false, true}};
  for(int i = 0; i < sizeof(wdp) / sizeof(wdp[0]); ++i){
    FWWinDesc wd;
    wd.title = wdp[i].title;
    wd.width = wdp[i].width, wd.height = wdp[i].height;
    wd.left = wdp[i].left, wd.top = wdp[i].top;
    wd.fullscreen = wdp[i].fullscreen, wd.debugMode = wdp[i].debugMode;
    CreateWin(wd);
    GetWin(i)->GetTrackball()->SetPosition(Vec3f(0.0, 0.0, 0.0));
    GetWin(i)->SetScene(GetSdk()->GetScene(0)); // all Win to Scene 0
  }
  SetCurrentWin(GetWin(0));

  fprintf(stdout, "Scenes: %d\n", GetSdk()->NScene());
  for(int i = 0; i < GetSdk()->NScene(); ++i){
    FWSceneIf *fwScene = GetSdk()->GetScene(i);
    fwScene->SetRenderMode(true, false); // solid
//    fwScene->SetRenderMode(false, true); // wire
    //fwScene->EnableRenderAxis();
    //fwScene->EnableRenderForce();
    //fwScene->EnableRenderContact();
    //fwScene->EnableRenderGrid();
  }

/**/
  InitCameraView();
  CreateCameras(); // reset cameras in Display() rendering each scene ?
  CreateLights();
  CreateObjects();
  CreateTimer();
}

void MyApp::TimerFunc(int id)
{
  ++tick;
//  FWApp::TimerFunc(id); // skip default
  GetSdk()->Step();
//  for(int i = 0; i < W; ++i) GetWin(i)->GetScene()->Step(); // speed x W
  PostRedisplay();
}

void MyApp::Display()
{
//  FWApp::Display(); // skip default
//  GetCurrentWin()->Display();
//  for(int i = 0; i < W; ++i) GetWin(i)->Display(); // (all Scene in all Win)
  for(int i = 0; i < W; ++i){
    FWWinIf *w = GetWin(i);
    SetCurrentWin(w);
//    w->Display();
    GRRenderIf *grRender = w->GetRender();
    grRender->BeginScene();
    grRender->ClearBuffer(true, true);
/**/
  if(i == W - 2){
    GRCameraDesc camd = grRender->GetCamera();
    //camd.size = Vec2f(0.2f, 0.0f);
    //camd.center = Vec2f();
    camd.front = 0.1f + tick / 100.0f; // default 0.1f;
    if(camd.front > 1.0f) camd.front = 1.0f;
    //camd.back = 500.0f;
    //camd.type = GRCameraDesc::PERSPECTIVE;
    grRender->SetCamera(camd);
  }
/**/
    GRMaterialDesc matd(
      Vec4f(0.9f, 0.8f, 0.2f, 1.0f), // ambient
      Vec4f(0.6f, 0.6f, 0.6f, 1.0f), // diffuse
      Vec4f(0.2f, 0.2f, 0.2f, 1.0f), // specular
      Vec4f(0.8f, 0.6f, 0.4f, 1.0f), // emissive
      10.0); // power
    grRender->SetMaterial(matd);
//    grRender->SetMaterial(GRRenderBaseIf::WHITE);
    grRender->SetViewMatrix(w->GetTrackball()->GetAffine().inv());
    FWSceneIf *fwScene = w->GetScene();
    if(i < W - 1) fwScene->SetRenderMode(true, false); // solid
    else fwScene->SetRenderMode(false, true); // wire
    if(DBG){
      fwScene->EnableRenderAxis(bDrawInfo);
      fwScene->EnableRenderForce(bDrawInfo);
      fwScene->EnableRenderContact(bDrawInfo);
      //fwScene->EnableRenderGrid(bDrawInfo);
    }
    //fwScene->Draw(grRender, w->GetDebugMode()); // shown when wireframe mode
    //fwScene->Draw(grRender, true); // force true PH only
    //fwScene->Draw(grRender, false); // force false GR only
    fwScene->Draw(grRender); // normal (not set debug=false) see FWScene.cpp
if(1){
  grRender->SetLighting(false);
  grRender->SetDepthTest(false);
  grRender->EnterScreenCoordinate();
  Vec2i scr = w->GetSize();
  Vec2i sch = scr / 2;
  Vec2i chr = Vec2i(8, 12); // font (left bottom) place left top (0 0)
/*
  GRFont ft;
  ft.height = chr.y;
  ft.width = chr.x;
  //ft.weight = ;
  ft.color = 0xFFCC33;
  //ft.bItalic = true;
  ft.face = "Verdana"; // > (8, 12) ?
  grRender->SetFont(ft); // calling SetFont() effects to Win(0) only ?
*/
  grRender->DrawFont(Vec2f(0.0f, chr.y), "ABC"); // left top 1st line
  grRender->DrawFont(Vec2f(0.0f, chr.y * 2), "DEF"); // left 2nd line
  grRender->DrawFont(Vec2f(0.0f, scr.y), "XXXX"); // left bottom
  grRender->DrawFont(Vec2f(scr.x - chr.x * 3, chr.y), "YYY"); // right top
  grRender->DrawFont(Vec2f(scr.x - chr.x * 2, scr.y), "ZZ"); // right bottom
  char s[128];
  int nobjects = fwScene->NObject();
  int nsolids = fwScene->GetPHScene()->NSolids();
  GRFrameIf *world = fwScene->GetGRScene()->GetWorld();
  int nchildren = world->NChildren();
  sprintf_s(s, sizeof(s), "FWObjects:%4d, PHSolids:%4d, GRWorldChildren:%4d",
    nobjects, nsolids, nchildren);
  grRender->DrawFont(Vec2f(sch.x - chr.x * 26, sch.y - chr.y * 14), s);
  Posed po = soBall_ref->GetPose();
  Vec3d p = po.Pos();
  sprintf_s(s, sizeof(s), "(%7.3f %7.3f %7.3f)", p.x, p.y, p.z);
  grRender->DrawFont(Vec2f(sch.x - chr.x * 12, sch.y - chr.y * 12), s);
  Quaterniond q = po.Ori();
//  Matrix3d m;
//  q.ToMatrix(m);
  Vec3d a = q.V(); // q.Axis(); // template TVec3<double> compile error
  double angle = q.Theta();
  sprintf_s(s, sizeof(s), "%7.3f (%7.3f %7.3f %7.3f)", angle, a.x, a.y, a.z);
  grRender->DrawFont(Vec2f(sch.x - chr.x * 12, sch.y - chr.y * 11), s);
  sprintf_s(s, sizeof(s), "(%7.3f %7.3f %7.3f %7.3f)", q.w, q.x, q.y, q.z);
  grRender->DrawFont(Vec2f(sch.x - chr.x * 12, sch.y - chr.y * 10), s);
  double cs = cos(angle/2); // == q.w (always same value) (2acos(q.w) == angle)
  double ss = sin(angle/2);
  sprintf_s(s, sizeof(s), "%7.3f [%7.3f %7.3f %7.3f]",
    2 * acos(q.w), q.x / ss, q.y / ss, q.z / ss);
  grRender->DrawFont(Vec2f(sch.x - chr.x * 12, sch.y - chr.y * 8), s);
  grRender->LeaveScreenCoordinate();
  grRender->SetDepthTest(true);
  grRender->SetLighting(true);
}
    grRender->EndScene();
    grRender->SwapBuffers();
  }
  SetCurrentWin(GetWin(0));
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
    CreateConvexMeshPin(GetSdk(), GRRenderBaseIf::GOLD, Vec3d(0, 5, 0), 0.2f);
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
/*
  HITrackballIf *tb = GetCurrentWin()->GetTrackball();
  std::istringstream issView(
    "((0.9996 0.0107463 -0.0261432 -0.389004)"
    "(-6.55577e-010 0.924909 0.380188 5.65711)"
    "(0.0282657 -0.380037 0.92454 13.7569)"
    "(     0      0      0      1))"
  );
  // tb->SetAffine(issView); // no method
  tb->xxx(issView.Rot());
  tb->xxx(issView.Trn());
//tb->SetAngle(0.78f, 0.35f); // move camera by angle: look left pi/4 down pi/9
//tb->SetAngle(1.05f, 0.26f); // pi/3 (-pi<=lng<=pi), pi/12 (-pi/2<=lat<=pi/2)
//tb->SetAngle(1.31f, 0.09f); // 5pi/12, pi/36
//tb->SetDistance(30.0f);
*/

  float lnd = 60.0f * 12.0f * PNS;
  MyCameraDescPart cdp[] = {
    {-lnd / 4.0f, 0.0f, 0.0f, -PI / 2.0f, 3.0f * PI / 8.0f, lnd * 4.0f / 5.0f},
    {lnd / 2.0f, 0.0f, 0.0f, -PI / 2.0f, PI / 36.0f, 20.0f},
    {lnd / 2.0f, 0.0f, 0.0f, -PI / 2.0f, PI / 2.0f, 20.0f},
    {0.0f, 0.0f, 0.0f, 17.0f * PI / 36.0f, PI / 36.0f, 95.0f}, // <= max 99
    {-lnd / 4.0f, 0.0f, 0.0f, -PI / 2.0f, 0.0f, lnd * 3.0f / 10.0f},
    {0.0f, 0.0f, 0.0f, -PI / 3.0f, PI / 9.0f, 10.0f}};
  for(int i = 0; i < sizeof(cdp) / sizeof(cdp[0]); ++i){
    HITrackballIf *tb = GetWin(i)->GetTrackball();
    tb->SetTarget(Vec3f(cdp[i].x, cdp[i].y, cdp[i].z));
    tb->SetAngle(cdp[i].lng, cdp[i].lat);
    tb->SetDistance(cdp[i].r);
  }
}

void MyApp::CreateCameras()
{
  GRSdkIf *grSdk = GetSdk()->GetGRSdk();
  GRSceneIf *grScene = grSdk->GetScene(0);

  const char *src[] = {"desc.transform", "GetTransform", "GetWorldTransform"};
  GRFrameIf *frm = grScene->GetWorld();
  GRFrameDesc frmd;
  frm->GetDesc(&frmd);
  for(int i = 0; i < 3; ++i){
    Affinef t;
    if(!i) t = frmd.transform; // 3x3 Identity
    else if(i == 1) t = frm->GetTransform(); // 3x3 Identity
    else t = frm->GetWorldTransform(); // 3x3 Identity
    fprintf(stdout, "%s:\n", src[i]);
    fprintf(stdout, " (%7.3f %7.3f %7.3f)\n", t[0][0], t[0][1], t[0][2]);
    fprintf(stdout, " (%7.3f %7.3f %7.3f)\n", t[1][0], t[1][1], t[1][2]);
    fprintf(stdout, " (%7.3f %7.3f %7.3f)\n", t[2][0], t[2][1], t[2][2]);
  }
// frm->SetTransform(Affinef::Trn(3.0, 0.0, 0.0)); // Trn(3.0, 1.0, 1.0) ?

  GRCameraDesc camd;
  camd.size = Vec2f(0.2f, 0.0f);
  camd.center = Vec2f();
  camd.front = 0.1f;
  camd.back = 500.0f;
  camd.type = GRCameraDesc::PERSPECTIVE;
//  GRCameraDesc camd = GetSdk()->GetRender()->GetCamera(); // get OK but black
//  grScene->SetCamera(camd);
  for(int i = 0; i < W; ++i){
    SetCurrentWin(GetWin(i));
    GRRenderIf *grRender = GetCurrentWin()->GetRender();
    grRender->SetCamera(camd);
  }
  SetCurrentWin(GetWin(0));
}

void MyApp::CreateLights()
{
  for(int i = 0; i < W; ++i){
    SetCurrentWin(GetWin(i));
    GRRenderIf *grRender = GetCurrentWin()->GetRender();
    GRLightDesc lightd;
    lightd.ambient = Vec4f(1, 1, 1, 1) * 0.6f;
    lightd.diffuse = Vec4f(1, 1, 1, 1) * 0.6f;
    lightd.specular = Vec4f(1, 1, 1, 1) * 0.6f;
    lightd.position = Vec4f(0, 50.0f, 0, 0); // (x, y, z, w(0:parallel,1:spot))
    grRender->SetLighting(true);
    grRender->PushLight(lightd);
/*
    // grRender->SetDepthWrite(true);
    // grRender->SetDepthTest(true);
    // grRender->SetDepthFunc(...);
    // grRender->SetAlphaTest(true);
    // grRender->SetAlphaMode(..., ...);
    grRender->SetLighting(true);
    grRender->PushLight(lightd);
    grRender->SetCamera(camd);
*/
  }
  SetCurrentWin(GetWin(0));
}

void MyApp::CreateObjects()
{
//  FWApp::CreateObjects(); // no method
//  PHSdkIf *phSdk = PHSdkIf::CreateSdk(); // singleton created by Framework
  PHSdkIf *phSdk = GetSdk()->GetPHSdk();
//  PHSceneIf *phScene = phSdk->CreateScene(); // create multi instance OK
  PHSceneIf *phScene = GetSdk()->GetScene(0)->GetPHScene();
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
  phScene->SetContactMode(PHSceneDesc::MODE_LCP); // all default == MODE_LCP
//  GetSdk()->SetDebugMode(DBG); // true works without camera light etc

/*
//  GRSdkIf *grSdk = GRSdkIf::CreateSdk();
  GRSdkIf *grSdk = GetSdk()->GetGRSdk();
//  GRSceneIf *grScene = grSdk->CreateScene();
//  GRSceneIf *grScene = GetSdk()->GetScene(0)->GetGRScene();
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
/*
  Quaterniond q = Quaterniond::Rot(Rad(45.0), Vec3d(1, 1, 1));
//  Matrix3d rot;
//  q.ToMatrix(rot);
  CDBoxDesc bd;
  PHSolidDesc sd;
  sd.mass = 1.0;

  PHSolidIf *floor = phScene->CreateSolid();
  floor->SetDynamical(false);
  floor->SetMass(10000.0);
  bd.boxsize = Vec3f(20.0f, 0.1f, 20.0f);
  floor->AddShape(phSdk->CreateShape(bd));
  floor->SetFramePosition(Vec3d(0, -1.0, 0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::HOTPINK, floor);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::HOTPINK, floor);

  PHSolidIf *box = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  box->AddShape(phSdk->CreateShape(bd));
  box->SetCenterPosition(Vec3d(0.0, 1.0, 0.0));
  box->SetVelocity(Vec3d(0.0, 0.7, 0.0));
  box->SetOrientation(q);
//  box->AddTorque(-Vec3d(1.0, 1.0, 5.0));
  box->AddForce(-Vec3d(0.0, 0.0, -5.0), Vec3d(0.15, 0.85, 0.0));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::BLUE, box);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::BLUE, box);

  PHSolidIf *sol = phScene->CreateSolid(sd); // sol->SetMass(5.0); // etc
  bd.boxsize = Vec3f(0.5f, 0.3f, 0.3f);
  sol->AddShape(phSdk->CreateShape(bd));
  sol->SetCenterPosition(Vec3d(0.0, 1.5, 0.0));
  sol->SetVelocity(Vec3d(0.0, 0.7, 0.0));
  sol->SetAngularVelocity(-Vec3d(0.5, 0.5, 0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::ORANGERED, sol);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::ORANGERED, sol);

  PHSolidIf *sol0 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol0->AddShape(phSdk->CreateShape(bd));
  sol0->SetCenterPosition(Vec3d(0.5, 0.5, -0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::CYAN, sol0);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::CYAN, sol0);
  PHSolidIf *sol1 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol1->AddShape(phSdk->CreateShape(bd));
  sol1->SetCenterPosition(Vec3d(0.5, 0.5, 0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::MAGENTA, sol1);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::MAGENTA, sol1);
  PHHingeJointDesc hjd;
  hjd.poseSocket.Pos() = Vec3d(1.0, 0.0, 0.0);
  hjd.posePlug.Pos() = Vec3d(-1.0, 0.0, 0.0);
  PHHingeJointIf *joint = phScene->CreateJoint(sol0, sol1, hjd)->Cast();
*/
  CreateLane(GetSdk(), Vec3d(0.0, -20.0, 0.0), PNS);
}

void MyApp::Reset()
{
  tick = 0;
//  FWApp::Reset(); // skip default
  GetSdk()->GetScene(0)->GetPHScene()->Clear();
  CreateObjects();
}

int main(int ac, char **av)
{
  fprintf(stdout, "sizeof(size_t): %zd\n", sizeof(size_t));
  fprintf(stdout, "%s\n", WIN_TITLE);
  MyApp app; // not catch exception AtExit() when declared at global
  app.Init(ac, av);
  app.StartMainLoop();
  return 0;
}

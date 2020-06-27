/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <test_Springhead.h>

bool DBG = false; // true works without camera light etc
const int W = 6; // number of windows
int WinID[W]; // ID
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
float BALL_R = 8.5f / 2.0f;
PHSolidIf *soBall_ref = NULL;
GRMaterialDesc mat_desc = GRMaterialDesc( // common
  Vec4f(0.8f, 0.8f, 0.8f, 1.0f), // ambient
  Vec4f(0.6f, 0.6f, 0.6f, 1.0f), // diffuse
  Vec4f(0.2f, 0.2f, 0.2f, 1.0f), // specular
  Vec4f(0.5f, 0.5f, 0.5f, 1.0f), // emissive
  10.0); // power
GRFrameDesc frm_desc = GRFrameDesc(); // common // .transform = Affinef();

void DispMeshDesc(GRMeshDesc &meshd)
{
  fprintf(stdout, "vertices: %zu\n", meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i){
    Vec3f v = meshd.vertices[i];
    fprintf(stdout, " %4d: %7.3f, %7.3f, %7.3f\n", i, v.x, v.y, v.z);
  }
  fprintf(stdout, "texCoords: %zu\n", meshd.texCoords.size());
  for(int i = 0; i < meshd.texCoords.size(); ++i){
    Vec2f v = meshd.texCoords[i];
    fprintf(stdout, " %4d: %7.3f, %7.3f\n", i, v.x, v.y);
  }
  fprintf(stdout, "colors: %zu\n", meshd.colors.size());
  for(int i = 0; i < meshd.colors.size(); ++i){
    Vec4f v = meshd.colors[i];
    fprintf(stdout, " %4d: %7.3f, %7.3f, %7.3f, %7.3f\n", i,
      v[0], v[1], v[2], v[3]);
  }
  fprintf(stdout, "faces: %zu\n", meshd.faces.size());
  for(int i = 0; i < meshd.faces.size(); ++i){
    GRMeshFace v = meshd.faces[i];
    fprintf(stdout, " %4d: %d: %3d %3d %3d %3d\n", i, v.nVertices,
      v.indices[0], v.indices[1], v.indices[2], v.indices[3]);
  }
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

void CreateCylinderMesh(GRMeshDesc &meshd, float r, float a, float b,
  float p[][2], int m, int t)
{
  meshd.vertices = std::vector<Vec3f>((m + 2) * (t + 1) - 2);
  meshd.texCoords = std::vector<Vec2f>(meshd.vertices.size());
  meshd.faces = std::vector<GRMeshFace>(m * t * 2); // {int nVertices=3|4, int indices[4]}
  for(int j = 0; j < m; ++j){
    for(int k = 0; k < t; ++k){
      float p0 = p[m - j - 1][0] * r, p1 = p[m - j - 1][1] * r / 2.0f;
      float th = 2.0f * PI * k / t;
      int f = (j + 1) * (t + 1) + k - 1;
      meshd.vertices[f] = Vec3f(p1 * sin(th), p0, p1 * cos(th));
      meshd.texCoords[f] = Vec2f((t - k) / (float)t, 1 - (p0-a) / ((b-a) * r));
      if(!k){
        int g = (j + 1) * (t + 1) + t - 1;
        meshd.vertices[g] = meshd.vertices[f];
        meshd.texCoords[g] = Vec2f(0.0f, meshd.texCoords[f].y);
      }
      if(j < m - 1){
        meshd.faces[j * t * 2 + t + k] = GRMeshFace{3, {f, f+1, f+1+t+1}};
        meshd.faces[(j + 1) * t * 2 + k] = GRMeshFace{3, {f+1+t+1, f+t+1, f}};
      }
      if(!j){
        meshd.vertices[k] = Vec3f(0.0f, a * r, 0.0f);
        meshd.texCoords[k] = Vec2f((2 * (t - k) - 1) / (2.0f * t), 1.0f);
        meshd.faces[k] = GRMeshFace{3, {k, f+1, f}};
      }else if(j == m - 1){
        int h = (m + 1) * (t + 1) + k - 1;
        meshd.vertices[h] = Vec3f(0.0f, b * r, 0.0f);
        meshd.texCoords[h] = Vec2f((2 * (t - k) - 1) / (2.0f * t), 0.0f);
        meshd.faces[j * t * 2 + t + k] = GRMeshFace{3, {h, f, f+1}};
      }
    }
  }
}

void CreateSphereMesh(GRMeshDesc &meshd, float r, float radius, int sl, int st)
{
  int t = sl;
  int m = st - 1;
  float c = radius;
  float (*q)[2] = new float[m][2];
  for(int j = 0; j < m; ++j){
    float ph = PI * (j + 1) / st;
    q[m - j - 1][0] = radius * (1 - cos(ph)) - c;
    q[m - j - 1][1] = radius * sin(ph) * 2.0f;
  }
  CreateCylinderMesh(meshd, r, q[m - 1][0], q[0][0], q, m, t);
  delete[] q;
}

void CreateFaceMesh(GRMeshDesc &meshd, float r, Vec3f sz,
  std::vector<Vec3f> &vertices, std::vector<GRMeshFace> &faces)
{
  std::vector<Vec2f> coords = std::vector<Vec2f>{
    {1, 0}, {1, 1}, {0, 1}, {0, 0}}; // 4 or 3
  int nv = faces[0].nVertices; // face: number of vertices = 4 or 3
  int nf = (int)faces.size(); // cube: number of faces = 6 or 4 etc
  meshd.vertices = std::vector<Vec3f>(nv * nf);
  meshd.faces = std::vector<GRMeshFace>(nf); // {int nVertices=3|4, int indices[4]}
  meshd.texCoords = std::vector<Vec2f>(meshd.vertices.size());
  for(int i = 0; i < nf; ++i){
    GRMeshFace g = GRMeshFace{nv, {0, 0, 0, 0}};
    GRMeshFace &f = faces[i];
    for(int j = 0; j < nv; ++j){
      g.indices[j] = i * nv + j;
      meshd.vertices[g.indices[j]] = vertices[f.indices[j]];
      meshd.texCoords[g.indices[j]] = coords[j];
    }
    meshd.faces[i] = g;
  }
  //meshd.normals = std::vector<Vec3f>{};
  //meshd.faceNormals = std::vector<GRMeshFace>{};
  //meshd.colors = std::vector<Vec4f>{};
  //meshd.texCoords = std::vector<Vec2f>{};
  //meshd.materialList = std::vector<int>{0}; // abort XCastPtr SprObject.h:43
}

void CreateBoxMesh(GRMeshDesc &meshd, float r, Vec3f sz, CDShapeIf *shape)
{
  CDBoxIf *box = shape->Cast();
  Vec3f *vtxs = box->GetVertices(); // not found box->GetVtxCount()
  std::vector<Vec3f> vertices = std::vector<Vec3f>(8);
  for(int i = 0; i < vertices.size(); ++i) vertices[i] = vtxs[i];
  // CDFaceIf *face = box->GetFace(0); // always NULL ! not found box->NFace()
  std::vector<GRMeshFace> faces = std::vector<GRMeshFace>{
    // CDBox vertices order is NOT same with ConvexMeshCube
    // replace CDBox vertices[73460251] <- vertices[01234567] ConvexMeshCube
    // ( x  y -z) ( x  y  z) ( x -y  z) ( x -y -z)
    // (-x  y -z) (-x  y  z) (-x -y  z) (-x -y -z)
    {4, {7, 4, 0, 3}}, {4, {7, 6, 5, 4}}, {4, {7, 3, 2, 6}},
    {4, {1, 0, 4, 5}}, {4, {1, 2, 3, 0}}, {4, {1, 5, 6, 2}}};
  CreateFaceMesh(meshd, r, sz, vertices, faces);
}

void BindSolidFrame(FWSdkIf *fwSdk, PHSolidIf *so,
  GRMeshDesc &meshd, GRMaterialDesc &matd, GRFrameDesc &frmd=frm_desc)
{
  GRSceneIf *grScene = fwSdk->GetScene(0)->GetGRScene();
#if 0
  GRFrameIf *frm = grScene->CreateVisual(frmd, grScene->GetWorld())->Cast();
#else
  GRFrameIf *frm = grScene->CreateVisual(frmd)->Cast(); // parent = world
#endif
//  DispMeshDesc(meshd); // long time // SprGRMesh.h SprGRFrame.h SprCDShape.h
  GRMeshIf *mesh = grScene->CreateVisual(meshd, frm)->Cast();
  GRMaterialIf *mat = grScene->CreateVisual(matd, frm)->Cast();
  mesh->AddChildObject(mat); // 0 -> meshd.materialList[0]
//  mesh->AddChildObject(mat); // 1 -> meshd.materialList[1]
//  mesh->AddChildObject(mat); // 2 -> meshd.materialList[2]
//  grRender->SetMaterial(mat);
  FWObjectIf *fwObj = fwSdk->GetScene(0)->CreateFWObject();
  fwObj->SetPHSolid(so);
  fwObj->SetGRFrame(frm);
  fwSdk->GetScene(0)->Sync(); // can not set true ?
}

void BindSolidBox(FWSdkIf *fwSdk, PHSolidIf *so, float r, Vec3f sz, int c)
{
  CDShapeIf *shape = so->GetShape(0);
  GRMeshDesc meshd;
  CreateBoxMesh(meshd, r, sz, shape);
  Vec4f col = fwSdk->GetRender()->GetReservedColor(c);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
#if 0 // no more match vertices.size()
  meshd.colors = std::vector<Vec4f>{
    {.5f,.5f,.5f, 1}, {.9f,  0,  0, 1}, {  0,.9f,  0, 1}, {  0,  0,.9f, 1},
    {.9f,.9f,  0, 1}, {.9f,  0,.9f, 1}, {  0,.9f,.9f, 1}, {.9f,.9f,.9f, 1}};
#endif
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_PLANE;
  BindSolidFrame(fwSdk, so, meshd, matd);
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

  GRMeshDesc meshd;
#if 1
  CreateCylinderMesh(meshd, r, PNR[m - 1][0], PNR[0][0], PNR, m, t);
#else
  float DUMMY[][2] = {
    {15.0f, 4.766f}, // 0.800f},
    {10.0f, 4.766f}, // 1.797f},
    { 0.0f, 4.766f}}; // 2.250f}};
  const int M = sizeof(DUMMY) / sizeof(DUMMY[0]);
  CreateCylinderMesh(meshd, r, DUMMY[M - 1][0], DUMMY[0][0], DUMMY, M, 6);
#endif
  //meshd.normals = std::vector<Vec3f>{};
  //meshd.faceNormals = std::vector<GRMeshFace>{};
  Vec4f col = fwSdk->GetRender()->GetReservedColor(c);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_PIN;
  BindSolidFrame(fwSdk, cvxs[0], meshd, matd);
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

#if 0
  GRSphereDesc meshd;
  meshd.radius = sd.radius;
  meshd.slices = 24; // 18; // 12; // lng default 16
  meshd.stacks = 18; // lat default 16
  //meshd.materialList = std::vector<int>{0}; // not a member
  GRMaterialDesc matd = mat_desc;
//  matd.texname = TEX_BALL; // no effect ?
#else
  GRMeshDesc meshd;
  CreateSphereMesh(meshd, r, rad, 24, 12); // slices=24, stacks=12
  //meshd.normals = std::vector<Vec3f>{};
  //meshd.faceNormals = std::vector<GRMeshFace>{};
  Vec4f col = fwSdk->GetRender()->GetReservedColor(c);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_BALL;
#endif
  BindSolidFrame(fwSdk, so, meshd, matd);
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

  BindSolidBox(fwSdk, soPlane, r, sz, c);
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
  float apd = 15.35f * feetinch;
  float psd = 2.6f * feetinch, lst = 2.849f * feetinch, pdr = 7.67f * feetinch;
  float aph = apd / 2.0f, lsh = lst / 2.0f;
  float lnh = 1.0f, lnw = 41.5f, lnd = 60.0f * feetinch;
  Vec3f gtsi = Vec3f(lnd + lst, lnh, (BALL_R * 2.0f + 0.75f) / 2.0f);
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
    pos + Vec3d(-lnd / 2.0f, lnh / 2.0f, lnw / 39.0f * 3.2f), BALL_R, r);
  soBall->SetMass(0.9);
  soBall->SetVelocity(Vec3d(lnd * r / 2.4, 0.0, 0.0));
  soBall->SetAngularVelocity(Vec3d(-5.0, 2.0, -5.0));
*/
/* // hooking point 50ft ? 7pin only or left gutter
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.9);
  soBall->SetVelocity(Vec3d(lnd * r / 6.0 / 2.4, 0.0, -38.0 * r / 2.4));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -17.5 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? 10pin tap
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.0 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? just ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.4 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
/* // hooking point 50ft ? 9pin tap
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 3.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 3.6, 0.0, 6.0 * -19.5 * r / 3.6));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
*/
// hooking point 40ft ?
  PHSolidIf *soBall = CreateBall(fwSdk, GRRenderBaseIf::BLUEVIOLET,
    pos + Vec3d(lnd / 6.0f, lnh / 2.0f, lnw / 39.0f * 19.9f), BALL_R, r);
  soBall->SetMass(0.85);
  soBall->SetVelocity(Vec3d(lnd * r / 4.8, 0.0, 3.0 * -19.3 * r / 4.8));
  soBall->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));

  soBall_ref = soBall;
  return soLane;
}

PHSolidIf *CreateConvexMeshTetra(FWSdkIf *fwSdk)
{
  float w = 1 - tan(15 * PI / 180);
  float a = sqrt(2.0f) * w;
  float g = (1 + w) / 3;
  float h = 2 * sqrt(3.0f) * w / 3; // == sqrt(6.0f) * a / 3;
  float c = sqrt(3.0f) * w / 6; // == h - sqrt(6.0f) * a / 4;
  std::vector<Vec3f> vertices = {
    {0, h - c, 0}, {w - g, -c, -g}, {1 - g, -c, 1 - g}, {-g, -c, w - g}};
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

  std::vector<GRMeshFace> faces = std::vector<GRMeshFace>{
    {3, {1, 0, 2}}, {3, {2, 0, 3}}, {3, {3, 0, 1}}, {3, {1, 2, 3}}};
  GRMeshDesc meshd;
  CreateFaceMesh(meshd, 1.0f, Vec3f(1.0f, 1.0f, 1.0f), vertices, faces);
  Vec4f col = fwSdk->GetRender()->GetReservedColor(GRRenderBaseIf::INDIGO);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
#if 0 // no more match vertices.size()
  meshd.colors = std::vector<Vec4f>{
    {.5f,.5f,.5f, 1}, {.9f,.7f,.2f, 1}, {.2f,.9f,.7f, 1}, {.7f,.2f,.9f, 1}};
#endif
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_TETRA;
  BindSolidFrame(fwSdk, cvx, meshd, matd);
  return cvx;
}

PHSolidIf *CreateConvexMeshCube(FWSdkIf *fwSdk)
{
  std::vector<Vec3f> vertices = {
    {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1},
    {1, 1, -1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1}};
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

  std::vector<GRMeshFace> faces = std::vector<GRMeshFace>{
    {4, {0, 2, 4, 1}}, {4, {0, 3, 6, 2}}, {4, {0, 1, 5, 3}},
    {4, {7, 4, 2, 6}}, {4, {7, 5, 1, 4}}, {4, {7, 6, 3, 5}}};
  GRMeshDesc meshd;
  CreateFaceMesh(meshd, 1.0f, Vec3f(1.0f, 1.0f, 1.0f), vertices, faces);
  Vec4f col = fwSdk->GetRender()->GetReservedColor(GRRenderBaseIf::NAVY);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_CUBE;
  BindSolidFrame(fwSdk, cvx, meshd, matd);
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

  BindSolidBox(fwSdk, soBox, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::DODGERBLUE);
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

  GRMeshDesc meshd;
  CreateSphereMesh(meshd, 1.0f, sd.radius, 24, 12); // slices=24, stacks=12
  //meshd.normals = std::vector<Vec3f>{};
  //meshd.faceNormals = std::vector<GRMeshFace>{};
  Vec4f col = fwSdk->GetRender()->GetReservedColor(GRRenderBaseIf::SEAGREEN);
  meshd.colors = std::vector<Vec4f>(meshd.vertices.size());
  for(int i = 0; i < meshd.vertices.size(); ++i) meshd.colors[i] = col;
//  meshd.materialList = std::vector<int>{0};
  GRMaterialDesc matd = mat_desc;
  matd.texname = TEX_BALL;
  BindSolidFrame(fwSdk, soSphere, meshd, matd);
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
    {640, 480, 120, 480 + 32 + 4, WIN_BALL, false, DBG},
    {640, 480, 120, 0, WIN_UP, false, DBG},
    {640, 480, 120 + 640 + 4, 0, WIN_PINTOP, false, DBG},
    {640, 480, 120 + 640 + 4, 480 + 32 + 4, WIN_SIDE, false, DBG},
    {480, 360, 120 + (640 + 4) * 2, 0, WIN_TITLE, false, DBG},
    {480, 360, 120 + (640 + 4) * 2, 360 + 32 + 4, WIN_DEBUG, false, true}};
  for(int i = 0; i < sizeof(wdp) / sizeof(wdp[0]); ++i){
    FWWinDesc wd;
    wd.title = wdp[i].title;
    wd.width = wdp[i].width, wd.height = wdp[i].height;
    wd.left = wdp[i].left, wd.top = wdp[i].top;
    wd.fullscreen = wdp[i].fullscreen, wd.debugMode = wdp[i].debugMode;
    CreateWin(wd);
    FWWinIf *w = GetWin(i);
    WinID[i] = w->GetID();
    w->GetTrackball()->SetPosition(Vec3f(0.0, 0.0, 0.0));
    w->SetScene(GetSdk()->GetScene(0)); // all Win to Scene 0
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
  for(int i = 0; i < W; ++i){
    SetCurrentWin(GetWin(i));
    // GetCurrentWin() SetViewMatrix() etc ...
    PostRedisplay();
  }
  SetCurrentWin(GetWin(0));
}

void MyApp::DispInf(FWWinIf *w, FWSceneIf *fwScene, GRRenderIf *grRender)
{
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

void MyApp::Display()
{
//  FWApp::Display(); // skip default
//  GetCurrentWin()->Display();
//  for(int i = 0; i < W; ++i) GetWin(i)->Display(); // (all Scene in all Win)
  FWWinIf *w = GetCurrentWin();
  int id = w->GetID();
//  w->Display();
  GRRenderIf *grRender = w->GetRender();
  grRender->BeginScene();
  grRender->ClearBuffer(true, true);
/**/
  if(id == WinID[0]){ // WIN_BALL
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
//  grRender->SetMaterial(GRRenderBaseIf::WHITE);
  grRender->SetViewMatrix(w->GetTrackball()->GetAffine().inv());
  FWSceneIf *fwScene = w->GetScene();
  if(id == WinID[W - 1]) fwScene->SetRenderMode(false, true); // wire WIN_DEBUG
  else fwScene->SetRenderMode(true, false); // solid
  if(DBG){
    fwScene->EnableRenderAxis(bDrawInfo);
    fwScene->EnableRenderForce(bDrawInfo);
    fwScene->EnableRenderContact(bDrawInfo);
    //fwScene->EnableRenderGrid(bDrawInfo);
  }
  fwScene->Draw(grRender, w->GetDebugMode()); // shown when wireframe mode
  //fwScene->Draw(grRender, true); // force true PH only
  //fwScene->Draw(grRender, false); // force false GR only
  //fwScene->Draw(grRender); // normal (not set debug=false) see FWScene.cpp
  if(id == WinID[W - 1]) DispInf(w, fwScene, grRender); // WIN_DEBUG
  grRender->EndScene();
  grRender->SwapBuffers();
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
    CreateConvexMeshPin(GetSdk(), GRRenderBaseIf::GOLD, Vec3d(0, 5, 0), PNS);
    break;
  case ',':
    DSTR << "spheremeshball" << std::endl;
    CreateBall(GetSdk(), GRRenderBaseIf::GREEN, Vec3d(0, 6, 0), BALL_R, PNS);
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
    {-lnd / 4.0f, 0.0f, 0.0f, -PI / 2.0f, 0.0f, lnd * 3.0f / 10.0f},
    {lnd / 2.0f, 0.0f, 0.0f, -PI / 2.0f, PI / 36.0f, 20.0f},
    {lnd / 2.0f, 0.0f, 0.0f, -PI / 2.0f, PI / 2.0f, 20.0f},
    {0.0f, 0.0f, 0.0f, 17.0f * PI / 36.0f, PI / 36.0f, 95.0f}, // <= max 99
    {-lnd / 4.0f, 0.0f, 0.0f, -PI / 2.0f, 3.0f * PI / 8.0f, lnd * 4.0f / 5.0f},
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
  //phScene->GetConstraintEngine()->SetUseContactSurface(true); // face contact
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

#if 0
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
  BindSolidBox(fwSdk, floor, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::HOTPINK);

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
  BindSolidBox(fwSdk, box, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::BLUE);

  PHSolidIf *sol = phScene->CreateSolid(sd); // sol->SetMass(5.0); // etc
  bd.boxsize = Vec3f(0.5f, 0.3f, 0.3f);
  sol->AddShape(phSdk->CreateShape(bd));
  sol->SetCenterPosition(Vec3d(0.0, 1.5, 0.0));
  sol->SetVelocity(Vec3d(0.0, 0.7, 0.0));
  sol->SetAngularVelocity(-Vec3d(0.5, 0.5, 0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::ORANGERED, sol);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::ORANGERED, sol);
  BindSolidBox(fwSdk, sol, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::ORANGERED);

  PHSolidIf *sol0 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol0->AddShape(phSdk->CreateShape(bd));
  sol0->SetCenterPosition(Vec3d(0.5, 0.5, -0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::CYAN, sol0);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::CYAN, sol0);
  BindSolidBox(fwSdk, sol0, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::CYAN);
  PHSolidIf *sol1 = phScene->CreateSolid(sd);
  bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
  sol1->AddShape(phSdk->CreateShape(bd));
  sol1->SetCenterPosition(Vec3d(0.5, 0.5, 0.5));
  fwSdk->GetScene(0)->SetSolidMaterial(GRRenderBaseIf::MAGENTA, sol1);
  fwSdk->GetScene(0)->SetWireMaterial(GRRenderBaseIf::MAGENTA, sol1);
  BindSolidBox(fwSdk, sol1, 1.0f, Vec3f(1.0f, 1.0f, 1.0f),
    GRRenderBaseIf::MAGENTA);
  PHHingeJointDesc hjd;
  hjd.poseSocket.Pos() = Vec3d(1.0, 0.0, 0.0);
  hjd.posePlug.Pos() = Vec3d(-1.0, 0.0, 0.0);
  PHHingeJointIf *joint = phScene->CreateJoint(sol0, sol1, hjd)->Cast();
#endif

  CreateLane(GetSdk(), Vec3d(0.0, -20.0, 0.0), PNS);
}

void MyApp::Reset()
{
  tick = 0;
//  FWApp::Reset(); // skip default
  GetSdk()->GetScene(0)->GetPHScene()->Clear();
  GetSdk()->GetScene(0)->GetGRScene()->Clear();
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

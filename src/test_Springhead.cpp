/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <Springhead.h>
#include <Framework/SprFWApp.h>

#include <test_Springhead.h>

using namespace Spr;

class MyApp : public FWApp {
public:
  virtual void Init(int ac=0, char **av=0)
  {
    FWApp::Init(ac, av);
    PHSdkIf *phSdk = GetSdk()->GetPHSdk();
    PHSceneIf *phscene = GetSdk()->GetScene()->GetPHScene();
    Quaterniond q = Quaterniond::Rot(Rad(45.0), Vec3d(1, 1, 1));
//    Matrix3d rot;
//    q.ToMatrix(rot);
    CDBoxDesc bd;
    PHSolidDesc sd;
    sd.mass = 1.0;

    PHSolidIf *floor = phscene->CreateSolid();
    floor->SetDynamical(false);
    floor->SetMass(1000.0);
    bd.boxsize = Vec3f(5.0f, 1.0f, 5.0f);
    floor->AddShape(phSdk->CreateShape(bd));
    floor->SetFramePosition(Vec3d(0, -1.0, 0));

    PHSolidIf *box = phscene->CreateSolid(sd);
    bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
    box->AddShape(phSdk->CreateShape(bd));
    box->SetCenterPosition(Vec3d(0.0, 1.0, 0.0));
    box->SetVelocity(Vec3d(0.0, 0.5, 0.0));
    box->SetOrientation(q);

    PHSolidIf *sol = phscene->CreateSolid(sd); // sol->SetMass(5.0); // etc
    bd.boxsize = Vec3f(0.5f, 0.3f, 0.3f);
    sol->AddShape(phSdk->CreateShape(bd));
    sol->SetCenterPosition(Vec3d(0.0, 1.5, 0.0));
    sol->SetVelocity(Vec3d(0.0, 0.5, 0.0));
    sol->SetAngularVelocity(-Vec3d(0.5, 0.5, 0.5));

    GetSdk()->SetDebugMode(true);
  }
} app;

int main(int ac, char **av)
{
  fprintf(stdout, "sizeof(size_t): %zd\n", sizeof(size_t));
  fprintf(stdout, "%s\n", TEST_WORD);
  app.Init(ac, av);
  app.StartMainLoop();
  return 0;
}

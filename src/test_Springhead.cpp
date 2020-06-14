/*
  test_Springhead.cpp

  Springhead.pdf p.14 Table 2.2
*/

#include <Springhead.h>
#include <Framework/SprFWApp.h>

using namespace Spr;

class MyApp : public FWApp {
public:
  virtual void Init(int ac=0, char **av=0)
  {
    FWApp::Init(ac, av);
    PHSdkIf *phSdk = GetSdk()->GetPHSdk();
    PHSceneIf *phscene = GetSdk()->GetScene()->GetPHScene();
    CDBoxDesc bd;

    PHSolidIf *floor = phscene->CreateSolid();
    floor->SetDynamical(false);
    bd.boxsize = Vec3f(5.0f, 1.0f, 5.0f);
    floor->AddShape(phSdk->CreateShape(bd));
    floor->SetFramePosition(Vec3d(0, -1.0, 0));

    PHSolidIf *box = phscene->CreateSolid();
    bd.boxsize = Vec3f(0.2f, 0.2f, 0.2f);
    box->AddShape(phSdk->CreateShape(bd));
    box->SetFramePosition(Vec3d(0.0, 1.0, 0.0));
    GetSdk()->SetDebugMode(true);
  }
} app;

int main(int ac, char **av)
{
  fprintf(stdout, "sizeof(size_t): %zd\n", sizeof(size_t));
  app.Init(ac, av);
  app.StartMainLoop();
  return 0;
}

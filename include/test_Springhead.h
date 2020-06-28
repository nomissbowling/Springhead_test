/*
  test_Springhead.h
*/

#ifndef __TEST_SPRINGHEAD_H__
#define __TEST_SPRINGHEAD_H__

#include <Springhead.h>
#include <Framework/SprFWApp.h>

#define WIN_TITLE "test_Springhead"
#define WIN_BALL WIN_TITLE"_BALL"
#define WIN_UP WIN_TITLE"_Up"
#define WIN_PINTOP WIN_TITLE"_PinTop"
#define WIN_SIDE WIN_TITLE"_Side"
#define WIN_DEBUG WIN_TITLE"_Debug"

#define RES_DIR "C:\\prj\\Springhead_test\\res\\"
#define TEX_LANE RES_DIR"lane_256x256.png"
#define TEX_PIN RES_DIR"pin_256x256.png"
#define TEX_BALL RES_DIR"ball_256x256.png"
#define TEX_PLANE RES_DIR"plane_256x256.png"
#define TEX_SPHERE RES_DIR"Globe_Lambert_256x81.png"
#define TEX_CUBE RES_DIR"cube_256x256.png"
#define TEX_TETRA RES_DIR"tetra_256x256.png"
#define TEX_LBRBRTLT RES_DIR"lbrbrtlt_256x256.png"

using namespace Spr;

struct MyCameraDescPart {
  float x, y, z, lng, lat, r;
};

struct MyWinDescPart {
  int width, height, left, top;
  const char *title;
  bool fullscreen, debugMode;
};

class MyApp : public FWApp {
public:
  bool bDrawInfo;
  unsigned long tick;
public:
  MyApp();
  virtual ~MyApp();
  virtual void Init(int ac=0, char **av=0);
  virtual void TimerFunc(int id);
  void DispInf(FWWinIf *w, FWSceneIf *fwScene, GRRenderIf *grRender);
  virtual void Display();
  virtual void Keyboard(int key, int x, int y);
  void InitCameraView();
  void CreateCameras();
  void CreateLights();
  void CreateObjects();
  void Reset();
};

#endif // __TEST_SPRINGHEAD_H__

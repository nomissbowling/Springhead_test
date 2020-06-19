/*
  test_Springhead.h
*/

#ifndef __TEST_SPRINGHEAD_H__
#define __TEST_SPRINGHEAD_H__

#include <Springhead.h>
#include <Framework/SprFWApp.h>

#define WIN_TITLE "test_Springhead"
#define WIN_UP "test_Springhead_Up"
#define WIN_PINTOP "test_Springhead_PinTop"
#define WIN_SIDE "test_Springhead_Side"

using namespace Spr;

class MyApp : public FWApp {
public:
  bool bDrawInfo;
public:
  MyApp();
  virtual ~MyApp();
  virtual void Init(int ac=0, char **av=0);
  virtual void TimerFunc(int id);
  virtual void Display();
  virtual void Keyboard(int key, int x, int y);
  void InitCameraView();
  void CreateObjects();
  void Reset();
};

#endif // __TEST_SPRINGHEAD_H__

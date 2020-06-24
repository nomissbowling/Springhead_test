Springhead_test
===============

Springhead test

[animgif]: https://github.com/nomissbowling/Springhead_test/blob/master/res/test_Springhead_bowling_4scenes_20200619.gif?raw=true
![screenshot][animgif]
 * https://github.com/nomissbowling/Springhead_test/blob/master/res/test_Springhead_bowling_4scenes_20200619.gif?raw=true

Install memo
------------

get clone from GitHub Springhead with submodule dependencies (tested 57fe4ca9741115099c427ed9f56c0a9829341eec)
- git clone --recurse-submodules https://github.com/sprphys/Springhead.git

mkdir
- mkdir ../../Springhead/build

cmake (3.17.3)
- source code '../../Springhead/core/src'
- binaries '../../Springhead/build'
- Specify the generator 'Visual Studio 15 2017'
- Optional platform for generator 'x64'
- Optional toolset none
- Use default native compilers

check Windows SDK version
- ../../Springhead/core/bin/src/swig/msvc/swig15.0.vcxproj (search 10.0.17763.0 and edit it.)

build (Visual Studio 2017)
- Springhead.sln (Release x64)
- built ../../Springhead/core/bin/src/swig/msvc/x64/Release15.0/swig15.0.exe
- built ../../Springhead/generated/lib/win64/Springhead15.0x64.lib

new solution and project
- test_Springhead/test_Springhead.sln (empty)
- test_Springhead/test_Springhead/test_Springhead.vcxproj (empty) Release x64
- append include/test_Springhead.h
- append src/test_Springhead.cpp
- project property (change output directory etc) active Release x64

source path
- ../../Springhead_test/src

include path
- ../../Springhead_test/include
- ../../Springhead/core/include
- ../../Springhead/core/src (need to use .hpp etc)

library path
- ../../Springhead/generated/lib/win64
- ../../Springhead/dependency/lib/win64

link library
- Springhead15.0x64.lib

platform
- x64

environment path
- Springhead/core/bin/win64
- Springhead/dependency/bin/win64
- Springhead/core/bin/win32
- Springhead/dependency/bin/win32


Springhead
----------

[Springhead](https://github.com/sprphys/Springhead)

[Springhead build (use cmake first)](http://springhead.info/dailybuild/generated/doc/HowToUseCMake.pdf)

[Springhead reference](http://springhead.info/dailybuild/generated/doc/Springhead.pdf)

# S01E00: 设置开发环境
1. 通过`git clone git@github.com:RongyeL/EasyAXI.git`获取本系列视频的项目文件;
2. 该系列视频中，在编译或仿真前，需要在`EasyAXI/S01EXX`路径下，执行`source script/project.sh`，通过该脚本确定当期项目的根路径；
3. 该环境下，执行编译或仿真
    - 进入到`EasyAXI/S01EXX/verification/sim`路径下
    - 仅编译：`make compile`
    - 编译并仿真：`make run`
    - 打开verdi：`make verdi&`，需先进行过编译
4. 打开verdi后，可以看到编译好的代码，并可以看到正常波形，说明环境已准备就绪。

> 关于开发软件：很抱歉，vcs+verdi的环境需要自行安装，本教学视频不会说明或提供。

---

# S01E00: Set up the development environment
1. Get the project files of this series of videos through `git clone git@github.com:RongyeL/EasyAXI.git`;
2. In this series of videos, before compiling or simulating, you need to execute `source script/project.sh` in the `EasyAXI/S01EXX` path, and use this script to determine the root path of the current project;
3. Execute compilation or simulation in this environment
- Enter the `EasyAXI/S01EXX/verification/sim` path
- Compile only: `make compile`
- Compile and simulate: `make run`
- Open verdi: `make verdi&`, which needs to be compiled first
4. After opening verdi, you can see the compiled code and the normal waveform, indicating that the environment is ready.

> About development software: Sorry, the vcs+verdi environment needs to be installed by yourself, which will not be explained or provided in this teaching video.

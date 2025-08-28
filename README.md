# EasyAXI(施工中)
## S01E01: 设置开发环境 
系列视频之初，先准备一个能够正常编译、仿真的环境。 - [跳转到 S01E01](./S01E01/README.md)

## S01E02: 初识AXI握手
AXI接口数据交互使用的是valid&ready握手机制，理解握手是使用AXI的第一步。 - [跳转到 S01E02](./S01E02/README.md)

## S01E03: AXI读通道初实现
以读通道为例，展示一个完整的AXI读操作流程。 - [跳转到 S01E03](./S01E03/README.md)

## S01E04: AXI读Burst实现
以读通道为例，说明AXI Burst特性的实现，包括FIXED、INCR、WRAP类型。 - [跳转到 S01E04](./S01E04/README.md)

## S01E05: AXI outstanding实现
以读通道为例，说明AXI OST特性的实现，支持多种OST深度可配。 - [跳转到 S01E05](./S01E05/README.md)

## S01E06: AXI 乱序&交织实现
读事务为例，说明AXI Out-of-order、Interleave特性的实现。 - [跳转到 S01E06](./S01E06/README.md)

## S01E07: AXI 同ID保序实现
读事务为例，说明AXI ID同保序特性实现。 - [跳转到 S01E07](./S01E07/README.md)


## 注：
1. 通过`git clone https://github.com/RongyeL/EasyAXI.git --depth 1`获取本系列视频的项目文件;
2. 在编译或仿真前，需要在`EasyAXI/S01EXX`路径下，执行`source script/project.sh`，通过该脚本确定当期项目的根路径；
3. 关于开发软件：很抱歉，vcs+verdi的环境需要自行安装，本教学视频不会说明或提供。

--- 

# EasyAXI (under construction)
## S01E01: Set up the development environment
At the beginning of the video series, prepare an environment that can compile and simulate normally. - [Jump to S01E01](./S01E01/README.md)

## S01E02: Introduction to AXI handshake
AXI interface data interaction uses the valid&ready handshake mechanism. Understanding the handshake is the first step to use AXI. - [Jump to S01E02](./S01E02/README.md)

## S01E03: Initial implementation of AXI read channel
Taking the read channel as an example, show a complete AXI read operation process. - [Jump to S01E03](./S01E03/README.md)

## S01E04: AXI read Burst implementation
Taking the read channel as an example, explain the implementation of the AXI Burst feature, including FIXED, INCR, and WRAP types. - [Jump to S01E04](./S01E04/README.md)ment needs to be installed by yourself, and this tutorial video will not explain or provide it.

## S01E05: AXI outstanding implementation
Take the read channel as an example to explain the implementation of the AXI OST feature, supporting multiple OST depth configurations. - [Jump to S01E05](./S01E05/README.md)

## S01E06: AXI Out-of-Order & Interleave Implementation
This section uses a read transaction as an example to illustrate the implementation of the AXI Out-of-Order and Interleave features. - [Jump to S01E06](./S01E06/README.md)

# S01E07: AXI Same-ID ORDER Implementation
Using a read transaction as an example, this section describes the implementation of the AXI Same-ID Sequence Preservation feature. - [Jump to S01E07](./S01E07/README.md)

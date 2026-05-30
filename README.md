# EasyAXI

从零开始实现 AXI 总线协议的教学项目，配套[视频系列](https://space.bilibili.com/)。

S01E09 为当前最新版本，在前 8 集逐步实现的 AXI 读/写通道、Burst、Outstanding、
乱序、保序等特性的基础上，构建了完整的 **EasyAXI 控制器 IP 框架**——将底层 AXI
握手封装为简洁的 req/resp 接口，由四个控制器自动完成 OST 管理、同 ID 保序、地址
计算。上层业务（Producer/Consumer）只需提供 addr + size + data，无需触碰 AW/W/B/AR/R。

```
Producer              Consumer
  │ m_wr_req/resp       │ m_rd_req/resp
  ▼                     ▼
MST_WR_CTRL          MST_RD_CTRL     ← Master Side — 请求/响应到 AXI 信号
  │ AXI AW/W/B          │ AXI AR/R
  ▼                     ▼
SLV_WR_CTRL          SLV_RD_CTRL     ← Slave Side — AXI 信号到内部接口
  │ s_wr_data_*          │ s_rd_req/resp_*
  ▼                     ▼
        SRAM / MEM
```

## 快速开始

```bash
git clone https://github.com/RongyeL/EasyAXI.git --depth 1
cd EasyAXI/S01E09
source script/project.sh
cd verification/sim
make run
```

> 需要自行安装 VCS + Verdi 环境，视频教程不涉及。

## 目录

| 章节 | 主题 | 内容 |
|------|------|------|
| [S01E01](./S01E01/) | 设置开发环境 | 准备 VCS + Verdi 编译仿真环境 |
| [S01E02](./S01E02/) | 初识 AXI 握手 | valid/ready 双向握手机制 |
| [S01E03](./S01E03/) | AXI 读通道 | 完成一次完整读操作 |
| [S01E04](./S01E04/) | AXI 读 Burst | FIXED / INCR / WRAP 三种 burst 类型 |
| [S01E05](./S01E05/) | AXI Outstanding | 流水线并行、多事务 OST 管理 |
| [S01E06](./S01E06/) | AXI 乱序 & 交织 | Out-of-Order / Interleave |
| [S01E07](./S01E07/) | AXI 同 ID 保序 | 读事务响应顺序保证 |
| [S01E08](./S01E08/) | AXI 写操作 | 写通道实现、写数据保序 |
| [**S01E09**](./S01E09/) | **CQE 生产者-消费者** | **EasyAXI 控制器 IP 框架（MST_WR/RD_CTRL、SLV_WR/RD_CTRL + OST/ORDER/Burst Addressing 共享机制）** |

## 工程结构

```
S01E09/                        # 最新版本，含完整控制器 IP 框架
├── doc/
│   └── axi_ctrl_spec.md       # 控制器接口规范（信号表、数据流、时序图）
├── rtl/
│   ├── easyaxi_define.v       # AXI 位宽宏定义
│   ├── easyaxi_top.v          # 顶层集成
│   ├── easyaxi_prod.v         # Producer — 生产者状态机
│   ├── easyaxi_cons.v         # Consumer — 消费者状态机
│   ├── easyaxi_mst_wr_ctrl.v  # Master 写控制器 (req → AW/W/B)
│   ├── easyaxi_mst_rd_ctrl.v  # Master 读控制器 (req → AR/R)
│   ├── easyaxi_slv_wr_ctrl.v  # Slave 写控制器  (AW/W/B → 内部接口)
│   ├── easyaxi_slv_rd_ctrl.v  # Slave 读控制器  (AR/R → 内部接口)
│   ├── easyaxi_sram.v         # SRAM 封装 (Slave 控制器 + MEM)
│   ├── easyaxi_mem.v          # 128-bit 寄存器堆存储器
│   ├── easyaxi_arb.v          # 轮转优先级仲裁器 (Round-Robin)
│   ├── easyaxi_order.v        # 同 ID 保序队列 (per-ID FIFO)
│   └── easyaxi_fifo.v         # 同步 FIFO
├── verification/
│   ├── tb/testbench.v         # 仿真验证
│   └── sim/Makefile           # VCS 编译/仿真/波形
└── script/project.sh          # 环境初始化
```

各子集（S01E01 ~ S01E08）仅包含当集相关的 RTL 文件，结构类似。

## 环境

| 工具 | 版本 | 用途 |
|------|------|------|
| VCS | O-2018.09-SP2 | 编译 + 仿真 |
| Verdi | O-2018.09-SP2 | 波形调试 |

---

[GitHub](https://github.com/RongyeL/EasyAXI) · [Apache 2.0](./LICENSE)

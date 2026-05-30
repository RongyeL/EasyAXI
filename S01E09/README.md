# S01E09 - CQE 生产者-消费者模型

## 概述

本实验实现了一个基于 AXI 协议的 **CQE (Completion Queue Entry) 生产者-消费者模型**：

- **Producer (EASYAXI_PROD)**：写 N 个 Data Block 到 SRAM（数量通过 PROD_NUM_BLOCKS 配置，1~15，默认 4），每个 Data Block 写完后写一个 CQE 通知 Consumer
- **Consumer (EASYAXI_CONS)**：轮询 CQE 表，发现 VALID=1 后读取对应的 Data Block 并校验数据（数量通过 CONS_NUM_BLOCKS 配置，与 Producer 一致）

## 地址映射

| 地址范围 | 内容 |
|---------|------|
| `0x00 ~ 0x0F` | CQE 表：CQE[idx] 在地址 `0x00 + idx`，1 字节 |
| `0x10 ~ 0xFF` | Data Block：Block[idx] 在地址 `0x10 + idx*0x10`，16B 对齐 |

### CQE 格式（1 字节）

| 位段 | 名称 | 说明 |
|------|------|------|
| `bit[7:4]` | SIZE | Data Block 的字节数编码（1/2/4/8，编码 15 表示 16B） |
| `bit[3:1]` | 保留 | 固定为 0 |
| `bit[0]` | VALID | 1 表示数据已就绪 |

### Data Block 格式

每个 Data Block 包含 `(SIZE)` 字节数据，按 4 字节 word 组织，每个 word 格式：

| 位段 | 名称 | 说明 |
|------|------|------|
| `bit[31:28]` | SIZE | 与 CQE 的 SIZE 字段一致（1/2/4/8，15 表示 16B） |
| `bit[27:20]` | byte_ofs | 该 word 在 block 中的字节偏移量（0/4/8/12...） |
| `bit[19:4]` | 保留 | `16'h0000` |
| `bit[3:0]` | idx | Block 索引（Consumer 校验 [3:0] 是否等于 idx） |

## 时序关系图

```
Producer (idx=0):
  IDLE --> WRITE_DATA --> WAIT_BRESP --> WRITE_CQE --> WAIT_CQE --> NEXT
                         (AW+W握手)      (写CQE)       (B响应 -> cqe_written脉冲)
                                                                     |
                                                                     v
  done <-- DONE <-- ... <-- WRITE_DATA(idx=1) <-- PROD_GAP(20+idx*18) <-- NEXT
              循环 N 次                      idx 越大间隔越长

  idx=0: gap=20,  idx=1: gap=38,  idx=2: gap=56,  idx=3: gap=74

Consumer (idx=0):
  IDLE --> POLL_WAIT(16cycle) --> READ_CQE --> WAIT_CQE --> CHECK_CQE
          (轮询间隔 16 clk)     (读CQE表)    (等R响应)     (cqe_valid/cqe_invalid)
                                                                     |
                                                    +---+------------+
                                                    |               |
                                              VALID=1           VALID=0
                                              (HIT)             (MISS)
                                              cqe_valid=1       cqe_invalid=1
                                                    |               |
                                              READ_DATA          v
                                                    |       POLL_WAIT(16cycle)
                                              WAIT_DATA          |  ^
                                                    |        READ_CQE..
                                              VERIFY <---+
                                              (校验数据)
                                                    |
                                                  NEXT

CQE MISS 原理（为啥 Consumer 要重新轮询）:
  Producer 写 idx= N 的 CQE 耗时:  PROD_GAP(20+N*18) + WRITE_DATA(~2) + WRITE_CQE(~2)
  Consumer 到达 idx=N CHECK_CQE:  (N-1个block处理时间) + POLL_WAIT(16)
  当 PROD_GAP 足够大时 Consumer 先到 -> cqe_invalid 拉高 -> CQE MISS -> 回到 POLL_WAIT
  当前配置: idx=0 HIT, idx=1/2/3 各触发 MISS（idx=3 重试 2 次）
```

## 视频讲解路线建议

1. **看架构图**（`doc/axi_ctrl_spec.md` 1.1 节速查卡）— 四个控制器各干什么
2. **看 Producer 代码** — 状态机 + `m_wr_req_*` 接口如何驱动 MST_WR_CTRL
3. **看 Consumer 代码** — 状态机 + `m_rd_req_*` 接口如何驱动 MST_RD_CTRL
4. **看 MST_WR_CTRL 内部** — req/size 如何转为 AXI AW/W/B 信号
5. **看 MST_RD_CTRL 内部** — AR/R 如何汇聚为 resp_data
6. **看波形印证** — 观察 AW/W/B/AR/R 握手与上层 req/resp 的对应关系

## AXI Ctrl 接口信号

EasyAXI 提供四个 AXI 控制器 IP，将复杂的 AXI 协议握手封装为简洁的 req/resp 接口。四个控制器共享 OST、ORDER、Burst Addressing 三套机制（详见 `doc/axi_ctrl_spec.md` 第 2-4 章）。

### 架构总览

```
Producer               Consumer
  │ m_wr_req/resp        │ m_rd_req/resp
  ▼                      ▼
MST_WR_CTRL           MST_RD_CTRL        ← Master Side
  │ AXI AW/W/B           │ AXI AR/R
  ▼                      ▼
SLV_WR_CTRL           SLV_RD_CTRL        ← Slave Side
  │ s_wr_data_*           │ s_rd_req/resp_*
  ▼                      ▼
         SRAM / MEM
```

### MST_WR_CTRL — Master Write Controller

将 Producer 的写请求映射为 AXI AW/W/B 三通道握手。上层只需 10 根 req/resp 信号，控制器自动完成 burst 拆分、OST 管理、W 数据保序。

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `m_wr_req_en` | 1 | 脉冲：拉高一拍即发起写请求 |
| I | `m_wr_req_id` | 4 | 事务 ID（`easyaxi_prod.v:233` 固定为 0） |
| I | `m_wr_req_addr` | 16 | 起始地址（字节地址） |
| I | `m_wr_req_size` | 8 | 传输字节数 1/2/4/8/16/32/64/128，自动转为 AXI size/len |
| I | `m_wr_req_burst` | 2 | Burst 类型 |
| I | `m_wr_req_data` | LOCAL_DATA_W | 写数据（一次性给出全部，控制器逐 beat 切到 32-bit W 通道） |
| I | `m_wr_req_strb` | LOCAL_DATA_W/8 | 字节 strobe |
| O | `m_wr_resp_en` | 1 | 脉冲：事务完成，可读取 id/resp |
| O | `m_wr_resp_id` | 4 | 事务 ID |
| O | `m_wr_resp_resp` | 2 | OKAY/SLVERR/DECERR |

**MST_WR_CTRL 写事务时序**

```
                               ┌─ size→AXI 转换、数据拆分 ─┐
                               │   OST 分配 (set)          │
                               ▼                           │
clk          _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
m_wr_req_en  __/‾‾‾\_______________________________________________
m_wr_req_addr __XXXX_______________________________________________
m_wr_req_size __XXXX_______________________________________________
m_wr_req_data __XXXX_______________________________________________

                        ┌── AW 握手, ORDER push ──┐
axi_awvalid  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________
axi_awready  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________
axi_awaddr   ___________XXXXXXXXXXXXXXXXXXXXX________________
axi_awlen    ___________XXXXXXXXXXXXXXXXXXXXX________________

                                    ┌── ORDER pop, W beats ──┐
axi_wvalid   __________________________/‾‾\_______/‾‾\_______
axi_wready   __________________________/‾‾\_______/‾‾\_______
axi_wdata    __________________________XXX________XXX________
axi_wlast    _____________________________________/‾‾\_______
                                          beat0     beat1=last

                                          ┌── B 握手, comp 清零 ──┐
axi_bvalid   ______________________________________/‾‾‾‾\______
axi_bready   _________________________________________________(1)
axi_bresp    _________________________________________XX_______

                                                       ┌── 释放 OST → resp ──┐
m_wr_resp_en _______________________________________________/‾‾\_________
m_wr_resp_resp_______________________________________________XX__________
```

**关键设计点：**
1. `m_wr_req_en` 脉冲 → OST 分配 entry，同时完成 `size→AxSIZE, AxLEN` 转换（`mst_wr_ctrl.v:317-351`）
2. AW 握手 → ORDER push（保证 W 数据按 AW 顺序发出），`req` flag 清零
3. W 数据逐 beat 切片：`wdata = data_buff[beat*32 +: 32]`，WLAST=1 后 ORDER pop
4. B 握手 → `comp` flag 清零 → `clear` 条件满足 → `m_wr_resp_en` 脉冲输出

---

### MST_RD_CTRL — Master Read Controller

将 Consumer 的读请求映射为 AXI AR/R 两通道握手。R beats 自动汇聚为 single-cycle 完整数据返回。

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `m_rd_req_en` | 1 | 脉冲：拉高一拍即发起读请求 |
| I | `m_rd_req_id` | 4 | 事务 ID（`easyaxi_cons.v:270` 固定为 0） |
| I | `m_rd_req_addr` | 16 | 起始地址 |
| I | `m_rd_req_size` | 8 | 传输字节数，自动转为 AXI size/len |
| I | `m_rd_req_burst` | 2 | Burst 类型 |
| O | `m_rd_resp_en` | 1 | 脉冲：数据返回就绪 |
| O | `m_rd_resp_id` | 4 | 事务 ID |
| O | `m_rd_resp_data` | LOCAL_DATA_W | 汇聚后的完整数据（16×32-bit → 512-bit） |
| O | `m_rd_resp_resp` | 2 | OKAY/SLVERR/DECERR |

**MST_RD_CTRL 读事务时序**

```
                               ┌─ size→AXI 转换 ─┐
                               │  OST 分配 (set)  │
                               ▼                  │
clk          _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
m_rd_req_en  __/‾‾‾\_______________________________________________
m_rd_req_addr __XXXX_______________________________________________
m_rd_req_size __XXXX_______________________________________________

                        ┌── AR 握手, ORDER push ──┐
axi_arvalid  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________
axi_arready  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________
axi_araddr   ___________XXXXXXXXXXXXXXXXXXXXX________________
axi_arlen    ___________XXXXXXXXXXXXXXXXXXXXX________________

                        │ Slave 处理延迟 │
                        |···············|
                                    ┌── ORDER pop, R beats ──┐
axi_rvalid   __________________________/‾‾\_______/‾‾\_______
axi_rready   _________________________________________________(1)
axi_rdata    __________________________XXX________XXX________
axi_rlast    _____________________________________/‾‾\_______
                                          beat0     beat1=last

                                          ┌── RLAST → 汇聚数据, comp 清零 ──┐
                                                                     ┌── 释放 OST → resp ──┐
m_rd_resp_en _____________________________________________________________/‾‾\_________
m_rd_resp_dat_____________________________________________________________XXX_________
```

**关键设计点：**
1. `m_rd_req_en` 脉冲 → OST 分配 + `size→AxSIZE, AxLEN`（与 MST_WR 对称，`mst_rd_ctrl.v:253-286`）
2. AR 握手 → ORDER push（按 `arid` 入队，支持 Out-of-Order 同 ID 保序）
3. R beats 逐 beat 存入 buffer：`data_buff[beat*32 +: 32] <= rdata`，RLAST=1 后 ORDER pop
4. `rd_buff_clr=1` 时 → 16 beats 拼接为 internal 512-bit 数据 → `m_rd_resp_en=1`

---

### SLV_WR_CTRL — Slave Write Controller

接收 AXI Master 的写事务，WLAST 后将完整 burst 数据以单拍 128-bit 写入内部 MEM。

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `axi_slv_aw{valid,ready,id,addr,len,size,burst}` | — | AXI AW 通道（宽度见 `easyaxi_define.v:15-21`） |
| I | `axi_slv_w{valid,ready,data[31:0],strb[3:0],last}` | — | AXI W 通道 |
| O | `axi_slv_b{valid,ready,id,resp}` | — | AXI B 通道 |
| O | `s_wr_data_en` | 1 | WLAST 到达后拉高一拍 |
| O | `s_wr_data_id` | 4 | 事务 ID |
| O | `s_wr_data_addr` | 16 | 写入目标地址 |
| O | `s_wr_data_data` | 128 | 汇聚后的写入数据（W beats → 128-bit internal） |
| O | `s_wr_data_strb` | 16 | 16 字节 strobe |

**SLV_WR_CTRL 写事务时序**

```
clk          _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
                        ┌── AW 握手 → OST set, ORDER push ──┐
axi_awvalid  ___________/‾‾‾‾‾‾‾‾‾‾‾‾\___________________
axi_awready  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________
axi_awaddr   ___________XXXXXXXXXXXXXX\___________________

                        │ AW-W 间可插入任意延迟 │
                                    ┌── ORDER pop → 逐 beat 缓存 ──┐
axi_wvalid   __________________________/‾‾\_______/‾‾\_______/‾‾\_______/‾‾\__
axi_wready   __________________________(1)_______(1)_______(1)_______(1)__
axi_wdata    __________________________XX________XX________XX________XX__
axi_wstrb    __________________________XX________XX________XX________XX__
axi_wlast    _____________________________________/‾‾\________
                                                     │
                                                     │ WLAST → 触发 s_wr_data_en
                                                     ▼
s_wr_data_en ________________________________________/‾‾\_______
s_wr_data_dat________________________________________XXXXX_______
                                                     (汇聚后的128-bit)

                                                              ┌── B 握手(模拟处理延迟后) → 释放OST ──┐
axi_bvalid   __________________________________________________________/‾‾‾‾\______
axi_bready   _____________________________________________________________(1)______
axi_bresp    _____________________________________________________________XX_______
```

**关键设计点：**
1. `awready = ~wr_buff_full` 做 OST 容量流控，`wready = 1` 无背压
2. W beats 逐拍缓存在 `wr_data_buff_r[entry_ptr]`，`wlast` 到达时触发 `s_wr_data_en` 脉冲
3. 模拟处理延迟（`wr_resp_get_cnt`）后返回 B 响应，随后释放 OST entry

---

### SLV_RD_CTRL — Slave Read Controller

接收 AXI AR 请求，转为脉冲式内部读请求发给 MEM。MEM 返回 128-bit 数据后自动拆分为 AXI R beats 回给 Master。

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `axi_slv_ar{valid,ready,id,addr,len,size,burst}` | — | AXI AR 通道 |
| O | `axi_slv_r{valid,ready,id,data[31:0],resp,last}` | — | AXI R 通道 |
| O | `s_rd_req_en` | 1 | AR 握手后拉高一拍，发起内部读 |
| O | `s_rd_req_id` | 4 | 事务 ID |
| O | `s_rd_req_addr` | 16 | 读目标地址 |
| I | `s_rd_resp_en` | 1 | MEM 返回数据脉冲 |
| I | `s_rd_resp_id` | 4 | 事务 ID |
| I | `s_rd_resp_data` | 128 | MEM 返回的 128-bit 数据，自动拆为 R beats |
| I | `s_rd_resp_resp` | 2 | 读响应状态 |

**SLV_RD_CTRL 读事务时序**

```
clk          _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
                        ┌── AR 握手 → OST set, ORDER push ──┐
axi_arvalid  ___________/‾‾‾‾‾‾‾‾‾‾‾‾\___________________
axi_arready  ___________/‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________
axi_araddr   ___________XXXXXXXXXXXXXX\___________________

                        │                                      │
s_rd_req_en  ___________/‾‾\___________________________________
s_rd_req_addr___________XXXXX___________________________________
                        │                                      │
                        │  MEM 读取延迟（由SRAM内部时序决定）     │
                        │                                      │
s_rd_resp_en ________________________________/‾‾\______________
s_rd_resp_dat________________________________XXXXX_____________
                                              (128-bit)
                                              │ 128-bit → 4×32-bit 拆分 │
                                              ▼
                                    ┌── ORDER pop → R beats ──┐
axi_rvalid   __________________________/‾‾\_______/‾‾\_______/‾‾\_______/‾‾\__
axi_rready   __________________________(1)_______(1)_______(1)_______(1)__
axi_rdata    __________________________XX________XX________XX________XX__
axi_rlast    _____________________________________/‾‾\________
                                                     │RLAST → 释放OST
```

**关键设计点：**
1. `arready = ~rd_buff_full` 做 OST 容量流控
2. AR 握手 → 同时发出 `s_rd_req_en` ← 单拍脉冲触发内部 MEM 读
3. `s_rd_resp_en` 到达后，128-bit 数据拆为 4×32-bit R beats，逐拍由 ORDER 控制的 `rd_result_ptr` 输出
4. RLAST=1 后 `comp` flag 清零 → 释放 OST entry

---

### 上层使用速查

```
Producer:  m_wr_req_en + addr + size + data + strb  →  等待 m_wr_resp_en
Consumer:  m_rd_req_en + addr + size                 →  等待 m_rd_resp_en + data
MEM:       s_wr_data_en + data + strb                →  写入存储
           s_rd_req_en + addr                        →  s_rd_resp_en + data
```

四个控制器自动处理：AXI 握手、burst 拆分/汇聚、OST 管理、同 ID 保序、WRAP 地址计算。

> **详细参考**：`doc/axi_ctrl_spec.md` — 包含 OST 原理（第 2 章）、ORDER 保序机制（第 3 章）、Burst Addressing（第 4 章）及各控制器完整数据流（第 5-8 章）和 AXI 侧完整接口表（Appendix B）。

## 波形观察指南

用 Verdi 打开波形后，加载 signal.rc 即可看到 8 个分组。以下是各分组的观察要点：

---

### 1. TOP CTRL — 顶层控制信号

```
prod_start  ──┐
prod_done   <──┘ Producer 完成 N 个 Data Block + N 个 CQE
cons_start  ──┐
cons_done   <──┘ Consumer 完成 N 个 CQE 轮询 + N 个 Data Block 校验
cons_error  ── 出现数据不匹配时拉高
```

**观察流程：**
1. `prod_start` 拉高 → 等待 `prod_done` 拉高
2. `cons_start` 拉高 → 等待 `cons_done` 拉高
3. 全程 `cons_error` 保持低电平

---

### 2. PRODUCER — 生产者状态机

**State & Ctrl 分组：**
- `state_r`：状态机
  - `0`=IDLE → `1`=WRITE_DATA → `2`=WAIT_BRESP → `3`=WRITE_CQE → `4`=WAIT_CQE → `5`=NEXT → `6`=DONE → `7`=PROD_GAP
- `idx_r`：当前处理的 Block 索引（0~N-1）
- `cqe_written`：WAIT_CQE 收到 B 响应时拉高一拍，表示一个 CQE 已写入 SRAM
- `w_size`：`idx_r % 5` 决定 block 大小（1/2/4/8/16 字节轮转）

**观察要点：**
- 每个 idx：先写 Data Block（WRITE_DATA → WAIT_BRESP），再写 CQE（WRITE_CQE → WAIT_CQE）
- idx 从 0 递增到 N-1，然后进入 DONE
- `w_size` 在 idx=0 时为 1 字节，idx=1 时为 2 字节，依此类推

**AXI AW/W/B 分组：**
- AW 通道：`awaddr` = `0x10 + idx*0x10`（Data Block）或 `0x00 + idx`（CQE）
- W 通道：`wdata` 每个 word = `{SIZE, byte_ofs, 16'h0, idx}`，`wlast` 在最后一个 beat 拉高
- B 通道：每个写事务完成后收到 B response

---

### 3. CONSUMER — 消费者状态机

**State & Ctrl 分组：**
- `state_r`：状态机
  - `0`=IDLE → `1`=POLL_WAIT → `2`=READ_CQE → `3`=WAIT_CQE → `4`=CHECK_CQE → `5`=READ_DATA → `6`=WAIT_DATA → `7`=VERIFY → `8`=NEXT → `9`=DONE
- `idx_r`：当前轮询的 Block 索引
- `poll_cnt_r`：轮询间隔计数器（每 16 个周期读一次 CQE）
- `cqe_valid`：CHECK_CQE 时 CQE 的 VALID 位为 1
- `cqe_invalid`：CHECK_CQE 时 CQE 的 VALID 位为 0，波形上高电平即为 MISS
- `r_size`：从 CQE 解码的 block 大小，与 Producer 的 `w_size` 一致

**观察要点：**
- 每 16 个周期读一次 CQE 表（READ_CQE → WAIT_CQE）
- CHECK_CQE 状态：检查读回的 CQE 数据 bit[0]（VALID）
  - VALID=0 → 回到 POLL_WAIT 继续轮询
  - VALID=1 → 进入 READ_DATA 读取 Data Block
- 读回 Data Block 后在 VERIFY 状态校验每个 word[3:0] 是否等于 idx

**AXI AR/R 分组：**
- AR 通道：`araddr` = `0x00`（读 CQE 表，固定读整个 16B 表）或 `0x10 + idx*0x10`（Data Block）
- R 通道：`rdata` 返回数据，`rlast` 在最后一个 beat 拉高

---

### 4. SRAM — Slave 侧 AXI 信号

由于 TOP 中 Producer 和 Consumer 的 AXI Master 信号直连到 SRAM 的 Slave 端口，所以 `axi_slv_*` 信号与 `axi_mst_*` 信号完全一致。

**观察要点：**
- 验证 AXI 协议握手是否正确（valid/ready 握手）
- 观察 AW 和 AR 是否交错出现（Producer 写 + Consumer 读）

---

### 5. 内部 — SRAM 内部 内部 接口

SLV_WR_CTRL 到 MEM 的写数据通道：
- `s_wr_data_en`：写数据有效
- `s_wr_data_addr`：写入地址
- `s_wr_data_data[127:0]`：写入数据（128-bit 内部接口）

SLV_RD_CTRL 到 MEM 的读写通道：
- `s_rd_req_en`：读请求
- `s_rd_resp_en`：读响应有效
- `s_rd_resp_data[127:0]`：读回数据（128-bit 内部接口）

---

### 6. MEMORY — 内存模块

直接观察内存的读写端口，确认数据是否正确写入和读出。

---

### 7. PROD INT — MST WR CTRL 内部

- `wr_req_en`：AW 通道握手成功（awvalid & awready）
- `wr_data_en`：W 通道握手成功（wvalid & wready）
- `wr_data_last`：wlast 信号
- `wr_id_buff_r[0]`、`wr_addr_buff_r[0]`、`wr_len_buff_r[0]`、`wr_size_buff_r[0]`：当前 OST 条目 0 的请求信息
- `wr_data_cnt_r[0]`：当前 OST 条目 0 已发送的 W data beat 数
- `wr_resp_buff_r[0]`：当前 OST 条目 0 的 B response

**观察要点：**
- `wr_req_en` 拉高时，`wr_*_buff_r[0]` 显示当前请求的 ID/ADDR/LEN/SIZE
- `wr_data_cnt_r[0]` 从 0 递增到 `wr_len_buff_r[0]`，然后 `wr_data_last` 拉高
- 收到 B response 后，该 OST 条目完成

---

### 8. CONS INT — MST RD CTRL 内部

- `rd_req_en`：AR 通道握手成功（arvalid & arready）
- `rd_result_en`：R 通道握手成功（rvalid & rready）
- `rd_result_last`：rlast 信号
- `rd_id_buff_r[0]`、`rd_addr_buff_r[0]`、`rd_len_buff_r[0]`、`rd_size_buff_r[0]`：当前 OST 条目 0 的请求信息
- `rd_data_cnt_r[0]`：当前 OST 条目 0 已接收的 R data beat 数
- `rd_resp_buff_r[0]`：当前 OST 条目 0 的 R response

---

## 典型波形观察流程

1. **看 TOP CTRL**：确认 prod_start→prod_done→cons_start→cons_done 的时序
2. **看 PRODUCER state**：确认每个 idx 先写 Data Block 再写 CQE
3. **看 AXI AW/W/B**：确认写事务的地址、数据、burst 长度正确
4. **看 CONSUMER state**：确认轮询 CQE、读到 VALID=1 后读 Data Block
5. **看 AXI AR/R**：确认读事务的地址与 Producer 写的地址一致
6. **看 MEMORY**：确认数据正确写入和读出
7. **看 PROD/CONS INT**：确认 MST_WR/RD_CTRL 内部状态机正常流转

## 运行测试

```bash
cd S01E09 && source script/project.sh
cd verification/sim && make run
```

预期输出：`All tests passed!`，仿真过程中会打印 CQE HIT/MISS 实时日志和重轮询统计。

### 测试配置

| 属性 | 值 | 说明 |
|------|-----|------|
| Block 数 | 4 | 在 testbench 中通过 `PROD_NUM_BLOCKS` / `CONS_NUM_BLOCKS` 设置，可配置 1~15 |
| Block 大小 | 1B/2B/4B/8B | 按 `idx % 5` 循环变化（idx=0→1B, idx=1→2B, idx=2→4B, idx=3→8B） |
| Burst 类型 | INCR | 默认值，可通过 `BURST_TYPE` 参数改为 FIXED/WRAP |
| Producer 间隔 | `20 + idx*18` | 由 `PROD_GAP_EN=1` 开启；idx=0 HIT，idx=1/2/3 触发 MISS 重轮询 |
| Consumer 轮询 | 每 16 cycle | 读一次 CQE 表 |

### Producer/Consumer 参数说明

Producer 和 Consumer 模块支持参数化配置，默认值匹配本 demo 场景：

| 模块 | 参数 | 默认值 | 作用 |
|------|------|--------|------|
| EASYAXI_PROD | `BURST_TYPE` | `AXI_BURST_INCR` | 改为 FIXED/WRAP 可测试不同 burst 类型 |
| EASYAXI_PROD | `PROD_GAP_EN` | `1` | `=0` 关闭生产间隔，做背靠背压力测试 |
| EASYAXI_PROD | `SIZE_MODE` / `FIXED_SIZE` | `0` / `16` | `=1` 时固定 block 大小（覆盖 32B/64B/128B） |
| EASYAXI_CONS | `BURST_TYPE` | `AXI_BURST_INCR` | 需与 PROD 保持一致 |
| EASYAXI_CONS | `SIZE_MODE` / `FIXED_SIZE` | `0` / `16` | `=1` 时绕过 CQE 解码，直接使用固定 size |

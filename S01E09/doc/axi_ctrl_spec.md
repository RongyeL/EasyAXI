# EasyAXI Controller Specification

## 1. Architecture Overview

EasyAXI 使用四层控制器将上层业务逻辑与 AXI 协议解耦。Master 侧控制器将简洁的请求/响应接口转换为标准 AXI 总线信号；Slave 侧控制器接收 AXI 总线信号，通过内部请求/响应通道与 SRAM/MEM 交互。

```
┌──────────────────┐          ┌──────────────────┐
│     Producer     │          │     Consumer     │
│  (ez_easyaxi_prod)          │  (ez_easyaxi_cons)
└────────┬─────────┘          └────────┬─────────┘
         │ m_wr_req/resp               │ m_rd_req/resp
┌────────┴─────────┐          ┌────────┴─────────┐
│   MST_WR_CTRL    │          │   MST_RD_CTRL    │  ← Master Side
└────────┬─────────┘          └────────┬─────────┘
         │ AXI AW/W/B                   │ AXI AR/R
┌────────┴─────────┐          ┌────────┴─────────┐
│   SLV_WR_CTRL    │          │   SLV_RD_CTRL    │  ← Slave Side
└────────┬─────────┘          └────────┬─────────┘
         │ s_wr_data_*                 │ s_rd_req/resp_*
┌────────┴────────────────────────────┴─────────┐
│                   SRAM / MEM                   │
└───────────────────────────────────────────────┘
```

**Master 侧**：对外暴露简洁的请求/响应接口。上层只需提供 addr/size/data，控制器负责 AXI 握手、burst 拆分/汇聚、OST 管理。

**Slave 侧**：接收 AXI 总线流量。每个写 burst 汇聚为单拍写请求输出给 MEM；每个读请求对应一次 MEM 读操作，数据返回后拆分为 AXI burst beats 返回 Master。

**内部请求/响应通道**：Slave 与 MEM 之间的数据通路。写通道 128-bit 数据 + 16-byte strobe；读通道 128-bit 数据。AXI 32-bit ↔ 内部 128-bit 的位宽转换由控制器自动处理。

> 四个控制器共享 OST、Ordering、Burst Addressing 三套机制。下文第 2-4 章集中说明这些共享概念，后续各章节不再重复。

---

## 1.1 Quick Reference -- Controller Interfaces

视频讲解时可通过下表快速理解四个控制器的接口设计思路。

### MST_WR_CTRL -- Master Write Controller

将上层"发出写请求、收到写响应"两个动作映射为 AXI AW/W/B 三通道握手。上层无需关心 burst 拆分、OST 管理、写数据保序。

| 方向 | 信号 | 位宽 | 视频讲解要点 |
|------|------|------|-------------|
| I | `m_wr_req_en` | 1 | 脉冲：拉高一拍即发起写请求 |
| I | `m_wr_req_addr` | 16 | 起始地址（字节地址） |
| I | `m_wr_req_size` | 8 | 传输字节数 1/2/4/8/16/32/64/128，控制器自动转为 AXI size/len |
| I | `m_wr_req_data` | LOCAL_DATA_W | 写数据（一次性给出全部，控制器逐 beat 拆分到 32-bit AXI W 通道） |
| I | `m_wr_req_strb` | LOCAL_DATA_W/8 | 字节 strobe |
| O | `m_wr_resp_en` | 1 | 脉冲：事务完成，可读取 id/resp |
| O | `m_wr_resp_id` | 4 | 事务 ID |
| O | `m_wr_resp_resp` | 2 | OKAY/SLVERR/DECERR |

> **视频 Demo 引用**：`easyaxi_prod.v:233-273` — Producer 通过 10 个信号驱动 MST_WR_CTRL，无需触碰 AW/W/B 三通道。

### MST_RD_CTRL -- Master Read Controller

将上层"发出读请求、等待读数据返回"两个动作映射为 AXI AR/R 两通道握手。R beats 自动汇聚为 single-cycle 数据返回。

| 方向 | 信号 | 位宽 | 视频讲解要点 |
|------|------|------|-------------|
| I | `m_rd_req_en` | 1 | 脉冲：拉高一拍即发起读请求 |
| I | `m_rd_req_addr` | 16 | 起始地址 |
| I | `m_rd_req_size` | 8 | 传输字节数，控制器自动转为 AXI size/len |
| O | `m_rd_resp_en` | 1 | 脉冲：数据返回就绪 |
| O | `m_rd_resp_data` | LOCAL_DATA_W | 汇聚后的完整数据（R beats 自动拼接） |

> **视频 Demo 引用**：`easyaxi_cons.v:261-296` — Consumer 通过 6 个信号驱动 MST_RD_CTRL。

### SLV_WR_CTRL -- Slave Write Controller

接收 AXI AW/W/B 三通道，WLAST 后将完整 burst 数据以单拍 128-bit 写入内部 MEM。

| 方向 | 信号 | 位宽 | 视频讲解要点 |
|------|------|------|-------------|
| O | `s_wr_data_en` | 1 | WLAST 后拉高一拍，表示完整 burst 数据已就绪 |
| O | `s_wr_data_data` | 128 | 汇聚后的写入数据（AXI W beats -> internal 128-bit） |
| O | `s_wr_data_strb` | 16 | 与 data 对应的字节 strobe |

> **视频 Demo 引用**：`easyaxi_sram.v:124-158` — SRAM 封装了 SLV_WR_CTRL，将 AXI 写流量转为内部写接口。

### SLV_RD_CTRL -- Slave Read Controller

接收 AXI AR，转为脉冲式内部读请求。MEM 返回 128-bit 数据后自动拆分为 AXI R beats 回给 Master。

| 方向 | 信号 | 位宽 | 视频讲解要点 |
|------|------|------|-------------|
| O | `s_rd_req_en` | 1 | AR 握手后拉高一拍，发起内部读 |
| O | `s_rd_req_addr` | 16 | 读地址 |
| I | `s_rd_resp_en` | 1 | MEM 读数据返回脉冲 |
| I | `s_rd_resp_data` | 128 | MEM 返回的 128-bit 数据，控制器自动拆为 R beats |

> **视频 Demo 引用**：`easyaxi_sram.v:89-119` — SRAM 封装了 SLV_RD_CTRL。

### 速查总结：上层只需关心什么

```
Producer:  m_wr_req_en + addr + size + data  -->  等待 m_wr_resp_en
Consumer:  m_rd_req_en + addr + size         -->  等待 m_rd_resp_en + data
MEM:       s_wr_data_en + data + strb        -->  写入存储
           s_rd_req_en + addr                -->  s_rd_resp_en + data
```

四个控制器自动完成：AXI 握手、burst 拆分/汇聚、OST 管理、同 ID 保序、WRAP 地址计算。

---

## 2. OST — Outstanding Transaction

### 2.1 为什么需要 OST

AXI 协议允许 Master 在收到前一笔事务的响应之前，连续发送多个请求。这种"未完成事务"称为 Outstanding Transaction。OST 用流水线隐藏延迟——当第 1 笔事务等待 MEM 响应时，第 2、3 笔已经发出。

### 2.2 OST Entry 结构

每个 OST Entry 是一组独立的寄存器阵列（使用 `generate for` 例化 OST_DEPTH 份），包含：

| 字段 | 位宽 | 作用 |
|------|------|------|
| `valid` | 1 | Entry 已被占用 |
| `req` | 1 | AW/AR 请求已发送到总线 |
| `comp` | 1 | 数据/响应尚未返回 |
| `clear` | 1 | 事务已完成，等待释放 |
| `id/addr/len/size/burst` | 各不同 | 事务 payload |
| `data_buff` | internal/AXI 宽度 × MAX_BURST | beat 数据缓存 |
| `data_cnt` | $clog2(MAX_BURST) | 已传输的 beat 计数 |

四个 flag 组成状态机：

```
         alloc     req_sent   data_done    resp_rcvd
  IDLE ───→ VALID ───→ REQ'd ───→ COMPLETE ───→ CLEAR ───→ IDLE
           (占用)    (已发)    (数据已收)   (可释放)
```

- `valid=1` 表示 entry 被占用
- `req=1` 表示 AW/AR 已成功握手，等待数据通道完成
- `comp=1` 表示 W/R 通道仍在传输中（写：WLAST 未到；读：RLAST 未到）
- `clear=1` 且 `valid=1, req=0, comp=0` 时，entry 被释放，同时向上层输出响应

### 2.3 ARB 分配器

每个控制器包含 2~3 个 EASYAXI_ARB 实例，分别负责 entry 的分配（set）、请求发送（req）、响应返回（result/clear）。

ARB 采用轮转优先级（round-robin）：
- 输入：`queue_i[DEEP_NUM-1:0]` —— 候选 bitmap
- 输出：`pointer_o` —— 选中的 entry 编号
- 保证公平：上次选中的位置在下一次调度时优先级最低

关键用法：
```
wr_buff_set = ~wr_buff_full & m_wr_req_en    // 有空位且外部请求到达时分配新 entry
wr_set_bits = ~wr_valid_bits                  // 空闲 entry 的 bitmap → 送入 ARB
wr_set_ptr  = ARB(wr_set_bits)               // ARB 选出下一个可用的 entry
```

### 2.4 跨控制器差异

四个控制器对 OST entry 的使用方式完全一致（valid/req/comp/clear + ARB + payload buffer），差异仅在于：

| 控制器 | 写通道 OST | 额外的 buffer |
|--------|----------|-------------|
| MST_WR | AW+W+B 流水 | `wr_data_buff`（internal→AXI 拆分）, `wr_strb_buff` |
| MST_RD | AR+R 流水 | `rd_data_buff`（AXI→internal 汇聚） |
| SLV_WR | AW+W+B 流水 | `wr_data_buff`（beat 缓存）, `wr_curr_addr`（WRAP 地址计算） |
| SLV_RD | AR+R 流水 | `rd_data_buff`（internal→AXI 拆分）, `rd_curr_addr`（WRAP 地址计算） |

---

## 3. Ordering — 保序机制

### 3.1 为什么需要保序

AXI 规范要求：
- **写通道**：W 数据 beats 的发送顺序必须与 AW 地址的顺序一致
- **读通道**：R 数据可以乱序返回（Out-of-Order），但同 ID 必须严格保序
- **响应通道**：B 响应的返回顺序应按 AW 顺序（同 ID 保序）

### 3.2 EASYAXI_ORDER 工作原理

EASYAXI_ORDER 为每个 AXI ID 维护一个独立的 FIFO，FIFO 中存储的是 OST entry 指针：

```
push:  awvalid & awready → 将 wr_set_ptr 写入 FIFO[awid]
pop:   wvalid & wready & wlast → 从 FIFO[0] 中读取下一个 wr_data_ptr
```

关键信号：
| 信号 | 方向 | 含义 |
|------|------|------|
| `push` / `push_id` / `push_ptr` | I | 将 entry 指针按 ID 入队 |
| `pop` / `pop_id` / `pop_last` | I | 当前事务完成，按 ID 出队 |
| `order_ptr` | O | 下一个允许执行的 entry 指针 |
| `order_bits` | O | 所有已入队但未出队的 entry bitmap |

**用法示例**（MST_WR_CTRL 写数据保序）：
```
// W 通道只有在 ORDER 允许且该 entry 数据就绪时才拉高 wvalid
axi_mst_wvalid = ~wr_data_order_empty          // ORDER 中有待发数据
               & wr_valid_buff_r[wr_data_ptr]   // entry 有效
               & wr_dat_buff_r[wr_data_ptr];    // 数据已就绪
```

### 3.3 各控制器的 ORDER 实例

每个控制器使用 2 个 ORDER 实例：

| 控制器 | ORDER #1（数据通道） | ORDER #2（响应通道） |
|--------|-------------------|-------------------|
| MST_WR | AW→W：push=AW握手, pop=W握手+wlast | AW→B：push=AW握手+awid, pop=B握手+bid |
| MST_RD | — | AR→R：push=AR握手+arid, pop=R握手+rlast |
| SLV_WR | AW→W：push=AW握手, pop=W握手+wlast | AW→B：push=AW握手+awid, pop=B握手+bid |
| SLV_RD | — | AR→R：push=AR握手+arid, pop=R握手+rlast |

> MST_RD 和 SLV_RD 不需要数据通道 ORDER，因为读数据方向是从 Slave 回 Master，数据自然按 R 通道返回。

---

## 4. Burst Addressing — Burst 地址计算

### 4.1 三种 Burst 类型

| 类型 | 编码 | 行为 |
|------|------|------|
| FIXED | 2'b00 | 每个 beat 使用相同地址（FIFO 类访问） |
| INCR | 2'b01 | 地址逐 beat 递增，增量为每 beat 字节数 |
| WRAP | 2'b10 | 到达 wrap boundary 后回绕到起始地址 |

### 4.2 地址计算参数

给定一笔 burst 事务：
- `start_addr` = 请求的起始地址
- `num_bytes`  = 2^AxSIZE（每 beat 传输的字节数）
- `burst_len`  = AxLEN + 1（总 beat 数）
- `aligned_addr` = start_addr 按 num_bytes 对齐

**FIXED**：每个 beat 地址 = start_addr（不变）

**INCR**：第 N 个 beat 的地址 = aligned_addr + N × num_bytes

**WRAP**：
```
wrap_boundary = start_addr / (burst_len × num_bytes) × (burst_len × num_bytes)

if (current_addr + num_bytes == wrap_boundary + burst_len × num_bytes)
    next_addr = wrap_boundary                    // 回绕
else if (已回绕过)
    next_addr = start_addr + N × num_bytes - burst_len × num_bytes
else
    next_addr = aligned_addr + N × num_bytes
```

### 4.3 Size → Burst Length 映射

上层请求以字节为单位指定 size（1~128B），控制器内部自动转换为 AXI AxSIZE 和 AxLEN：

| 请求 Size | AXI AxSIZE | Beats | AxLEN |
|----------|-----------|-------|-------|
| 1B | 3'b000 | 1 | 0 |
| 2B | 3'b001 | 1 | 0 |
| 4B | 3'b010 | 1 | 0 |
| 8B | 3'b011 | 2 | 1 |
| 16B | 3'b100 | 4 | 3 |
| 32B | 3'b101 | 8 | 7 |
| 64B | 3'b110 | 16 | 15 |
| 128B | 3'b111 | 32 | 31 |

> 1B/2B 请求在 AXI 总线上仍占用 1 个 beat（32-bit），通过 wstrb 信号标记有效字节。

---

## 5. EASYAXI_MST_WR_CTRL — Master Write Controller

### 5.1 职责

将上层（Producer）的 internal 宽度（512-bit）写请求拆分为 AXI 32-bit burst beats，管理 AW/W/B 三通道握手的 OST 流水线。

### 5.2 数据流

```
上层请求 (m_wr_req_en + size + data[511:0])
    │
    ▼
┌──────────────────────────────────────┐
│ 1. OST 分配                          │
│    wr_buff_set → ARB 选 free entry   │
│    写入: id/addr/len/size/burst/data │
│    Size(byte) 转 AXI size/len        │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 2. AW 通道                           │
│    awvalid = |wr_req_bits            │
│    awaddr  = wr_addr_buff_r[wr_req_ptr] │
│    AW握手 → req flag 清零            │
│    AW握手 → ORDER push (entry ptr)   │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 3. W 通道（保序输出）                 │
│    ORDER pop → wr_data_ptr           │
│    wdata = wr_data_buff[wr_data_ptr][beat*32 +: 32] │
│    wstrb = wr_strb_buff[wr_data_ptr][beat*4 +: 4]  │
│    wlast = (wr_data_cnt == wr_len)   │
│    每个 beat 握手后 data_cnt++       │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 4. B 通道（响应回收）                 │
│    bready = 1（始终就绪）            │
│    B握手 → ORDER pop (by bid)        │
│    B握手 → comp flag 清零            │
│    记录 bresp（累积最高错误级别）     │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 5. 释放 OST entry + 输出响应         │
│    条件: valid=1, req=0, comp=0      │
│    m_wr_resp_en = 1                  │
│    m_wr_resp_id = wr_id_buff_r[clr_ptr] │
│    m_wr_resp_resp = 累积的 bresp     │
└──────────────────────────────────────┘
```

### 5.3 关键设计点

**W 数据保序**：必须按 AW 通道的顺序发送 W 数据。ORDER #1 在每次 AW 握手时 push entry ptr，在 WLAST 时 pop，order_ptr 指向当前允许发送的 entry。

**数据拆分**：internal 512-bit 数据在一次请求中全部给出。控制器逐 beat 取出 32-bit 切片：
```
wdata = wr_data_buff_r[wr_data_ptr][(wr_data_cnt*32) +: 32]
```

**错误累积**：同一笔事务如果收到多个 B 响应，选择最高严重级别（SLVERR > DECERR > OKAY）在最终响应中报告。

---

## 6. EASYAXI_MST_RD_CTRL — Master Read Controller

### 6.1 职责

将上层（Consumer）的读请求转换为 AXI AR 通道请求，将返回的 R 通道 burst beats 汇聚为 internal 512-bit 单拍数据，支持 Out-of-Order 返回。

### 6.2 数据流

```
上层请求 (m_rd_req_en + addr + size)
    │
    ▼
┌──────────────────────────────────────┐
│ 1. OST 分配                          │
│    rd_buff_set → ARB 选 free entry   │
│    写入: id/addr/len/size/burst      │
│    (无 data —— 读请求不需要带数据)    │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 2. AR 通道                           │
│    arvalid = |rd_req_bits            │
│    araddr  = rd_addr_buff_r[rd_req_ptr] │
│    AR握手 → req flag 清零            │
│    AR握手 → ORDER push (entry+arid)  │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 3. R 通道（按 ORDER 接收）            │
│    rready = 1（始终就绪）            │
│    ORDER pop (by rid) → rd_result_ptr│
│    rdata 存入 rd_data_buff[rd_result_ptr][beat*32 +: 32] │
│    rresp 累积最高错误级别             │
│    每个 beat 握手后 data_cnt++       │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 4. RLAST → 汇聚 + 释放               │
│    RLAST 到达 → comp flag 清零       │
│    16×32-bit beats → 512-bit internal 数据│
│    OST entry 进入 clear 状态         │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 5. 输出响应                           │
│    m_rd_resp_en = 1                  │
│    m_rd_resp_data = 汇聚后的 internal 数据 │
└──────────────────────────────────────┘
```

### 6.3 关键设计点

**Out-of-Order 支持**：AXI 允许不同 ID 的 R 数据以任意顺序返回。ORDER 按 AR 通道的 (arid, entry_ptr) 顺序记录，R 通道收到 (rid, rlast) 时出队。如果 rid=2 的数据先于 rid=1 返回，ORDER 会阻塞 rid=1 对应的 entry，直到 rid=0 先出队。

**数据汇聚**：`rd_data_buff_r[rd_clr_ptr]` 在 RLAST 到达后包含完整的 burst 数据，16 个 32-bit beats 拼接为 512-bit internal 数据输出。

**与 MST_WR 的差异**：读控制器不需要管理 W 通道的数据保序（因为 R 通道由 Slave 驱动），ORDER 仅用于响应保序。

---

## 7. EASYAXI_SLV_WR_CTRL — Slave Write Controller

### 7.1 职责

接收 AXI Master 的写事务。缓存 W 通道的 burst beats，wlast 到达后将完整数据通过内部接口输出给 MEM，处理完成后返回 B 响应。

### 7.2 数据流

```
AXI AW 通道 (awvalid & awready)
    │
    ▼
┌──────────────────────────────────────┐
│ 1. OST 分配                          │
│    awready = ~wr_buff_full           │
│    握手 → 分配 entry                │
│    记录: awid/awaddr/awlen/awsize/awburst │
│    AW握手 → ORDER push (entry ptr)   │
└──────────────┬───────────────────────┘
               ▼
AXI W 通道 (wvalid & wready)
    │
    ▼
┌──────────────────────────────────────┐
│ 2. 逐 beat 缓存 W 数据               │
│    wready = 1（始终就绪）            │
│    ORDER pop → wr_data_ptr           │
│    wr_curr_index 递增               │
│    存入 wr_data_buff[wr_data_ptr][(curr_idx-1)*32 +: 32] │
│    存入 wr_data_strb[...]            │
│    burst 地址计算（FIXED/INCR/WRAP）│
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 3. WLAST → 内部数据输出                  │
│    触发 wr_data_last_r 脉冲          │
│    internal: s_wr_data_en 拉高一拍        │
│    s_wr_data_data = wr_data_buff[wr_data_ptr][127:0]  │
│    (internal 使用 128-bit，超出部分截断)   │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 4. B 响应                            │
│    模拟处理延迟（wr_resp_get_cnt）    │
│    bvalid = wr_valid & wr_result     │
│    B握手 → ORDER pop (by bid)        │
│    B握手 → comp flag 清零            │
│    释放 OST entry                    │
└──────────────────────────────────────┘
```

### 7.3 关键设计点

**Burst 地址计算**：Slave 侧需要根据 AW 参数实时计算每个 beat 对应的地址。FIXED/INCR/WRAP 三种模式的地址计算由第 4 章的公式驱动。

**WLAST 触发 内部数据输出**：内部写不是逐 beat 的，而是整个 burst 接收完毕后单拍发出。`wr_data_last_r` 在 WLAST 握手成功后拉高一拍，触发写数据和后续 B 响应流程。

**流控**：当前实现 `awready=~wr_buff_full` 做 OST 容量流控，`wready=1`（无背压）。

---

## 8. EASYAXI_SLV_RD_CTRL — Slave Read Controller

### 8.1 职责

接收 AXI Master 的读请求，转换为 internal 读请求发给 MEM。MEM 返回 128-bit 数据后拆分为 32-bit AXI R 通道 burst beats。

### 8.2 数据流

```
AXI AR 通道 (arvalid & arready)
    │
    ▼
┌──────────────────────────────────────┐
│ 1. OST 分配 + internal 读请求              │
│    arready = ~rd_buff_full           │
│    握手 → 分配 entry                │
│    记录: arid/araddr/arlen/arsize/arburst │
│    AR握手 → ORDER push (entry+arid)  │
│    AR握手 → s_rd_req_en 拉高一拍 │
└──────────────┬───────────────────────┘
               ▼
         MEM 处理 (潜伏期)
               │
               ▼
┌──────────────────────────────────────┐
│ 2. internal 读响应 → 拆分数据              │
│    s_rd_resp_en → 触发 rd_data_en    │
│    128-bit internal 数据 → rd_data_buff   │
│    rd_data_vld 全部标记为有效         │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐
│ 3. R 通道逐 beat 输出                 │
│    ORDER pop → rd_result_ptr         │
│    rvalid = rd_valid & rd_result     │
│    rdata = rd_data_buff[rd_result_ptr][beat*32 +: 32] │
│    rlast = (rd_data_cnt == rd_len)   │
│    每个 beat 握手后 data_cnt++       │
└──────────────┬───────────────────────┘
               ▼
         RLAST → comp flag 清零
               → 释放 OST entry
```

### 8.3 关键设计点

**Pulse-based 触发**：与 SLV_WR 的 WLAST 触发不同，SLV_RD 使用 internal 响应脉冲 `s_rd_resp_en` 触发数据拆分。`rd_data_en_pulse_r` 在 internal 响应到达后拉高一拍，启动 R 通道 beat 输出。

**RLAST 判断**：`rlast = rvalid & (rd_data_cnt[rd_result_ptr] == rd_len_buff_r[rd_result_ptr])`。当已发送 beat 数等于 burst 长度时，最后一拍拉高 rlast。

**与 SLV_WR 的对称性**：SLV_WR 汇聚 beats → 内部数据输出；SLV_RD 拆分 internal → beats 输出。两者对 OST entry 的管理方式完全一致，区别仅在于数据流向相反。

---

## Appendix A. Parameter Reference

| 参数 | 默认值 | 适用模块 | 说明 |
|------|--------|---------|------|
| `OST_DEPTH` | 16 | 全部四个 | Outstanding 深度，须为 2 的幂 |
| `LOCAL_DATA_W` | 512 | MST_WR, MST_RD | 内部请求/响应数据位宽 |

---

## Appendix B. Interface Reference

### B.1 EASYAXI_MST_WR_CTRL

**Request Side**（连接 Producer）

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `m_wr_req_en` | 1 | 写请求有效 |
| I | `m_wr_req_id` | 4 | 事务 ID |
| I | `m_wr_req_addr` | 16 | 起始地址 |
| I | `m_wr_req_size` | 8 | 传输字节数 (1/2/4/8/16/32/64/128) |
| I | `m_wr_req_burst` | 2 | Burst 类型 |
| I | `m_wr_req_data` | LOCAL_DATA_W | 写数据 |
| I | `m_wr_req_strb` | LOCAL_DATA_W/8 | 字节 strobe |

**Response Side**

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| O | `m_wr_resp_en` | 1 | 响应有效 |
| O | `m_wr_resp_id` | 4 | 事务 ID |
| O | `m_wr_resp_resp` | 2 | OKAY/SLVERR/DECERR |

**AXI Side**

| 方向 | 信号 | 位宽 | 通道 |
|------|------|------|------|
| O | `axi_mst_awvalid` | 1 | AW |
| I | `axi_mst_awready` | 1 | AW |
| O | `axi_mst_awid` | 4 | AW |
| O | `axi_mst_awaddr` | 16 | AW |
| O | `axi_mst_awlen` | 8 | AW |
| O | `axi_mst_awsize` | 3 | AW |
| O | `axi_mst_awburst` | 2 | AW |
| O | `axi_mst_awuser` | 5 | AW |
| O | `axi_mst_wvalid` | 1 | W |
| I | `axi_mst_wready` | 1 | W |
| O | `axi_mst_wdata` | 32 | W |
| O | `axi_mst_wstrb` | 4 | W |
| O | `axi_mst_wlast` | 1 | W |
| O | `axi_mst_wuser` | 5 | W |
| I | `axi_mst_bvalid` | 1 | B |
| O | `axi_mst_bready` | 1 | B |
| I | `axi_mst_bid` | 4 | B |
| I | `axi_mst_bresp` | 2 | B |
| I | `axi_mst_buser` | 5 | B |

### B.2 EASYAXI_MST_RD_CTRL

**Request Side**（连接 Consumer）

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| I | `m_rd_req_en` | 1 | 读请求有效 |
| I | `m_rd_req_id` | 4 | 事务 ID |
| I | `m_rd_req_addr` | 16 | 起始地址 |
| I | `m_rd_req_size` | 8 | 传输字节数 |
| I | `m_rd_req_burst` | 2 | Burst 类型 |

**Response Side**

| 方向 | 信号 | 位宽 | 说明 |
|------|------|------|------|
| O | `m_rd_resp_en` | 1 | 响应有效 |
| O | `m_rd_resp_id` | 4 | 事务 ID |
| O | `m_rd_resp_data` | LOCAL_DATA_W | 读回数据 |
| O | `m_rd_resp_resp` | 2 | OKAY/SLVERR/DECERR |

**AXI Side**

| 方向 | 信号 | 位宽 | 通道 |
|------|------|------|------|
| O | `axi_mst_arvalid` | 1 | AR |
| I | `axi_mst_arready` | 1 | AR |
| O | `axi_mst_arid` | 4 | AR |
| O | `axi_mst_araddr` | 16 | AR |
| O | `axi_mst_arlen` | 8 | AR |
| O | `axi_mst_arsize` | 3 | AR |
| O | `axi_mst_arburst` | 2 | AR |
| O | `axi_mst_aruser` | 5 | AR |
| I | `axi_mst_rvalid` | 1 | R |
| O | `axi_mst_rready` | 1 | R |
| I | `axi_mst_rid` | 4 | R |
| I | `axi_mst_rdata` | 32 | R |
| I | `axi_mst_rresp` | 2 | R |
| I | `axi_mst_rlast` | 1 | R |
| I | `axi_mst_ruser` | 5 | R |

### B.3 EASYAXI_SLV_WR_CTRL

| 方向 | 信号 | 位宽 | 通道 |
|------|------|------|------|
| I | `axi_slv_awvalid` | 1 | AW |
| O | `axi_slv_awready` | 1 | AW |
| I | `axi_slv_awid` | 4 | AW |
| I | `axi_slv_awaddr` | 16 | AW |
| I | `axi_slv_awlen` | 8 | AW |
| I | `axi_slv_awsize` | 3 | AW |
| I | `axi_slv_awburst` | 2 | AW |
| I | `axi_slv_awuser` | 5 | AW |
| I | `axi_slv_wvalid` | 1 | W |
| O | `axi_slv_wready` | 1 | W |
| I | `axi_slv_wdata` | 32 | W |
| I | `axi_slv_wstrb` | 4 | W |
| I | `axi_slv_wlast` | 1 | W |
| I | `axi_slv_wuser` | 5 | W |
| O | `axi_slv_bvalid` | 1 | B |
| I | `axi_slv_bready` | 1 | B |
| O | `axi_slv_bid` | 4 | B |
| O | `axi_slv_bresp` | 2 | B |
| O | `axi_slv_buser` | 5 | B |
| O | `s_wr_data_en` | 1 | internal |
| O | `s_wr_data_id` | 4 | internal |
| O | `s_wr_data_addr` | 16 | internal |
| O | `s_wr_data_data` | 128 | internal |
| O | `s_wr_data_strb` | 16 | internal |

### B.4 EASYAXI_SLV_RD_CTRL

| 方向 | 信号 | 位宽 | 通道 |
|------|------|------|------|
| I | `axi_slv_arvalid` | 1 | AR |
| O | `axi_slv_arready` | 1 | AR |
| I | `axi_slv_arid` | 4 | AR |
| I | `axi_slv_araddr` | 16 | AR |
| I | `axi_slv_arlen` | 8 | AR |
| I | `axi_slv_arsize` | 3 | AR |
| I | `axi_slv_arburst` | 2 | AR |
| I | `axi_slv_aruser` | 5 | AR |
| O | `axi_slv_rvalid` | 1 | R |
| I | `axi_slv_rready` | 1 | R |
| O | `axi_slv_rid` | 4 | R |
| O | `axi_slv_rdata` | 32 | R |
| O | `axi_slv_rresp` | 2 | R |
| O | `axi_slv_rlast` | 1 | R |
| O | `axi_slv_ruser` | 5 | R |
| O | `s_rd_req_en` | 1 | internal |
| O | `s_rd_req_id` | 4 | internal |
| O | `s_rd_req_addr` | 16 | internal |
| I | `s_rd_resp_en` | 1 | internal |
| I | `s_rd_resp_id` | 4 | internal |
| I | `s_rd_resp_data` | 128 | internal |
| I | `s_rd_resp_resp` | 2 | internal |

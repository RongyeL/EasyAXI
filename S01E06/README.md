# S01E06: AXI 乱序&交织实现 
1. 读事务为例，说明AXI Out-of-order、Interleave特性的实现；
2. 设计轮询调度器，支撑实现乱序和交织；
3. 引入同ID保序问题。

---

# S01E06: AXI Out-of-Order & Interleave Implementation
1. Using read transactions as an example, illustrate the implementation of AXI out-of-order and interleave features;
2. Design a polling scheduler to support out-of-order and interleave implementations;
3. Introduce the issue of order preservation for identical IDs.

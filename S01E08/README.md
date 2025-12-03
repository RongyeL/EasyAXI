# S01E07: AXI 同ID保序实现 
1. 读事务为例，说明AXI ID同保序特性实现；
2. SLV侧，同ID请求按收请求顺序返回响应；
3. MST侧，同ID请求按发请求顺序接收响应；
 
---

# S01E07: AXI Same-ID ORDER Implementation
1. Using a read transaction as an example, this section describes the implementation of the AXI Same-ID Sequence Preservation feature.
2. On the SLV side, responses to same-ID requests are returned in the order in which they are received.
3. On the MST side, responses to same-ID requests are received in the order in which they are sent.

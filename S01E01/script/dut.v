// 端口信号声明
wire  clk            ;
wire  rst_n          ;
wire  enable         ;
wire  axi_mst_arvalid;
wire  axi_mst_arready;
wire  wire           ;

// 模块实例化
EASYAXI_MST U_EASYAXI_MST (
    .clk             (clk),  // i
    .rst_n           (rst_n),  // i
    .enable          (enable),  // i
    .axi_mst_arvalid (axi_mst_arvalid),  // o
    .axi_mst_arready (axi_mst_arready),  // i
    .wire            (wire),  // o
);

#!/bin/bash

# 获取脚本的当前路径并进入该目录
cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null 2>&1

# 导出为环境变量
export ROOT_PATH="$(dirname "$(pwd)")"

# 打印路径，供验证
echo "set Project ROOT_PATH: $ROOT_PATH"

# 返回原始路径，静默模式
cd - > /dev/null 2>&1


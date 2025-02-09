import re
import os
import sys

def remove_comments(content):
    """
    移除 Verilog 文件中的注释。
    """
    # 移除多行注释 /* ... */
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    # 移除单行注释 //
    content = re.sub(r'//.*', '', content)
    return content

def parse_verilog_module(content):
    """
    解析 Verilog 文件内容，提取模块名称、参数和端口信息。
    """
    # 移除注释
    content = remove_comments(content)

    # 提取模块名称
    module_match = re.search(r'module\s+(\w+)\s*#?\(?', content)
    if not module_match:
        raise ValueError("未找到模块定义！")
    module_name = module_match.group(1)

    # 提取参数部分（如果有）
    parameter_section = re.search(r'#\s*\(([\s\S]*?)\)', content)
    parameters = []
    if parameter_section:
        parameter_content = parameter_section.group(1)
        parameter_matches = re.findall(r'parameter\s+(\w+)\s*=\s*([^,;\s]+)', parameter_content)
        for name, value in parameter_matches:
            parameters.append({"name": name, "value": value})

    # 提取端口部分
    port_section = re.search(r'\(\s*([\s\S]*?)\s*\)\s*;', content)
    if not port_section:
        raise ValueError("未找到端口定义！")
    port_content = port_section.group(1)

    # 提取端口列表（支持 input wire/output reg 等格式）
    port_matches = re.findall(r'(input|output|inout)\s+(?:wire|reg)?\s*(?:\[([\w\-\+\/\*:]+)\])?\s*(\w+)', port_content)
    ports = []
    for direction, width, port_name in port_matches:
        ports.append({
            "name": port_name,
            "direction": direction,
            "width": f"[{width}]" if width else ""
        })

    return module_name, parameters, ports

def generate_signal_declarations(ports):
    """
    生成端口信号的声明，使用 wire 类型，信号名对齐。
    """
    # 计算最大信号名称长度和最大位宽长度
    max_port_name_length = max(len(port['name']) for port in ports)
    max_width_length = max(len(port['width']) for port in ports)

    declarations = []
    for port in ports:
        # 对齐信号名和位宽
        port_name_padding = " " * (max_port_name_length - len(port['name']))
        width_padding = " " * (max_width_length - len(port['width']))
        declaration = f"wire {port['width']}{width_padding} {port['name']}{port_name_padding};"
        declarations.append(declaration)
    return "\n".join(declarations)

def generate_verilog_instance(module_name, instance_name, parameters, ports):
    """
    生成 Verilog 实例化代码，实例名称使用 U_ 前缀，括号缩进对齐，并添加端口方向注释。
    """
    instance_code = ""
    if parameters:
        instance_code += f"{module_name} #(\n"
        for param in parameters:
            instance_code += f"    .{param['name']}({param['value']}),\n"
        instance_code = instance_code.rstrip(",\n") + "\n) "
    else:
        instance_code += f"{module_name} "
    
    # 实例名称使用 U_ 前缀
    instance_code += f"U_{module_name} (\n"

    # 计算最大端口名称长度，用于对齐
    max_port_name_length = max(len(port['name']) for port in ports)

    # 生成端口连接，左括号对齐，右括号对齐，并添加注释
    for port in ports:
        port_name = port['name']
        direction = "i" if port['direction'] == "input" else "o"  # 输入端口标 i，输出端口标 o
        padding = " " * (max_port_name_length - len(port_name))  # 对齐填充
        instance_code += f"    .{port_name}{padding} ({port['name']}),  // {direction}\n"

    # 右括号对齐
    instance_code += ");"
    return instance_code

def main(input_file_path, output_file_path):
    """
    主函数：读取 Verilog 文件，生成实例化代码并保存。
    """
    # 读取 Verilog 文件内容
    with open(input_file_path, 'r') as file:
        verilog_content = file.read()

    # 解析 Verilog 内容
    module_name, parameters, ports = parse_verilog_module(verilog_content)

    # 生成端口信号声明
    signal_declarations = generate_signal_declarations(ports)

    # 生成实例化代码
    instance_code = generate_verilog_instance(module_name, module_name, parameters, ports)

    # 将信号声明和实例化代码写入输出文件
    with open(output_file_path, 'w') as file:
        file.write("// 端口信号声明\n")
        file.write(signal_declarations + "\n\n")
        file.write("// 模块实例化\n")
        file.write(instance_code + "\n")

    print(f"实例化代码已生成并保存到 {output_file_path} 中。")

# 示例用法
if __name__ == "__main__":
    # 检查命令行参数
    if len(sys.argv) != 2:
        print("使用方法：python generate_dut.py <verilog文件>")
        sys.exit(1)

    # 输入文件路径（命令行参数）
    input_file_path = sys.argv[1]

    # 输出文件路径（脚本所在路径下的 dut.v）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_file_path = os.path.join(script_dir, "dut.v")

    # 调用主函数
    main(input_file_path, output_file_path)

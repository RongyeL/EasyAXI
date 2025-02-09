#!/usr/bin/env python
# coding=utf-8
'''
 $ @Author       : Luma
 $ @Date         : 2021-02-05 21:40:10
 $ @LastEditTime : 2024-04-27 15:56:19
 $ @LastEditors  : Luma
 $ @Description  : The script is obtained from 
                   https://gitee.com/lumafilter/script/blob/master/Python/ins/ins.py.  

 $ @FilePath     : \working\script\Python\ins\ins.py
'''
import os
import re
import logging
import argparse
import glob

class sources:
    def __init__(self,file_name=None,module_name=None,ins_mode='inst_&_wire',ins_name=None,indentation=30,tar_file_name='DUT.sv'):
        self.file_name         = file_name
        self.module_name       = module_name
        self.ins_mode          = ins_mode
        self.indentation       = indentation
        self.ins_name          = ins_name
        self.tar_file_name     = tar_file_name
        self.head_lines        = []
        self.write_list        = []
        print("Start parsing {}...".format(self.file_name))
        self.get_module(self.module_name)

        if self.head_lines:
            self.write_list.append(['module {}();'.format('DUT')])
            for module_head_lines in self.head_lines:
                module_curr = module(module_head_lines[0],list(module_head_lines[2:]),self.ins_mode,self.ins_name,self.indentation)
                self.write_list.append(['\n//------------------------------Instantiated module: {} ------------------------------\n'.format(module_curr.ins_name)])
                self.write_list.append(module_curr.out_line_list)
            self.write_list.append(['endmodule\n'])
            self.gen_file(self.write_list)
        else:
            print(f'There is no any module in {self.file_name}')


    def get_module(self,module_name=None):
        regex_module_head_verilog_2001 = re.compile(r'''
            [^\w]*module\s*
            (%s)                                                                                                                                                                    #1 module name
            ((?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*\#(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*\((?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*parameter(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*)?            #2
            ((?(2)(?:.*\n)*?[^/\n]*(?:/[^/\n]+)*))                                                                                                                                  #3 parameter
            (?(2)\s*\)(?:\s*//.*\n)?)
            (?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*\((?:\s*/\*.*\*/\n)?(?:\s*//.*\n)?\s*
            ((?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*,?\s*(?:\(.*\))?\s*(?:output|input|inout).*\n)+?[^/\n]*(?:/[^/\n]+)*                              #4 ports:main
             (?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*)                                                                                         #4 ports:comments,`endif,.etc
            \s*\)[ \t]*?;
            (.*)                                                                                                                                                                    #5 comments
            [\s\S]*?endmodule
            ''' %(module_name if module_name is not None else '\w+'),re.VERBOSE)

        regex_module_head_verilog_1995 = re.compile(r'''
            \W*module\s*
            (%s)                                                                                                                                              #1 module name
            \s*\(
            ((?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*,?(?:\s*\w+(?<!input)(?<!output)(?<!inout)[ \t]*?,?)+(?:[ \t]*/\*.*\*/)?(?:[ \t]*//.*)?(?:\s*`.*\n)?\s*)*?)   #2 module head
            (?:\s*/\*.*\*/\n)*?(?:\s*//.*\n)*?\s*?\)[ \t]*?;(?:[ \t]*/\*.*\*/)?(?:[ \t]*//.*)?\n
            (?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*`.*\n)*?                                                                                                     #  define、include
            (?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*localparam.*\n)*                                               #  localparam
            ((?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*parameter.*\n)*
             (?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*)                                                                   #3 parameter
            (?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*`.*\n)*                                                                                                     #  define、include
            (?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*localparam.*\n)*                                               #  localparam
            ((?:(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*\s*(?:output|input|inout).*\n)+
             (?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*(?:\s*`.*\n)*(?:\s*/\*.*\*/\n)*(?:\s*//.*\n)*)                                                                   #4 ports
        #     [\s\S]*?endmodule
        #     ''' %(module_name if module_name is not None else '\w+'),re.VERBOSE)

        file_obj         = open(self.file_name,'r')
        file_str         = file_obj.read()
        file_obj.close()
        self.head_lines  =  re.findall(regex_module_head_verilog_1995,file_str) + re.findall(regex_module_head_verilog_2001,file_str)
        # self.head_lines  =  re.findall(regex_module_head_verilog_1995,file_str)
        # self.head_lines  =   re.findall(regex_module_head_verilog_2001,file_str)
        logging.info(f'self.head_lines,len={len(self.head_lines)}\t:{self.head_lines}')

    def gen_file(self,write_list=[]):
        try:
            if self.__class__.gen_file.called:
                wr_mode = 'a'
        except AttributeError:
            self.__class__.gen_file.called = True
            wr_mode = 'w'

        with open(self.tar_file_name,'{}'.format(wr_mode)) as file_obj:
            for module in write_list:
                for line in module:
                    file_obj.write(line)
                    file_obj.write('\n')
        print('{} in {} been Instantiated successfully into {}!'.format('All modules' if self.module_name is None else 'Module '+self.module_name,
                                                                      self.file_name+' have' if self.module_name is None else self.file_name+' has',
                                                                      self.tar_file_name))

class module:
    regex_module_params = re.compile(r'''
        ^[ \t]*(?!//)[ \t]*?(?:parameter[ \t]*)*
        ,?\s*(\w+)                                           #1 param name
        [ \t]*=[ \t]*
        ([`'{}()\w \t+\-*<>:]+(?:/?[`'{}()\w \t+\-*<>:]+)+)  #2 value
        [ \t]*[,;]?[ \t]*
        ((?:/\*.*\*/)?(?://.*)?)?                            #3 comment
        ''',re.VERBOSE)
    regex_module_ports = re.compile(r'''
        ^[ \t]*(?!//)[ \t]*?(?:\(.*\))?[ \t]*?,?(output|input|inout)   #1 direction
        [ \t]*
        (wire|reg)?                                                    #2 type
        [ \t]*
        ((?:\[[`'{},()\w \t+\-*/<>:?]+\][ \t]*)*)                      #3 width
        [ \t]*
        ((?:\w+[ \t]*[,;]?[ \t]*)+)                                    #4 port name
        [ \t]*
        ((?:/\*.*\*/)?(?://.*)?)?                                      #5 comment
        ''',re.VERBOSE)
    tb_initial = '''
        bit Clk  ;
        bit Clear;
        bit Rstn ;

        initial forever #2.5 Clk = ~Clk;//200MHz
        initial
        begin
            repeat (5) @posedge( Clk );
            Rstn <= 'b1
        end
        '''

    def __init__(self,module_name=None,extract_list=[],ins_mode='inst_&_wire',ins_name=None,indentation=30):
        self.ins_mode    = ins_mode
        self.ins_name    = ins_name if ins_name is not None else 'U_'+module_name
        self.indentation = indentation
        self.module_name = module_name
        self.params_list = ( list(extract_list[0].strip(' \n').split('\n')) if len(extract_list[0].strip(' \n').split('\n')) > 1 else list(extract_list[0].strip(' \n').split(',')) ) if list(extract_list[0].strip(' \n')) else []
        self.ports_list  = list(extract_list[1].strip(' \n').split('\n'))
        self.comment     = extract_list[2] if len(extract_list) > 2 else ''
        self.params_info_list = []
        self.ports_info_list  = []
        self.out_line_list    = []
        print("Start parsing module {}...".format(self.module_name))
        logging.info(f'extract_list\t:{extract_list}')
        logging.info(f'self.params_list\t:{self.params_list}')
        logging.info(f'self.ports_list\t:{self.ports_list}')
        if self.params_list:
            self.get_params()
        self.get_ports()
        self.instance()

    def get_params(self):
        for line in self.params_list:
            re_params_obj = re.search(module.regex_module_params,line)
            if re_params_obj is not None:
                params_info = {'flag'     :                     1,
                               'name'     :re_params_obj.group(1),
                               'value'    :re_params_obj.group(2),
                               'comment'  :re_params_obj.group(3) if re_params_obj.group(3) is not None else ""     }
            else:
                params_info = {'flag' : 0,'comment' : line}
            self.params_info_list.append(params_info)
            logging.debug(f're_params_obj\t:{re_params_obj}')
            logging.debug(f'params_info\t:{params_info}')
        logging.info(f'params_info_list\t:{self.params_info_list}')


    def get_ports(self):
        for line in self.ports_list:
            re_ports_obj = re.search(module.regex_module_ports,line)
            if re_ports_obj is not None:
                port_info = {'flag'     :                    1,
                             'direction':re_ports_obj.group(1),
                             'type'     :re_ports_obj.group(2) if re_ports_obj.group(2) is not None else "wire",
                             'width'    :re_ports_obj.group(3) if re_ports_obj.group(3) is not None else ""    ,
                             'name'     :re_ports_obj.group(4).rstrip(',; '),
                             'comment'  :re_ports_obj.group(5) if re_ports_obj.group(5) is not None else ""     }
                logging.debug(f'name-group(4)\t:{re_ports_obj.group(4)}')
            else:
                port_info = {'flag' : 0,'comment' : line}
            self.ports_info_list.append(port_info)
            logging.debug(f'curr_line\t:{line}')
            logging.debug(f're_ports_obj\t:{re_ports_obj}')
            logging.debug(f'port_info\t:{port_info}')
        logging.info(f'ports_info_list\t:{self.ports_info_list}')

    def instance(self):
        params_num                = len(self.params_info_list)
        params_num_vld            = len([line for line in self.params_info_list if line['flag']])
        params_max_width_of_name  = max([len(str( line['name'] ))   for line in self.params_info_list if line['flag'] ]) if params_num > 0 else 0
        params_max_width_of_value = max([len(str( line['value'] ))  for line in self.params_info_list if line['flag'] ]) if params_num > 0 else 0

        ports_num                 = len(self.ports_info_list)
        ports_num_vld             = len([line for line in self.ports_info_list if line['flag']])
        ports_max_width_of_width  = max([len(str( line['width'] )) for line in self.ports_info_list if line['flag'] ])
        ports_max_width_of_name   = max([len(str( line['name'] ))  for line in self.ports_info_list if line['flag'] ])
        ports_max_width_of_name_s = max([len(str( port_name )) for line in self.ports_info_list if line['flag'] for port_name in line['name'].split(',')])

        ins_max_width_of_name     = max(params_max_width_of_name ,ports_max_width_of_name_s,self.indentation)
        ins_max_width_of_value    = max(params_max_width_of_value,ports_max_width_of_name_s,self.indentation)

        #param & wire declariton
        if ins_mode != 'inst_only':
            # param
            for i in range(params_num):
                if self.params_info_list[i]['flag']:
                    self.out_line_list.append('localparam {:<{}} = {:<{}} ;{}'.format(self.params_info_list[i]['name'].lstrip() ,ins_max_width_of_name,
                                                                              self.params_info_list[i]['value'].lstrip(),ins_max_width_of_value,
                                                                              self.params_info_list[i]['comment']  ))
                else:
                    self.out_line_list.append('{}'.format(self.params_info_list[i]['comment'].lstrip()))
            self.out_line_list.append('\n')

            # wire
            for i in range(ports_num):
                if self.ports_info_list[i]['flag']:
                    self.out_line_list.append('{:<4} {:>{}} {:<{}};{}'.format('reg' if self.ports_info_list[i]['type'] == 'reg' else 'wire',
                                                                   self.ports_info_list[i]['width']  ,ports_max_width_of_width,
                                                                   self.ports_info_list[i]['name']   ,ports_max_width_of_name  ,
                                                                   self.ports_info_list[i]['comment']  ))
                else:
                    self.out_line_list.append('{}'.format(self.ports_info_list[i]['comment'].lstrip()))
            self.out_line_list.append('\n')
            if ins_mode == 'inst_tb':
                for line in module.tb_initial.split('\n'):
                    self.out_line_list.append(line.lstrip())

        #intance
        if params_num > 0:
            self.out_line_list.append('{} # ('.format(self.module_name))
            params_cnt_vld = 0
            for i in range(params_num):
                if self.params_info_list[i]['flag']:
                    params_cnt_vld += 1
                    self.out_line_list.append('    .{:<{}} ( {:<{}} ){}{}'.format(self.params_info_list[i]['name'].lstrip(),ins_max_width_of_name,
                                                                              self.params_info_list[i]['name'].lstrip(),ins_max_width_of_value,
                                                                              ',' if params_cnt_vld < params_num_vld else "",
                                                                              self.params_info_list[i]['comment']  ))
                    logging.debug(f"params_info_list:\t{self.params_info_list[i]['name']}\t")
                else:
                    self.out_line_list.append('{}'.format(self.params_info_list[i]['comment'].lstrip()))
            self.out_line_list.append(')\n{}\n('.format(self.ins_name))
        else:
            self.out_line_list.append('{} {} ('.format(self.module_name,self.ins_name))

        ports_cnt_vld = 0
        for i in range(ports_num):
            if self.ports_info_list[i]['flag']:
                ports_cnt_vld += 1
                for j,port_name in enumerate(self.ports_info_list[i]['name'].split(',')):
                    self.out_line_list.append('    .{:<{}} ( {:<{}} ){}{}'.format(port_name.lstrip(),ins_max_width_of_name,
                                                                              port_name.lstrip(),ins_max_width_of_value,
                                                                              ',' if not( ports_cnt_vld == ports_num_vld and j == len(self.ports_info_list[i]['name'].split(','))-1 ) else "",
                                                                              self.ports_info_list[i]['comment']  ))
                    logging.debug(f"ports_info_list:\t{self.ports_info_list[i]['name']}:j\t:{j}")
            else:
                self.out_line_list.append('{}'.format(self.ports_info_list[i]['comment'].lstrip()))

        self.out_line_list.append(');{}\n'.format(self.comment))

def arg_parser():
    parser = argparse.ArgumentParser(allow_abbrev = True)
    parser.add_argument("-f","--filename",metavar="InputFile",dest='filename',nargs='+')
    parser.add_argument("-module",metavar="ModuleName",help='specify one module in the file')
    parser.add_argument("-insmode",choices=['0','1','2'],help='0:{};1:{};2:{}'.format(ins_mode_list[0],ins_mode_list[1],ins_mode_list[2]),default=0)
    parser.add_argument("-insname",metavar="InstantiatedModuleName",help='specify the module name after instance',type=str)
    parser.add_argument("-indent",metavar="IndentWidth",help='specify the indentation of instance',type=int,default=30)
    parser.add_argument("-fout",metavar="OutputFile",help='specify the output file',default='ins.v')
    parser.add_argument('-v','--verbose',choices=['DEBUG','INFO','WARNING','ERROR','CRITICAL'],dest='verbose',help='set logger_level',default='ERROR')
    parser.add_argument('-log',help='output log to log file',action='store_true')

    # parser.print_help()
    options       = vars(parser.parse_args())
    file_name_lst = options["filename"]
    module_name   = options["module"]
    ins_mode      = ins_mode_list[int(options["insmode"])]
    ins_name      = options["insname"]
    indentation   = options["indent"]
    tar_file_name = options["fout"]
    logger_level  = options["verbose"]
    logger_log    = options["log"]
    print('input file    : {}\nmodule name   : {}\ninstance mode : {}\noutput file   : {}'.format(file_name_lst,module_name,ins_mode,tar_file_name))
    return file_name_lst,module_name,ins_mode,ins_name,indentation,tar_file_name,logger_level,logger_log

# --Main-----------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    ins_mode_list  = ['inst_&_wire','inst_only','inst_tb']
    file_name_lst,module_name,ins_mode,ins_name,indentation,tar_file_name,logger_level,logger_log = arg_parser()

    logger = logging.getLogger(__name__)
    if logger_log:
        logging.basicConfig(format = '%(levelname)s - %(message)s\n/-------------------------------------------------------------------------------------------',level=logger_level,filename='ins.log',filemode='w')
    else:
        logging.basicConfig(format = '%(levelname)s - %(message)s\n/-------------------------------------------------------------------------------------------',level=logger_level)
    # logging.basicConfig(format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s\t\t- %(funcName)s',level=logger_level)
    # logging.debug('debug，用来打印一些调试信息，级别最低,所以加调试语句用这个，然后发布时将level改成ERROR')
    # logging.info('info，用来打印一些正常的操作信息')
    # logging.warning('waring，用来用来打印警告信息')
    # logging.error('error，一般用来打印一些错误信息')
    # logging.critical('critical，用来打印一些致命的错误信息，等级最高')
    # logging.debug(f'logger_level:{logger_level}')


    file_list  = file_name_lst if file_name_lst else [f for f in glob.glob('*.v')+glob.glob('*.sv') if not re.match(r'DUT.sv', f)]
    for file in file_list:
        sources(file,module_name,ins_mode,ins_name,indentation,tar_file_name)


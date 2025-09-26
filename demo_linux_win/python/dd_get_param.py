import configparser
import sys
from io import StringIO


def parse_robot_config(config_content):
    """
    解析机器人配置文件内容
    """
    # 创建一个ConfigParser对象
    config = configparser.ConfigParser()

    # 保持键的大小写（默认会转换为小写）
    config.optionxform = lambda option: option

    # 读取配置内容
    config.read_string(config_content)

    # 存储解析后的数据
    parsed_data = {}

    # 遍历所有节
    for section in config.sections():
        parsed_data[section] = {}

        # 获取该节的所有选项
        for option in config.options(section):
            value = config.get(section, option)

            # 尝试将数值转换为适当的数据类型
            try:
                # 检查是否是浮点数
                if '.' in value:
                    parsed_value = float(value)
                else:
                    parsed_value = int(value)
            except ValueError:
                # 如果不是数值，保持字符串格式
                parsed_value = value

            parsed_data[section][option] = parsed_value

    return parsed_data


def print_config_summary(parsed_data):
    """
    打印配置摘要信息
    """
    print("机器人配置摘要:")
    print("=" * 50)

    # 查找所有不同的机器人部分
    robot_sections = {}
    for section in parsed_data.keys():
        if section.startswith('R.A'):
            # 提取机器人标识符 (如 A0, A1)
            robot_id = section.split('.')[1]
            section_type = section.split('.')[2]

            if robot_id not in robot_sections:
                robot_sections[robot_id] = []

            robot_sections[robot_id].append(section_type)

    # 打印每个机器人的基本信息
    a=''
    b=''
    for robot_id, sections in robot_sections.items():
        print(f"\n机器人 {robot_id}:")

        if f"R.{robot_id}.BASIC" in parsed_data:
            basic = parsed_data[f"R.{robot_id}.BASIC"]
            print(f"  类型: {basic.get('Type', '未知')}")
            print(f"  自由度: {basic.get('Dof', '未知')}")
            print(f"  重力: X={basic.get('GravityX', 0)}, Y={basic.get('GravityY', 0)}, Z={basic.get('GravityZ', 0)}")
            if robot_id=="A0":
                a=f"{basic.get('Type', '未知')}"+","+f"{basic.get('GravityX', 0)}"+","+f"{basic.get('GravityY', 0)}"+","+f"{basic.get('GravityZ', 0)},"+"\n"
            else:
                b = f"{basic.get('Type', '未知')}" + "," + f"{basic.get('GravityX', 0)}" + "," + f"{basic.get('GravityY', 0)}" + "," + f"{basic.get('GravityZ', 0)},"+"\n"

        # # 统计关节数量
        # joint_count = sum(1 for s in sections if s.startswith('L') and len(s) == 2)
        # print(f"  关节数量: {joint_count}")


        dh=''
        dh0=''
        for i in range(7):
            dh = parsed_data[f"R.{robot_id}.L{i}.DH"]
            print(f"{i} axis dh: {dh.get('A', 0)},{dh.get('Alpha', 0)},{dh.get('D', 0)},{dh.get('Theta', 0)},")
            kine=parsed_data[f"R.{robot_id}.L{i}.BASIC"]
            print(f"{i} axis vel acc low up: {kine.get('AccMax', 0)}, {kine.get('VelMax', 0)},{kine.get('LimitNeg', 0)},{kine.get('LimitPos',0)},")
            dyn = parsed_data[f"R.{robot_id}.L{i}.DYNAMIC"]
            print(
                f"{i} axis dyn: {dyn.get('InertiaXX', 0)},{dyn.get('InertiaXY', 0)},{dyn.get('InertiaXZ', 0)},"
                f"{dyn.get('InertiaYY', 0)}, {dyn.get('InertiaYZ', 0)},{dyn.get('InertiaZZ', 0)},"
                f"{dyn.get('M', 0)},{dyn.get('MRX', 0)},{dyn.get('MRY', 0)},{dyn.get('MRZ', 0)},")

            if robot_id=='A0':
                a+=f"{dh.get('A', 0)},{dh.get('Alpha', 0)},{dh.get('D', 0)},{dh.get('Theta', 0)},"+f"{kine.get('AccMax', 0)},{kine.get('VelMax', 0)},{kine.get('LimitNeg', 0)},{kine.get('LimitPos', 0)},"+f"{dyn.get('InertiaXX', 0)},{dyn.get('InertiaXY', 0)},{dyn.get('InertiaXZ', 0)},{dyn.get('InertiaYY', 0)}, {dyn.get('InertiaYZ', 0)},{dyn.get('InertiaZZ', 0)},{dyn.get('M', 0)},{dyn.get('MRX', 0)},{dyn.get('MRY', 0)},{dyn.get('MRZ', 0)}"+",\n"
            else:
                b += f"{dh.get('A', 0)},{dh.get('Alpha', 0)},{dh.get('D', 0)},{dh.get('Theta', 0)}," + f"{kine.get('AccMax', 0)},{kine.get('VelMax', 0)},{kine.get('LimitNeg', 0)},{kine.get('LimitPos', 0)}," + f"{dyn.get('InertiaXX', 0)},{dyn.get('InertiaXY', 0)},{dyn.get('InertiaXZ', 0)},{dyn.get('InertiaYY', 0)}, {dyn.get('InertiaYZ', 0)},{dyn.get('InertiaZZ', 0)},{dyn.get('M', 0)},{dyn.get('MRX', 0)},{dyn.get('MRY', 0)},{dyn.get('MRZ', 0)}" + ",\n"

        dh0 = parsed_data[f"R.{robot_id}.FLANGE"]
        print(f"{i} axis dh: {dh0.get('A', 0)}, {dh0.get('Alpha', 0)},{dh0.get('D', 0)},{dh0.get('Theta', 0)},")
        if robot_id=='A0':
            a +=f"{dh0.get('A', 0)},{dh0.get('Alpha', 0)},{dh0.get('D', 0)},{dh0.get('Theta', 0)},\n"
        else:
            b += f"{dh0.get('A', 0)},{dh0.get('Alpha', 0)},{dh0.get('D', 0)},{dh0.get('Theta', 0)},\n"

        if f"R.{robot_id}.CTRL" in parsed_data:
            ctrl = parsed_data[f"R.{robot_id}.CTRL"]
            print(f"{ctrl.get('BD67NN0', 0)},{ctrl.get('BD67NN1', 0)},{ctrl.get('BD67NN2', 0)},\n"
                  f"{ctrl.get('BD67NP0', 0)},{ctrl.get('BD67NP1', 0)},{ctrl.get('BD67NP2', 0)},\n"
                  f"{ctrl.get('BD67PN0', 0)},{ctrl.get('BD67PN1', 0)},{ctrl.get('BD67PN2', 0)},\n"
                  f"{ctrl.get('BD67PP0', 0)},{ctrl.get('BD67PP1', 0)},{ctrl.get('BD67PP2', 0)},\n")
            if robot_id == 'A0':
                a += f"{ctrl.get('BD67NN0', 0)},{ctrl.get('BD67NN1', 0)},{ctrl.get('BD67NN2', 0)},\n{ctrl.get('BD67NP0', 0)},{ctrl.get('BD67NP1', 0)},{ctrl.get('BD67NP2', 0)},\n{ctrl.get('BD67PN0', 0)},{ctrl.get('BD67PN1', 0)},{ctrl.get('BD67PN2', 0)},\n{ctrl.get('BD67PP0', 0)},{ctrl.get('BD67PP1', 0)},{ctrl.get('BD67PP2', 0)},\n"
            else:
                b += f"{ctrl.get('BD67NN0', 0)},{ctrl.get('BD67NN1', 0)},{ctrl.get('BD67NN2', 0)},\n{ctrl.get('BD67NP0', 0)},{ctrl.get('BD67NP1', 0)},{ctrl.get('BD67NP2', 0)},\n{ctrl.get('BD67PN0', 0)},{ctrl.get('BD67PN1', 0)},{ctrl.get('BD67PN2', 0)},\n{ctrl.get('BD67PP0', 0)},{ctrl.get('BD67PP1', 0)},{ctrl.get('BD67PP2', 0)},\n"


    # # 打印基本配置
    # if "R.BASIC" in parsed_data:
    #     basic = parsed_data["R.BASIC"]
    #     print(f"\n系统基本配置:")
    #     print(f"  名称: {basic.get('Name', '未知')}")
    #     print(f"  总线频率: {basic.get('BusFreq', '未知')}Hz")
    #     print(f"  使用急停: {'是' if basic.get('UseEMG', 0) == 1 else '否'}")

    return a,b




if __name__ == "__main__":
    with open('robot_1003_UB.ini', 'r') as file:
        config_content = file.read()

        # 解析配置文件
    parsed_data = parse_robot_config(config_content)

    # 打印配置摘要
    a, b = print_config_summary(parsed_data)


    c = a + b
    print(c)

    # 可以选择保存解析后的数据到文件s
    import json

    with open('ccs.MvKDCfg', 'w') as f:
        f.write(c)
    f.close()
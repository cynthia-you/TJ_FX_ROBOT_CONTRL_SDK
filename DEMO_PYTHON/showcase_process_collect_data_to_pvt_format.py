import re
def process_and_downsample(file_path, format_unify=True):
    """
    完整处理：下采样 + 特征重映射 + 格式统一
    1. 第一行：将'='和'@'之间的数字改为7，@后的行数减半
    2. 下采样：每隔一行取一行（1000Hz->500Hz）
    3. 特征重映射：删除前两列，后面7列数据往前移动，重新分配字母标识
    4. 格式统一：统一使用\n换行符，数值格式化为6位小数

    参数：
    file_path: 要处理的文件路径
    format_unify: 是否统一数值格式为6位小数（默认True）
    """

    # 读取文件所有行
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    if not lines:
        print("文件为空")
        return

    processed_lines = []
    # 处理第一行
    first_line = lines[0].strip('\r\n')
    if '=' in first_line and '@' in first_line:
        # 分离@前后的部分
        parts = first_line.split('@')
        if len(parts) == 2:
            # 处理=和@之间的数字
            first_part = re.sub(r'(=)\d+(@?)', r'\g<1>7\2', parts[0])
            # 获取原始行数并减半（1000Hz->500Hz）
            try:
                original_rows = int(parts[1])
                new_rows = original_rows // 2  # 下采样行数减半
                if original_rows % 2 != 0:
                    print(f"注意：原始行数{original_rows}不是偶数，下采样后行数为{new_rows}")
            except ValueError:
                print(f"警告：无法解析行数 '{parts[1]}'，保持原样")
                new_rows = parts[1]
            processed_first_line = f"{first_part}@{new_rows}"
            processed_lines.append(processed_first_line + '\n')
            print(f"第一行已修改: {first_line} -> {processed_first_line}")
        else:
            processed_lines.append(first_line + '\n')
    else:
        processed_lines.append(first_line + '\n')

    # 新的特征字母顺序
    new_letters = ['X', 'Y', 'Z', 'A', 'B', 'C', 'U']
    # 下采样处理：从第1行开始，每隔一行取一行并处理
    original_data_lines = 0
    processed_data_lines = 0

    for i in range(1, len(lines)):
        line = lines[i]
        # 去除换行符
        line = line.rstrip('\r\n')
        if not line:
            continue
        original_data_lines += 1
        # 下采样：每隔一行取一行（保留奇数行）
        if (i - 1) % 2 == 0:
            # 格式统一：分离字母和数值
            if format_unify:
                # 按$分割各个特征
                features = [f.strip() for f in line.split('$') if f.strip()]
                if len(features) >= 9:
                    # 统一数值格式为6位小数
                    unified_features = []
                    for feature in features:
                        # 分离字母和数值
                        match = re.match(r'([A-Z])\s+(-?\d+\.?\d*)', feature)
                        if match:
                            letter = match.group(1)
                            value_str = match.group(2)
                            # 格式化为6位小数
                            try:
                                value = float(value_str)
                                formatted_value = f"{value:.6f}"
                                unified_features.append(f"{letter} {formatted_value}")
                            except ValueError:
                                unified_features.append(feature)
                        else:
                            unified_features.append(feature)

                    # 使用统一后的特征进行处理
                    if len(unified_features) >= 9:
                        features = unified_features

                    # 特征重映射处理
                    values = []
                    for feature in features:
                        # 分离字母和数值
                        parts = feature.split(' ', 1)
                        if len(parts) == 2:
                            values.append(parts[1].strip())
                        else:
                            # 查找第一个数字或负号或小数点的位置
                            match = re.search(r'[-]?\d+\.?\d*', feature)
                            if match:
                                value = match.group()
                                values.append(value)
                            else:
                                values.append('')

                    # 删除前两个值（X和Y），取后面7个值
                    if len(values) >= 9:
                        selected_values = values[2:9]  # 取索引2到8的7个值

                        # 创建新的特征行
                        new_features = []
                        for j in range(7):
                            new_features.append(f"{new_letters[j]} {selected_values[j]}")

                        processed_line = '$'.join(new_features) + '$'
                        processed_lines.append(processed_line + '\n')
                        processed_data_lines += 1
                    else:
                        processed_lines.append(line + '$\n')
                        processed_data_lines += 1
                        print(f"警告: 第{i + 1}行数值不足，保持原样")
                else:
                    processed_lines.append(line + '$\n')
                    processed_data_lines += 1
                    print(f"警告: 第{i + 1}行特征数不足9个，保持原样")
            else:
                # 不统一格式，只进行下采样和特征重映射
                # 按$分割各个特征，提取数值部分
                features = [f.strip() for f in line.split('$') if f.strip()]

                if len(features) >= 9:
                    # 提取各特征的数值部分
                    values = []
                    for feature in features:
                        # 分离字母和数值
                        parts = feature.split(' ', 1)
                        if len(parts) == 2:
                            values.append(parts[1].strip())
                        else:
                            # 查找第一个数字或负号或小数点的位置
                            match = re.search(r'[-]?\d+\.?\d*', feature)
                            if match:
                                value = match.group()
                                values.append(value)
                            else:
                                values.append('')

                    # 删除前两个值（X和Y），取后面7个值
                    if len(values) >= 9:
                        selected_values = values[2:9]  # 取索引2到8的7个值

                        # 创建新的特征行
                        new_features = []
                        for j in range(7):
                            new_features.append(f"{new_letters[j]} {selected_values[j]}")

                        processed_line = '$'.join(new_features) + '$'
                        processed_lines.append(processed_line + '\n')
                        processed_data_lines += 1
                    else:
                        processed_lines.append(line + '$\n')
                        processed_data_lines += 1
                        print(f"警告: 第{i + 1}行数值不足，保持原样")
                else:
                    processed_lines.append(line + '$\n')
                    processed_data_lines += 1
                    print(f"警告: 第{i + 1}行特征数不足9个，保持原样")

    print(f"下采样：从{original_data_lines}行减少到{processed_data_lines}行")

    # 另存为，不覆盖
    if '/'in file_path:
        file_path=file_path.split('/')[-1]
    file_path_save='processed_'+file_path
    with open(file_path_save, 'w', encoding='utf-8', newline='\n') as f:
        f.writelines(processed_lines)

    print(f"\n文件 '{file_path}' 处理完成并已保存为 {file_path_save}")
    print(f"频率：1000Hz -> 500Hz")
    print(f"行数：{original_data_lines} -> {processed_data_lines}")
    print(f"格式统一：{'是' if format_unify else '否'}")
    print("新的特征对应关系:")
    print("原始: X Y Z A B C U V W")
    print("新的: X Y Z A B C U")
    print("对应: - - Z→X A→Y B→Z C→A U→B V→C W→U")

if __name__ == "__main__":
    # 指定文件路径
    file_path = "left_drag_y_data.txt"  # 替换为你的文件路径,保存文件也在这个路径下，以“process_”开头
    process_and_downsample(file_path, format_unify=True)
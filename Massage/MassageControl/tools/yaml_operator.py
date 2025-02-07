import yaml

def read_yaml(file_path):
    """
    读取 YAML 文件并返回 Python 对象。

    参数:
    file_path (str): YAML 文件路径。

    返回:
    data (dict): YAML 文件内容转换的 Python 对象。
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
    return data

def write_yaml(data, file_path):
    """
    将 Python 对象写入 YAML 文件。

    参数:
    data (dict): 要写入 YAML 文件的 Python 对象。
    file_path (str): 目标 YAML 文件路径。
    """
    with open(file_path, 'w', encoding='utf-8') as file:
        yaml.safe_dump(data, file, default_flow_style=False, allow_unicode=True,sort_keys=False)

def update_yaml(file_path, dict)->None:
    """
    更新 YAML 文件中的指定键值对。

    参数:
    file_path (str): YAML 文件路径。
    key (str): 要更新的键。
    value: 要更新的值。
    """
    data = read_yaml(file_path)
    if data is not None:
        data.update(dict)
    else:
        data = dict
    write_yaml(data, file_path)

def delete_key_yaml(file_path, key):
    """
    删除 YAML 文件中的指定键。

    参数:
    file_path (str): YAML 文件路径。
    key (str): 要删除的键。
    """
    data = read_yaml(file_path)
    if key in data:
        del data[key]
    write_yaml(data, file_path)

if __name__ == '__main__':

    # 示例使用
    yaml_file = 'example.yaml'
    data_to_write = {
        'name': 'Sun Yat-sen University',
        'location': 'Guangzhou',
        'founded': 1924
    }

    write_yaml(data_to_write, yaml_file)  # 写入 YAML 文件

    read_data = read_yaml(yaml_file)  # 读取 YAML 文件
    print(read_data)

    update_yaml(yaml_file, {'founded': 1910})  # 更新键值对
    read_data = read_yaml(yaml_file)
    print(read_data)
    delete_key_yaml(yaml_file, 'location')  # 删除键
    read_data = read_yaml(yaml_file)
    print(read_data)

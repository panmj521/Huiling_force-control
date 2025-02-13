import json
import logging
import os
from colorama import Fore, Style, init
from datetime import datetime
import numpy as np
import sys

# 初始化 colorama
init(autoreset=True)

# 定义日志记录器
class CustomLogger:
    def __init__(self, log_name=None, propagate=False):
        # 配置日志记录器
        self.logger = logging.getLogger(f"custom_logger_{log_name}")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = propagate
        self.log_name = log_name

    def __enter__(self):
        # 上下文管理器进入方法
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # 上下文管理器退出方法
        pass

    def log_info(self, message, is_print=True):
        self._log(message, logging.INFO, Fore.GREEN, is_print)

    def log_yellow(self, message, is_print=True):
        self._log(message, logging.INFO, Fore.YELLOW, is_print)
    
    def log_blue(self, message, is_print=True):
        self._log(message, logging.INFO, Fore.CYAN, is_print)

    def log_warning(self, message, is_print=True):
        self._log(message, logging.WARNING, Fore.YELLOW, is_print)

    def log_error(self, message, is_print=True):
        self._log(message, logging.ERROR, Fore.RED, is_print)

    def _log(self, message, level, color, is_print=True):
        # 获取当前时间并格式化，包含毫秒级精度
        current_time = datetime.now()
        formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        # 序列化消息
        if isinstance(message, (int, float, list, dict, tuple)):
            try:
                message = json.dumps(message, ensure_ascii=False)
            except (TypeError, ValueError):
                message = str(message)
        elif isinstance(message, np.ndarray):
            message = message.tolist()
            message = json.dumps(message, ensure_ascii=False)
        else:
            message = str(message)

        # 确保日志目录存在
        log_dir = 'Huiling/log'
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 记录彩色日志，使用 colorama
        if is_print:
            print(f"{color}{formatted_time} - {message}{Style.RESET_ALL}")


if __name__ == "__main__":
    # 配置 logging 模块
    log_file = 'log/test.log'
    logging.basicConfig(
        level=logging.INFO,
        format='%(message)s',  # 仅保留日志消息
        handlers=[
            logging.FileHandler(log_file, mode='w'),  # 覆盖模式，每次运行清空日志
            logging.StreamHandler(sys.stdout)  # 输出到终端
        ]
    )

    # 重定向标准输出和标准错误
    sys.stdout = LoggerWriter(logging.getLogger(), logging.INFO)
    sys.stderr = LoggerWriter(logging.getLogger(), logging.ERROR)

    # 使用示例
    with CustomLogger() as logger:
        logger.log_info("这是一个绿色的消息")
        logger.log_warning("这是一个黄色的警告")
        logger.log_error("这是一个红色的错误")

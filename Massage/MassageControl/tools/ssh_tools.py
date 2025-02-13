
import paramiko

def execute_command_on_remote(host, username, password, command, port=22):
    # 创建SSH客户端
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        # 连接到远程服务器，指定端口
        ssh.connect(hostname=host, port=port, username=username, password=password)

        # 执行命令
        stdin, stdout, stderr = ssh.exec_command(command)

        # 获取命令输出和错误信息
        output = stdout.read().decode('utf-8')
        error = stderr.read().decode('utf-8')

        return output, error
    finally:
        # 关闭SSH连接
        ssh.close()
file_path = '/sdcard/data_log.txt'

# 写文件，使用with结构自动管理文件关闭
with open(file_path, 'w') as f:
    f.write('hello hiwonder')

# 读文件，使用with结构，避免忘记关闭
with open(file_path, 'r') as f:
    content = f.read()
    print(content)

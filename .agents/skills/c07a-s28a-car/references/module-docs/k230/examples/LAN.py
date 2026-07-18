import network

def main():
    # 获取 lan 接口
    a=network.LAN()
    # 获取网口是否在使用
    print(a.active())
    # 查看网口 ip，掩码，网关， dns 配置
    print(a.ifconfig())
    # 设置网口 ip，掩码，网关， dns 配置
    print(a.ifconfig(('192.168.0.4', '255.255.255.0', '192.168.0.1', '8.8.8.8')))
    # 查看网口 ip，掩码，网关， dns 配置
    print(a.ifconfig())
    # 设置网口为 dhcp 模式
    print(a.ifconfig("dhcp"))
    # 查看网口 ip，掩码，网关， dns 配置
    print(a.ifconfig())
    # 查看网口 mac 地址
    print(a.config("mac"))
    # 设置网口为 dhcp 模式
    print(a.ifconfig("dhcp"))
    # 查看网口 ip，掩码，网关， dns 配置
    print(a.ifconfig())

main()

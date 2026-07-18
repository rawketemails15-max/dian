from machine import FPIOA  # 导入 FPIOA 模块，用于配置引脚的功能

# 实例化 FPIOA 对象
fpioa = FPIOA()

# 打印所有引脚的当前功能配置
fpioa.help()

# 打印引脚0的详细配置
fpioa.help(0)

# 打印所有支持 I2C 数据引脚（IIC0_SDA）的配置
fpioa.help(FPIOA.IIC0_SDA, func=True)

# 将引脚0设置为普通的 GPIO0 功能
fpioa.set_function(0, FPIOA.GPIO0)

# 将引脚2设置为 GPIO2，并设置一些额外的参数：
# ie=1: 引脚使能（输入使能）
# oe=1: 引脚使能（输出使能）
# pu=0: 禁用上拉电阻
# pd=0: 禁用下拉电阻
# st=1: 设置该引脚为高电平
# ds=7: 驱动能力设置为最大值
fpioa.set_function(2, FPIOA.GPIO2, ie=1, oe=1, pu=0, pd=0, st=1, ds=7)

# 获取当前配置为 UART0_TXD 功能的引脚号
fpioa.get_pin_num(FPIOA.UART0_TXD)

# 获取引脚0的当前功能配置
fpioa.get_pin_func(0)

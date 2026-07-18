from machine import SPI
from machine import FPIOA

# 实例化FPIOA对象
a = FPIOA()

# 配置GPIO14~17为QSPI功能
a.help(14)
a.set_function(14, a.QSPI0_CS0)
a.help(14)

a.help(15)
a.set_function(15, a.QSPI0_CLK)
a.help(15)

a.help(16)
a.set_function(16, a.QSPI0_D0)
a.help(16)

a.help(17)
a.set_function(17, a.QSPI0_D1)
a.help(17)

# 实例化SPI(1)，参数5MHz，极性0，相位0，8bit数据宽度
spi = SPI(1, baudrate=5000000, polarity=0, phase=0, bits=8)

# 使能GD25LQ128复位序列
spi.write(bytes([0x66]))
spi.write(bytes([0x99]))

# 读ID命令 0x9F + 3个占位字节，一共4字节
write_buf = bytes([0x9F, 0x00, 0x00, 0x00])
read_buf = bytearray(4)

spi.write_readinto(write_buf, read_buf)
print("ID:", read_buf[1:])  # ID通常在后面3字节中

# 读ID命令0x90 + 3个占位字节，这条命令需要读2字节ID数据
write_buf = bytes([0x90, 0x00, 0x00, 0x00])
read_buf = bytearray(4)
spi.write_readinto(write_buf, read_buf)
print("ID:", read_buf[2:4])
#Pin号	对应功能（配置后）
#14	QSPI0_CS0（SPI片选信号）
#15	QSPI0_CLK（SPI时钟信号）
#16	QSPI0_D0（SPI数据线D0，即MOSI或者数据线）
#17	QSPI0_D1（SPI数据线D1，即MISO或者数据线）

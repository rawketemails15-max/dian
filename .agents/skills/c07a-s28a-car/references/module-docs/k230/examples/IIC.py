from machine import I2C
i2c = I2C(2, scl=11, sda=12, freq=40000)  # 默认为 100kHz
print(i2c.scan())  # 返回第一个响应的设备地址，如 [59]
print("对应十六进制地址：", [hex(dev) for dev in i2c.scan()])

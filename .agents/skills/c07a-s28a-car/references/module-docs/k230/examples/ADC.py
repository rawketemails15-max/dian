from machine import ADC

# 实例化 ADC 通道 0
adc = ADC(0)

# 获取 ADC 通道 0 的采样值
print(adc.read_u16())

# 获取 ADC 通道 0 的电压值
print(adc.read_uv(), "uV")

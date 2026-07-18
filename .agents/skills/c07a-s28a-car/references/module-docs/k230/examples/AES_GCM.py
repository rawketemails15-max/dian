def pad(data):
    n = 16 - (len(data) % 16)
    return data + bytes([0]*n)

def unpad(data):
    # 简单去除末尾的0，不适合所有情景，示例用
    return data.rstrip(b'\x00')

import ucryptolib
import collections

Aes = collections.namedtuple('Aes', ['key', 'iv', 'pt'])

aes = [
    Aes(
        b'\xb5\x2c\x50\x5a\x37\xd7\x8e\xda\x5d\xd3\x4f\x20\xc2\x25\x40\xea',
        b'\x51\x6c\x33\x92\x9d\xf5\xa3\x28\x4f\xf4\x63\xd7\x00\x00\x00\x00',
        b'Hello World!!!123'  # 15字节，举例改成16字节以下示范
    )
]

# 原明文
pt = aes[0].pt
# 补齐，确保16的倍数
pt_padded = pad(pt)

print('明文长度:', len(pt), '补齐后长度:', len(pt_padded))

crypto = ucryptolib.aes(aes[0].key, 1, aes[0].iv)

ct = crypto.encrypt(pt_padded)
print('密文:', ct)

crypto_dec = ucryptolib.aes(aes[0].key, 1, aes[0].iv)
pt_dec_padded = crypto_dec.decrypt(ct)

pt_dec = unpad(pt_dec_padded)

print('解密后明文:', pt_dec)
print('是否匹配:', pt_dec == pt)

from numpy.random import Generator
from randomgen import Xoroshiro128

x = Xoroshiro128([0x0123456789abcdef, 0xfedcba9876543210], plusplus=True)
x.state['s'][0] = 0x0123456789abcdef
x.state['s'][1] = 0xfedcba9876543210
print(f"{hex(x.state['s'][0])} {hex(x.state['s'][1])}")
print(hex(x.random_raw(output=True)))




import random

with open('/home/ricardororiz/Documents/RicardoRoriz/temp/units/sim/mem_cart.txt', 'w') as f:
    for i in range(1000000):
        x = random.randint(0, 2000)
        y = random.randint(0, 2000)
        z = random.randint(0, 2000)
        reg = (x) | (y << 16) | (z << 32)
        f.write(f"{reg}\n")
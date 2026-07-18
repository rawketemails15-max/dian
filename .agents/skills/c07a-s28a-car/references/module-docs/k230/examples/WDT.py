from machine import WDT
import time


wdt = WDT(1,3)

for i in range(3):

    time.sleep(1)
    print(i)

    wdt.feed()
while True:

    time.sleep(0.01)

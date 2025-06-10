from plutocontrol import pluto as Pluto
import time

pluto=Pluto()

pluto.connect()
time.sleep(3)
# pluto.arm()
time.sleep(1)
# pluto.take_off()

# time.sleep(3)

for i in range(3):
    pluto.motor_speed(i, 500)
    time.sleep(1)

# pluto.land()
time.sleep(1)
# pluto.disarm()
time.sleep(3)
pluto.disconnect()
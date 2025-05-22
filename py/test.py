from plutocontrol import pluto
import time

# Create an instance of the Pluto class
Pluto = pluto()

# Connect to the drone
Pluto.connect()
time.sleep(5)
# Arm the drone
Pluto.arm()
time.sleep(5)
# Disarm the drone
Pluto.disarm()

# Disconnect from the drone
Pluto.disconnect()
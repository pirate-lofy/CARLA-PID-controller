from environment import CarlaEnv
from controller import Controller
import logging

try:
    env=CarlaEnv(0)
    
    target_speed=0.5
    control=Controller()
    #control=VehiclePIDController(env.vehicle)
    steer=speed=0
    while 1:
        c_speed,ct,nt=env.step([steer,target_speed])
        speed,steer=control.run(target_speed,c_speed,nt,ct)
        
        print(speed,steer)


except Exception:
    logging.exception('e')

finally:
    env.close()

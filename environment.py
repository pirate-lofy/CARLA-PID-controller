import sys
import numpy as np
import random
import time
from colorama import Fore

#linux
try:
    sys.path.append("carla-0.9.5-py3.5-linux-x86_64.egg")
except IndexError:
    print(Fore.YELLOW+'CarlaEnv log: cant append carla #egg'+Fore.WHITE)


import carla


class CarlaEnv():
    
    def __init__(self,env_id,host='127.0.0.1', port=2000,timeout=10):
        super(CarlaEnv,self).__init__()
        self.actors=[]   

        self.env_id=env_id
        self.host=host
        self.port=port
        self.timeout=timeout
        
        self.blueprint=self._connect()
        self.vehicle=self._add_vehicle(self.blueprint)
        self._initialize_position()
    
    def _connect(self):
        self.client = carla.Client(self.host,self.port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()
        self.map=self.world.get_map()
        print(Fore.YELLOW+'CarlaEnv log: env no. {0} client connected'.format(self.env_id)+Fore.WHITE)
        return self.world.get_blueprint_library()
    
    def _add_vehicle(self,blueprint):
        car = blueprint.filter('tesla')[0]
        self.spawn_points=self.map.get_spawn_points()
        transform = random.choice(self.spawn_points)
        vehicle = self.world.spawn_actor(car, transform)        
        self.actors.append(vehicle)
        return vehicle


    def _initialize_position(self):
        # for being sure
        self.vehicle.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0))
        transform = carla.Transform(carla.Location(-75,-20.,5),carla.Rotation(10,-100,0))
        
        self.vehicle.set_transform(transform)
        time.sleep(2)
        

    def step(self,actions,dead=False):
        steer=actions[0]
        throttle=actions[1]
        control=carla.VehicleControl(throttle,steer,0)
        self.vehicle.apply_control(control)

        speed=self.vehicle.get_velocity()
        curr_trans=self.vehicle.get_transform()
        wps=self.map.get_waypoint(self.vehicle.get_location())
        nwp=np.random.choice(wps.next(0.3))
        return speed,curr_trans,nwp.transform
        
    def close(self):
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actors])
        

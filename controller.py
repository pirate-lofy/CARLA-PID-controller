import numpy as np
import math
import sys
from collections import deque

sys.path.append("carla-0.9.5-py3.5-linux-x86_64.egg")
import carla


def value(v):
    return np.sqrt(v.x**2+v.y**2)

def get_speed(v):
    return 3.6*value(v)

class PID:
    def __init__(self,KP,KI,KD,min_v,dt=0.03):
        self.KP=KP
        self.KI=KI
        self.KD=KD
        self.min_v=min_v
        self.dt=dt
        self.errors=deque(maxlen=10)

    def run(self,err):
        self.errors.append(err)
        p_e=err

        if len(self.errors)>=2:
            e_d=(self.errors[-1]-self.errors[-2])/self.dt
            e_i=sum(self.errors)*self.dt
        else:
            e_d=e_i=0
        update=self.KP*p_e+(self.KI*e_i)/self.dt+self.KD*e_d*self.dt
        return np.clip(update,self.min_v,1)

class Controller:
    def __init__(self,steer_args=None,speed_args=None):
        if steer_args is None:
            steer_args={'KP':1.,'KI':0.,'KD':0.,'min_v':-1}
        if speed_args is None:
            speed_args={'KP':1.,'KI':0.,'KD':0.,'min_v':0}
        
        self.steer_pid=PID(**steer_args)
        self.speed_pid=PID(**speed_args)


    def run(self,target_speed,curr_speed,nt,curr_t):
        curr_speed=get_speed(curr_speed)
        speed_err=target_speed-curr_speed
        speed=self.speed_pid.run(speed_err)

        steer_err=self.prepare_steer(nt,curr_t)
        steer=self.steer_pid.run(steer_err)
        
        return speed,steer


    def prepare_steer(self,nt,ct):
        loc=ct.location
        rot=ct.rotation
        nloc=nt.location
        nrot=nt.rotation

        v_end=loc+carla.Location(math.cos(math.radians(rot.yaw)),math.sin(math.radians(rot.yaw)))
        v=np.array([v_end.x-loc.x,v_end.y-loc.y,0.])
        w=np.array([nloc.x-loc.x,nloc.y-loc.y,0.])
        res=np.dot(w,v)/np.linalg.norm(w)*np.linalg.norm(v)
        res=math.acos(np.clip(res,-1.,1.))
        cross=np.cross(v,w)
        if cross[2]<0.:
            res*=-1.

        return res


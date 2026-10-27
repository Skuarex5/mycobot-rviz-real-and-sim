import yaml,numpy as np,xml.etree.ElementTree as ET,json
from scipy.spatial.transform import Rotation
import sys
p=yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
r=ET.fromstring(p['robot_description']);joints={j.find('child').get('link'):j for j in r.findall('joint')}
names=['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3','joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6']
q=np.array([.48153634,-.64263023,-.64716809,-.43563418,.2560398,-.1993166])
def fk(q):
 chain=[];link='gripper_tool_frame'
 while link in joints:
  j=joints[link];chain.append(j);link=j.find('parent').get('link')
 t=np.eye(4);vals=dict(zip(names,q))
 for j in reversed(chain):
  o=j.find('origin');a=np.eye(4)
  if o is not None:
   a[:3,3]=[float(v) for v in o.get('xyz','0 0 0').split()];a[:3,:3]=Rotation.from_euler('xyz',[float(v) for v in o.get('rpy','0 0 0').split()]).as_matrix()
  t=t@a
  if j.get('name') in vals:
   axis=np.array([float(v) for v in j.find('axis').get('xyz').split()]);a=np.eye(4);a[:3,:3]=Rotation.from_rotvec(axis*vals[j.get('name')]).as_matrix();t=t@a
 return t
from mycobot_280_hardware.hardware_session import HardwareSession
from scipy.optimize import least_squares
base=fk(q)
for z in [.001,.003,.005,.01,.02]:
 target=base.copy();target[2,3]+=z
 def error(x):
  t=fk(x);return np.r_[10*(t[:3,3]-target[:3,3]),Rotation.from_matrix(target[:3,:3].T@t[:3,:3]).as_rotvec()]
 result=least_squares(error,q,bounds=(q-.35,q+.35),gtol=1e-12,ftol=1e-12,xtol=1e-12)
 d=result.x-q;old=np.clip(d,-.015,.015);old*=min(1,.012/sum(abs(old)))
 s=HardwareSession([-4.]*6,[4.]*6,command_period=.2,shared_step_budget=True)
 token=s.select(q,1.);s.start_live(token,q,1.);s.command(token,1,result.x,1.,1.,1.)
 uniform=np.array(s.next_command(q,1.2))-q
 print(json.dumps(dict(z=z,error=float(np.linalg.norm(error(result.x))),goal_delta=d.tolist(),old_step=old.tolist(),old_xyz=(fk(q+old)[:3,3]-base[:3,3]).tolist(),uniform_xyz=(fk(q+uniform)[:3,3]-base[:3,3]).tolist())))

"""Isolated integration only: terminates the FakeUSB process, never real USB."""
import importlib.util,os,signal,threading,time
assert os.environ.get('ROS_DOMAIN_ID') == '87', 'Isolated test domain required'
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from mycobot_280_control_interfaces.msg import HardwareStatus
from mycobot_280_hardware.robot_monitor_node import DEFAULT_JOINT_NAMES
p='/root/mycobot_ros2_packages/mycobot_280/mycobot_280_hardware/test/unified_control_integration.py'
s=importlib.util.spec_from_file_location('probe',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
rclpy.init();n=m.Probe();e=MultiThreadedExecutor(4);e.add_node(n);t=threading.Thread(target=e.spin,daemon=True);t.start()
pub=n.create_publisher(HardwareStatus,'/physical/control/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 m.until(lambda:'simulation' in n.states and n.hw.connected)
 assert n.service('/mycobot_control/select_physical',True).success
 m.until(lambda:n.selected)
 fake_pids=[]
 for name in os.listdir('/proc'):
  if not name.isdigit():continue
  try:
   args=open('/proc/'+name+'/cmdline','rb').read().split(b'\0')
   env=open('/proc/'+name+'/environ','rb').read()
   if args[0].endswith(b'python3') and any(a.endswith(b'/test/fake_hardware.py') for a in args) and b'ROS_DOMAIN_ID=87\0' in env:fake_pids.append(int(name))
  except OSError:pass
 assert len(fake_pids)==1, fake_pids
 fault=HardwareStatus(driver_id=n.hw.driver_id,status_sequence=n.hw.status_sequence+100000,connected=False,selected=False,live=False,fault=True,message='Fake USB disconnected; stop unverified')
 os.kill(fake_pids[0],signal.SIGINT)
 time.sleep(.5)
 pub.publish(fault)
 m.until(lambda:not n.hw.connected)
 time.sleep(.3)
 assert n.selected is True
 print('PASS physical loss keeps physical destination until explicit user choice',flush=True)
 reply=n.service('/mycobot_control/select_physical',False)
 assert reply.success,reply.message
 m.until(lambda:n.selected is False)
 q=[n.states['simulation'][k] for k in DEFAULT_JOINT_NAMES];q[0]+=.03
 trajectory=n.plan(q)
 from moveit_msgs.action import ExecuteTrajectory
 result=n.action(ExecuteTrajectory,'/execute_trajectory',ExecuteTrajectory.Goal(trajectory=trajectory))
 assert result.error_code.val==1
 assert n.hw.fault
 print('PASS disconnected driver absent: return to Gazebo, Plan, Execute; physical fault preserved',flush=True)
finally:
 e.shutdown();t.join(timeout=2);n.destroy_node();rclpy.shutdown()

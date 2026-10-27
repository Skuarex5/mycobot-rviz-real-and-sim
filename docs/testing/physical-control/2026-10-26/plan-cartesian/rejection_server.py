import sys,time
import rclpy
from mycobot_280_hardware.control_coordinator import ControlCoordinator
class OldCoordinator(ControlCoordinator):
 def accept_moveit(self, request):return self.accept_job(request)
rclpy.init();n=(OldCoordinator if '--old' in sys.argv else ControlCoordinator)()
n.policy.live=True
n.timer.cancel()
def fresh():
 now=time.monotonic();n.sources['simulation'].update(n.names,[0.]*len(n.names),now,now)
n.create_timer(.1,fresh);fresh();print('Ready: no hardware, live policy active',flush=True)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
n.destroy_node();rclpy.shutdown()

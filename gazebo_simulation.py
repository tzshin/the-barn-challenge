import rospy
import numpy as np

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState, ODEPhysics
from gazebo_msgs.srv import SetModelState, GetModelState, SetPhysicsProperties
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

def create_model_state(x, y, z, angle):
    # the rotation of the angle is in (0, 0, 1) direction
    model_state = ModelState()
    model_state.model_name = 'jackal'
    model_state.pose.position.x = x
    model_state.pose.position.y = y
    model_state.pose.position.z = z
    model_state.pose.orientation = Quaternion(0, 0, np.sin(angle/2.), np.cos(angle/2.))
    model_state.reference_frame = "world"

    return model_state


class GazeboSimulation():

    def __init__(self, init_position = [0, 0, 0]):
        self._pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self._init_model_state = create_model_state(init_position[0],init_position[1],0,init_position[2])
        
        self.collision_count = 0
        self._collision_sub = rospy.Subscriber("/collision", Bool, self.collision_monitor)

    def make_physics_easier(self, max_step_size=0.005, real_time_update_rate=200):
        """
        Makes physics simulation lighter and easier to run!
        Perfect for development when you don't need perfect physics.
        
        Args:
            max_step_size (float): Time duration in seconds for each physics update step
            real_time_update_rate (int): The number of physics update attempts per second (Hz)
        
        Returns:
            bool: True if we successfully made physics easier, False if not
        """
        try:
            rospy.loginfo("🚀 Attempting to make physics simulation lighter...")
            rospy.wait_for_service('/gazebo/set_physics_properties', timeout=5)
            set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
            
            ode_config = ODEPhysics()
            
            ode_config.auto_disable_bodies = False
            ode_config.sor_pgs_precon_iters = 0
            ode_config.sor_pgs_iters = 50
            ode_config.sor_pgs_w = 1.3
            ode_config.sor_pgs_rms_error_tol = 0.0
            ode_config.contact_surface_layer = 0.001
            ode_config.contact_max_correcting_vel = 0.0
            ode_config.cfm = 0.0
            ode_config.erp = 0.2
            ode_config.max_contacts = 20
            
            gravity = Vector3()
            gravity.x = 0.0
            gravity.y = 0.0
            gravity.z = -9.8
            
            result = set_physics(
                max_step_size,
                real_time_update_rate, 
                gravity, 
                ode_config
            )
            
            if result.success:
                rospy.loginfo("✨ Physics made lighter successfully! Your CPU will thank you.")
            else:
                rospy.logwarn("😕 Couldn't make physics lighter through service call.")
                
            return result.success
            
        except rospy.ROSException as e:
            rospy.logerr(f"💥 Making physics lighter failed: {str(e)}")
            return False
        
    def collision_monitor(self, msg):
        if msg.data:
            self.collision_count += 1
    
    def get_hard_collision(self):
        # hard collision count since last call
        collided = self.collision_count > 0
        self.collision_count = 0
        return collided

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self._pause()
        except rospy.ServiceException:
            print ("/gazebo/pause_physics service call failed")

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self._unpause()
        except rospy.ServiceException:
            print ("/gazebo/unpause_physics service call failed")

    def reset(self):
        """
        /gazebo/reset_world or /gazebo/reset_simulation will
        destroy the world setting, here we used set model state
        to put the model back to the origin
        """
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            self._reset(self._init_model_state)
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/set_model_state service call failed")

    def get_laser_scan(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('front/scan', LaserScan, timeout=5)
            except:
                pass
        return data

    def get_model_state(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            return self._model_state('jackal', 'world')
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/get_model_state service call failed")

    def reset_init_model_state(self, init_position = [0, 0, 0]):
        """Overwrite the initial model state
        Args:
            init_position (list, optional): initial model state in x, y, z. Defaults to [0, 0, 0].
        """
        self._init_model_state = create_model_state(init_position[0],init_position[1],0,init_position[2])
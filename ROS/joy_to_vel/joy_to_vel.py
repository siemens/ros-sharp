import rospy
import sensor_msgs
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def joy_cb(joy, pub_vel):
  vel = Twist()
  vel.linear.x = l_scale * joy.axes[1]
  vel.linear.y = l_scale * joy.axes[0]
  vert_vel = -(1.-joy.axes[2])/2. + (1.-joy.axes[5])/2.
  vel.linear.z = l_scale * vert_vel
  vel.angular.z = a_scale*joy.axes[3]
  pub_vel.publish(vel)


if __name__ == '__main__':

  name ='teleop'
  rospy.init_node(name)

  l_scale = rospy.get_param('scale_linear', 5.0)
  a_scale = rospy.get_param('scale_angular', 15.0)
  
  pub = rospy.Publisher("cmd_vel", Twist)
  sub = rospy.Subscriber("joy", Joy, lambda m: joy_cb(m, pub))

  rospy.spin()

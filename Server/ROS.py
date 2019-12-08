import rospy
from std_msgs.msg import String

def publish(topic,signal):
    pub = rospy.Publisher(topic, String, queue_size=10)
    rospy.init_node('MSRR', anonymous=True)
    rospy.loginfo('To '+topic+': '+signal)
    pub.publish(signal)

def callback(data):
    rospy.loginfo("Received: %s", data.data)
    
def listen():
    rospy.init_node('ModuleListner', anonymous=True)
    for i in range(64):
        rospy.Subscriber("Module"+str(i), String, callback)
    rospy.spin()
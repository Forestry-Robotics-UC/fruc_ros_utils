#!/usr/bin/env python2
import rospy
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class transform_pcl():
	def __init__(self):
		# Create tf buffer and listener to change frame_id
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		
		# Change this to new frame wanted
		self.newFrame_id = rospy.get_param('~newFrame_id','front_lslidar')
		# pcl topic to change frame
		cloud_topic = rospy.get_param('~cloud_topic', '/fused_point_cloud') 
		print(cloud_topic)
		# Subscriber topic
		rospy.Subscriber(cloud_topic, PointCloud2, self.transform_pcl, 
								queue_size=10)
		# Publisher
		self.cloud_pub = rospy.Publisher('transformed_pcl', 
										 PointCloud2, queue_size=10)

	def transform_pcl(self, msg):
		try:
			# Find translation matrix between the 2 frames
		    trans = self.tf_buffer.lookup_transform(self.newFrame_id, 
		    								   msg.header.frame_id,
		                                       msg.header.stamp,
		                                       rospy.Duration(2.0))
		except tf2.LookupException as ex:
		    rospy.logwarn(ex)
		    return
		except tf2.ExtrapolationException as ex:
		    rospy.logwarn(ex)
		    return
		    # publish pointcloud with new frame_id
		self.cloud_pub.publish(do_transform_cloud(msg, trans))


def main():
  node = "transform_pcl"
  rospy.init_node(node)
  print ("Initialized node: " + str(node))

  # Run new class
  t_pcl = transform_pcl()

  # Spin until ctrl + c
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
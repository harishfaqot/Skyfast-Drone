#!/usr/bin/env python3
import rospy
from geographic_msgs.msg import GeoPointStamped

def publish_origin():
    rospy.init_node("set_gp_origin_pub", anonymous=True)
    pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
    rospy.sleep(1)  # tunggu publisher siap

    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.position.latitude = -6.2279296
    msg.position.longitude = 106.7878919
    msg.position.altitude = 0.0

    rospy.loginfo("Publishing set_gp_origin...")
    pub.publish(msg)

if __name__ == "__main__":
    try:
        publish_origin()
    except rospy.ROSInterruptException:
        pass
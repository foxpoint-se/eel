def publish_once_in_cmd_vel(self):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    while not self.ctrl_c:
        connections = self.vel_publisher.get_num_connections()
        if connections > 0:
            self.vel_publisher.publish(self.cmd)
            # rospy.loginfo("Cmd Published")
            break
        else:
            self.rate.sleep()

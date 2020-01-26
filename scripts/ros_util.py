import rospy

def get_param(param):
    
    if rospy.has_param(param):
        return rospy.get_param(param)
    else:
        return rospy.get_param("~" + param)


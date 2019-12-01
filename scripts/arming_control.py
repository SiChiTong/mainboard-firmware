import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class VehicleArming(object):
    def __init__(self):
        self.proxy = rospy.ServiceProxy("/turquoise/set_arming", SetBool)
        self.rate = rospy.Rate(10)
    
    def set_arming(self, data):
        req = SetBoolRequest()
        req.data = data
        success = False
        resp = None
        while not success:
            resp =  self.proxy(req)
            success = resp.success
            self.rate.sleep()
        return resp
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class DVLPower(object):
    def __init__(self):
        self.proxy = rospy.ServiceProxy("/turquoise/dvl/set_power", SetBool)
        self.rate = rospy.Rate(10)
    
    def set(self, data):
        req = SetBoolRequest()
        req.data = data
        success = False
        resp = None
        while not success:
            resp =  self.proxy(req)
            success = resp.success
            self.rate.sleep()
        return resp
    
    def on(self):
        self.set(True)

    def off(self):
        self.set(False)
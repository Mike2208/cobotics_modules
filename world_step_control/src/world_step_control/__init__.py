from world_step_control.PyStepModule import *
from world_step_control import PyStepModule, msg
import rospy

class StepModule(PyStepModule._PyStepModule):
    def _PyExecuteStep(self, sec, nsec):
        res = self.ExecuteStep(rospy.Time(sec,nsec))
        return PyStepModule._PyExecResult(res.PauseTime.secs, res.PauseTime.nsecs, \
            res.ExecutionTime.secs, res.ExecutionTime.nsecs, \
            res.ModuleName)

    # def ExecuteStep(self, time):
    #     print("Test vfunc")
    #     ret = msg.ModuleExecutionResult()
    #     ret.ExecutionTime = rospy.Time(0.01)
    #     return ret
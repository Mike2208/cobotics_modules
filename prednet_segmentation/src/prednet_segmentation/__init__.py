import os
import rospkg

__dir_path = os.path.dirname(os.path.realpath(__file__))
__ros_path = rospkg.RosPack().get_path("prednet_segmentation")

# File names used by prednet module
ts_model_dir = __ros_path + "/" + "ckpt"


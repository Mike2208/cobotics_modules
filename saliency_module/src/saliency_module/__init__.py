import os
import rospkg

__dir_path = os.path.dirname(os.path.realpath(__file__))
__ros_path = rospkg.RosPack().get_path("saliency_module")

# File names used by saliency module
model_salicon_gpu = __ros_path + "/src/saliency_module/" +  "model_salicon_gpu.pb"
test_input        = __ros_path + "/test/" +  "test_input.jpg"
test_input_gz_01  = __ros_path + "/test/" +  "test_input_gz_01.jpg"
test_input_gz_02  = __ros_path + "/test/" +  "test_input_gz_02.jpg"
test_input_gz_03  = __ros_path + "/test/" +  "test_input_gz_03.jpg"
test_input_gz_04  = __ros_path + "/test/" +  "test_input_gz_04.jpg"

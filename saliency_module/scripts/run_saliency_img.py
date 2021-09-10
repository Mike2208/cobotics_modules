#!/usr/bin/env python

from world_step_control.srv import RegisterModule
from world_step_control.msg import ModuleExecutionResult

from world_step_control import StepModule
import rospy
import importlib.resources as pkg_resources
import saliency_module

import cv2
import numpy as np
import tensorflow as tf
import time


class SaliencyModule(StepModule):
    def __init__(self, name, gpu_factor):
        super().__init__(name)

        self.graph_def = tf.GraphDef()

        # either model_salicon_cpu.pb or model_salicon_gpu.pb
        # with pkg_resources.path(saliency_module, "model_salicon_gpu.pb") as pb_name:
        #     with tf.gfile.Open(pb_name, "rb") as file:
        #         self.graph_def.ParseFromString(file.read())
        with tf.gfile.Open(saliency_module.model_salicon_gpu, "rb") as file:
            self.graph_def.ParseFromString(file.read())

        self.input_plhd = tf.placeholder(tf.float32, (None, None, None, 3))

        [self.predicted_maps] = tf.import_graph_def(self.graph_def,
                                            input_map={"input": self.input_plhd},
                                            return_elements=["output:0"])

        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=gpu_factor)
        self.sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))

        # with pkg_resources.path(saliency_module, "test_input.jpg") as im_name:
        #     self.image = cv2.imread(im_name)
        #     self.image = cv2.resize(self.image, (320, 240))
        #     self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        #     self.image = self.image[np.newaxis, :, :, :]
        self.image = cv2.imread(saliency_module.test_input)
        self.image = cv2.resize(self.image, (320, 240))
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.image = self.image[np.newaxis, :, :, :]

        self.saliency = self.sess.run(self.predicted_maps,
                                      feed_dict={self.input_plhd: self.image})

    def ExecuteStep(self, time):
        print("Running Saliency")

        image_gz_01 = cv2.imread(saliency_module.test_input_gz_01)
        image_gz_01 = cv2.resize(image_gz_01, (320, 240))
        image_gz_01 = cv2.cvtColor(image_gz_01, cv2.COLOR_BGR2RGB)
        image_gz_01 = image_gz_01[np.newaxis, :, :, :]

        self.saliency = self.sess.run(self.predicted_maps,
                                      feed_dict={self.input_plhd: image_gz_01})

        self.saliency = cv2.cvtColor(self.saliency.squeeze(),
                                     cv2.COLOR_GRAY2BGR)

        self.saliency = cv2.resize(self.saliency, (640, 480))
        self.saliency = np.uint8(self.saliency * 255)

        res = ModuleExecutionResult()
        res.PauseTime = rospy.Time(0)
        res.ExecutionTime = rospy.Time(0.1)

        return res

    # def ExecuteStep(self, time):
    #     print("Saliency Module Executing")

    #     res = ModuleExecutionResult()
    #     res.PauseTime = rospy.Time(0)
    #     res.ExecutionTime = rospy.Time(0.1)

    #     return res


if __name__ == "__main__":
    try:
        module_name = "saliency_module"
        rospy.init_node(module_name)

        gpu_fact = rospy.get_param("saliency_gpu_factor")

        module = SaliencyModule(module_name, gpu_fact)
        #module.ExecuteStep(rospy.Time().now())

        while not rospy.is_shutdown():
            module.RunOnce()
            time.sleep(1.0/60.0)
    
    except rospy.ROSInterruptException:
         pass
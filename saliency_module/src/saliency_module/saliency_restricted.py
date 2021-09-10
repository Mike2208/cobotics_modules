import cv2
import numpy as np
import tensorflow as tf

graph_def = tf.GraphDef()

# either model_salicon_cpu.pb or model_salicon_gpu.pb
with tf.gfile.Open("model_salicon_gpu.pb", "rb") as file:
    graph_def.ParseFromString(file.read())

input_plhd = tf.placeholder(tf.float32, (None, None, None, 3))

[predicted_maps] = tf.import_graph_def(graph_def,
                                       input_map={"input": input_plhd},
                                       return_elements=["output:0"])

window_name = "Saliency Output"

gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)

with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
    image = cv2.imread("test_input.jpg")
    image = cv2.resize(image, (320, 240))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image[np.newaxis, :, :, :]

    saliency = sess.run(predicted_maps,
                        feed_dict={input_plhd: image})

    saliency = cv2.cvtColor(saliency.squeeze(),
                            cv2.COLOR_GRAY2BGR)

    saliency = cv2.resize(saliency, (640, 480))
    saliency = np.uint8(saliency * 255)

    #cv2.imshow("test_output", saliency)
    #cv2.waitKey(0)

    cv2.imshow(window_name, saliency)
    while cv2.waitKey(100) == -1:
        pass


import cv2
import numpy as np
import tensorflow as tf
import time

graph_def = tf.GraphDef()

# either model_salicon_cpu.pb or model_salicon_gpu.pb
with tf.gfile.Open("model_salicon_gpu.pb", "rb") as file:
    graph_def.ParseFromString(file.read())

input_plhd = tf.placeholder(tf.float32, (None, None, None, 3))

[predicted_maps] = tf.import_graph_def(graph_def,
                                       input_map={"input": input_plhd},
                                       return_elements=["output:0"])

window_name = "Saliency Output"
#cv2.cv.NamedWindow(window_name)

with tf.Session() as sess:
    image = cv2.imread("test_input.jpg")
    image = cv2.resize(image, (320, 240))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image[np.newaxis, :, :, :]

    #h = sess.partial_run_setup([predicted_maps], [input_plhd])
    #saliency = sess.partial_run(h, predicted_maps,
    #                    feed_dict={input_plhd: image})

    # Run at least once to set everything up
    saliency = sess.run(predicted_maps,
                        feed_dict={input_plhd: image})

    start_time = time.time()
    saliency = sess.run(predicted_maps,
                        feed_dict={input_plhd: image})
    run_time = time.time() - start_time
    print("Execution Time: %s seconds" % run_time)

    saliency = cv2.cvtColor(saliency.squeeze(),
                            cv2.COLOR_GRAY2BGR)

    saliency = cv2.resize(saliency, (640, 480))
    saliency = np.uint8(saliency * 255)

    cv2.imshow(window_name, saliency)

    while cv2.waitKey(100) == -1:
        pass

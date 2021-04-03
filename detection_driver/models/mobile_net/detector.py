import numpy as np
import tensorflow.compat.v1 as tf
import cv2
import time
import imutils
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
tf.logging.set_verbosity(tf.logging.ERROR)
tf.logging.set_verbosity(tf.logging.ERROR)

class DetectorAPI:
    def __init__(self, path_to_ckpt, threshold, draw_boxes):
    
        self.path_to_ckpt = path_to_ckpt
        self.threshold = threshold
        self.draw_boxes = draw_boxes

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.default_graph = self.detection_graph.as_default()
        self.sess = tf.Session(graph=self.detection_graph)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def processFrame(self, image):
        image_np_expanded = np.expand_dims(image, axis=0)
        start_time = time.time()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        duration = time.time() - start_time
        im_height, im_width,_ = image.shape
        boxes_list = [None for i in range(boxes.shape[1])]
        for i in range(boxes.shape[1]):
            boxes_list[i] = (int(boxes[0,i,0] * im_height),
			int(boxes[0,i,1]*im_width),
			int(boxes[0,i,2] * im_height),
			int(boxes[0,i,3]*im_width))

        return boxes_list, scores[0].tolist(), [int(x) for x in classes[0].tolist()], int(num[0]), duration

    def close(self):
        self.sess.close()
        self.default_graph.close()
    
    def single_image_det(self, image):
        boxes, scores, classes, num, duration = self.processFrame(image)
        if self.draw_boxes:
            for i in range(len(boxes)):
                if classes[i] == 1 and scores[i] > self.threshold:
                    box = boxes[i]
                    cv2.rectangle(image,(box[1],box[0]),(box[3],box[2]),(134,235,52),2)
                    cv2.rectangle(image, (box[1],box[0]-30),(box[1]+125,box[0]),(134,235,52), thickness=cv2.FILLED)
                    cv2.putText(image, '  Person '+str(round(scores[i],2)), (box[1],box[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225,255,225), 1)
        return image, boxes, scores, num, duration

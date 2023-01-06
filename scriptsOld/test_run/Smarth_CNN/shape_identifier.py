import tensorflow as tf
import numpy as np
# import cv2

class ShapeIdentifier:
    IMAGE_HEIGHT = 100
    IMAGE_WIDTH = 100
    
    def __init__(self, path = ''):
        self.model = tf.keras.models.load_model( r"D:\Robotics\FlipkartD2C\grid 3.0\test_run\Smarth_CNN\shape_identification_model_v3")

    def predict(self, image):
        '''
        image: 2D numpy array of the image to be classified. It should be 100 x 100
        return: 0 for circle.   1 for triangle.
        '''
        # return int(tf.argmax(self.model.predict(image.reshape(-1,self.IMAGE_HEIGHT,self.IMAGE_WIDTH,1)), axis = 1)[0])
        prediction = self.model(np.array([image.reshape(self.IMAGE_HEIGHT,self.IMAGE_WIDTH,1)]),training=False)
        if prediction[0][1] > prediction[0][0]:
            return 1
        else: 
            return 0

shape_identifier = ShapeIdentifier()
# img2 = cv2.imread('ML_training_set_circle/15657.png', cv2.IMREAD_GRAYSCALE)
# print(shape_identifier.predict(img2))
#G:\SOFTWARE BASED PROJECTS\ARDUINO\fg_car\test_run\Smarth_CNN\shape_identifier.py
# G:/SOFTWARE BASED PROJECTS/ARDUINO/fg_car/test_run/Smarth_CNN/shape_identification_model_v3
import cv2
import numpy as np
from keras.models import load_model


class FireDetection:
    def __init__(self, model_path):
        self.model = load_model(model_path)
        self.firelist = ["FIRE", "NOT A FIRE"]

    def process_frame(self, frame):
        # Preprocess the frame
        resized = cv2.resize(frame, (86, 48))

        # Use the model to predict whether there is fire in the frame
        prediction = self.model.predict(np.expand_dims(resized, axis=0))
        # print(prediction)
        # label = self.firelist[np.argmax(prediction)]
        if prediction[0][0] > 0.98:
            label = "FIRE"
        else:
            label = "NOT A FIRE"

        if label == "FIRE":
            return True
        else:
            return False


if __name__ == "__main__":
    # Start capturing the camera feed
    cap = cv2.VideoCapture(0)

    # Initialize the class
    fire = FireDetection("DATASET/Models/candle.h5")

    try:
        while True:
            ret, frame = cap.read()
            # if frame is available, process it:
            if ret:
                print(fire.process_frame(frame))
    except KeyboardInterrupt:
        # Release the camera
        cap.release()

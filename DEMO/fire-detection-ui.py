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

        if prediction[0][0] > 0.90:
            label = "FIRE"
        else:
            label = "NOT A FIRE"

        if label == "FIRE":
            return True
        else:
            return False


def main():
    cap = cv2.VideoCapture(0)
    # Initialize the class
    fire = FireDetection("DATASET/Models/candle.h5")
    try:
        while True:
            ret, frame = cap.read()
            # if frame is available, process it:
            if ret:
                is_fire = fire.process_frame(frame)
                if is_fire:
                    # Convert the frame to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    # Apply thresholding
                    _, threshold = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
                    # Find contours
                    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        # Find the contour with the largest area
                        largest_contour = max(contours, key=cv2.contourArea)
                        # Get the bounding rectangle
                        x, y, w, h = cv2.boundingRect(largest_contour)

                        # Draw an inner rectangle around the fire region
                        inner_rect_x = x + 20  # Adjust the left offset
                        inner_rect_y = y + 20  # Adjust the top offset
                        inner_rect_w = w - 40  # Adjust the width
                        inner_rect_h = h - 40  # Adjust the height
                        cv2.rectangle(frame, (inner_rect_x, inner_rect_y), (inner_rect_x + inner_rect_w, inner_rect_y + inner_rect_h), (0, 255, 0), 2)

                        # Write the label FIRE
                        cv2.putText(frame, "FIRE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

                        

                cv2.imshow("Fire Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        # Release the camera
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

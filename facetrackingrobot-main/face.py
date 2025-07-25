import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
import time
from esp32tst import cam  # Ensure this function returns a valid OpenCV frame
import time




# Initialize Face Detection
mp_drawing = mp.solutions.drawing_utils
mp_face = mp.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)

# Frame dimensions
width, height = 640, 480
mqttBroker = "192.168.45.170"

# MQTT Client Setup
client = mqtt.Client("windows_pc")
client.connect(mqttBroker)



def publish_message(topic, message):
    """Publish a message to the MQTT broker."""
    client.publish(topic, message)
    

# Movement Functions



def stop():
    publish_message("test2", "stop")

def turn_left():
    publish_message("test2", "turn_left")

def turn_right():
    publish_message("test2", "turn_right")

def forward():
    publish_message("test2", "forward")

def backward():
    publish_message("test2", "backward")

def obj_data(img):
    """Process face detection and control movements."""
    image_input = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = mp_face.process(image_input)

    if not results.detections:
        stop()
    else:
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            x, y, w, h = (int(bbox.xmin * width), int(bbox.ymin * height),
                          int(bbox.width * width), int(bbox.height * height))

            # Draw Bounding Box
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # Calculate Center of Face
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

            # Calculate movement direction
            a = cx // 62  # Horizontal position
            b = max((cy - h) // 10, 0)  # Ensure b is non-negative

            # Display position values
            cv2.putText(img, f"A: {a}", (430, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
            cv2.putText(img, f"B: {b}", (30, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            # Movement Logic
            if b > 12:
                
                forward()
            elif b < 8:
                backward()
            else:
                stop()

            if a > 6:
                turn_right()
                time.sleep(0.1)  # Quick movement
                stop()

            if a < 4:
                turn_left()
                time.sleep(0.1)
                stop()

# Main Loop
while True:
    frame = cam()


    frame = cv2.flip(frame, 1)
    frame = cv2.resize(frame, (width, height))
    
    obj_data(frame)
    
    cv2.imshow("FRAME", frame)
    
    if cv2.waitKey(1) & 0xFF == 27:  # Exit on 'Esc' key
        break
   
    if frame is None:
        print("⚠️ Error: Camera frame is empty!")
        continue

cv2.destroyAllWindows()

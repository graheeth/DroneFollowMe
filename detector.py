import cv2
import numpy as np

def load_network():
    net = cv2.dnn.readNet("yolo/yolov3-tiny.weights", "yolo/yolov3-tiny.cfg")
    with open("yolo/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layer_indexes = net.getUnconnectedOutLayers()
    output_layers = [layer_names[i - 1] for i in output_layer_indexes.flatten()]
    return net, output_layers

def detect_objects(frame, net, output_layers):
    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and class_id == 0:  # Confidence threshold and person class
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    biggest_box_center = find_biggest_box(boxes, indexes)
    if biggest_box_center:
        image_center = (width // 2, height // 2)
        distance_from_center = ( image_center[0] - biggest_box_center[0],  image_center[1] - biggest_box_center[1])
        return biggest_box_center, distance_from_center
    else:
        return None, None

def find_biggest_box(boxes, indexes):
    max_area = 0
    biggest_box_center = None
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            area = w * h
            if area > max_area:
                max_area = area
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                biggest_box_center = (center_x, center_y)
    return biggest_box_center

def main():
    net, output_layers = load_network()
    cap = cv2.VideoCapture(0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    while True:
        ret, frame = cap.read()
        height, width, _ = frame.shape
        image_center = (width // 2, height // 2)
        biggest_box_center, distance_from_center = detect_objects(frame, net, output_layers)
        if biggest_box_center:
            center_x, center_y = biggest_box_center
            cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), -1)  # Draw center of the box
            cv2.arrowedLine(frame, image_center, (center_x, center_y), (0, 0, 255), 2)  # Draw arrow from image center to box center
            cv2.putText(frame, f"Center: ({center_x}, {center_y})", (center_x, center_y - 20), font, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"Distance from center: {distance_from_center}", (10, 30), font, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Camera resolution: {width}x{height}", (10, 60), font, 0.5, (0, 255, 0), 1)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
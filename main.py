import cv2
import threading
import websocket
import json
import numpy
import os
import signal
import time
from ultralytics import YOLO
from vehicle import Vehicle
from pedestrian import Pedestrian
from queuing import ControlQueue
from video_capture import VideoCapture

window_name = "Ceiling Camera Feed"

# define intersections lane lines and border lines
boundary_lines = [
    #octagon
    [(322, 97), (379, 191)],
    [(375, 384), (325, 470)],
    [(168, 378), (217, 470)],
    [(219, 97), (170, 187)],

    # top lane
    [(221, 0), (220, 97)],
    [(274, 0), (273, 94)],
    [(327, 0), (323, 93)],

    # bottom lane
    [(217, 474), (215, 613)],
    [(271, 476), (268, 613)],
    [(324, 476), (322, 613)],

    # right lane
    [(380, 192), (589, 194)],
    [(379, 289), (591, 291)],
    [(377, 383), (589, 392)],

    # left lane
    [(27, 184), (168, 185)],
    [(23, 279), (167, 284)],
    [(23, 370), (167, 379)],

    # other lines
    [(30, 13), (221, 8)],
    [(325, 8), (589, 8)],
    [(30, 13), (16, 615)],
    [(589, 8), (588, 613)],
    [(16, 615), (588, 613)]
]

# define each lane's stop line
stop_lines = {
    'top': [(218, 95), (324, 95)],
    'bottom': [(216, 474), (325, 475)],
    'right': [(380, 192), (376, 384)],
    'left': [(168, 185), (168, 378)]
}

# define the two lines that border and are parallel to each lane
lanes = {
    'top-backward': [boundary_lines[5][0], boundary_lines[6][1]],
    'top-forward': [boundary_lines[4][0], boundary_lines[5][1]],
    'bottom-backward': [boundary_lines[7][0], boundary_lines[8][1]],
    'bottom-forward': [boundary_lines[8][0], boundary_lines[9][1]],
    'right-backward': [boundary_lines[11][0], boundary_lines[12][1]],
    'right-forward': [boundary_lines[10][0], boundary_lines[11][1]],
    'left-backward': [boundary_lines[13][0], boundary_lines[14][1]],
    'left-forward': [boundary_lines[14][0], boundary_lines[15][1]]
}

# define dictionary of cars
cars = {
    'green-car': Vehicle({'id': 'green-car', 'color': [201,197,134]}),
    'orange-car': Vehicle({'id': 'orange-car', 'color': [208,162,64]})
}

# define car websockets
websocket_uris = [
    "ws://172.20.10.10:8765",
    "ws://172.20.10.12:8765",
]

intend_turns = False
control_queue = ControlQueue()

# load pedestrian and vehicle detection Yolo v8 CNN model
model = YOLO('./data/model.pt')

ws_array = []

# control_queue.addCar(cars['green-car'], 'right')
# control_queue.addCar(cars['orange-car'], 'forward')

def exit_handler(signal, frame):
    print('\n\nCtrl+C detected. Ending Program.')
    os._exit(1)

def identifyVehicle(frame, contour):
    roi = frame[contour[1]: contour[1] + contour[3], contour[0]: contour[0] + contour[2]]
    for car_key in cars:
        car = cars[car_key]
        target_color = car.color
        threshold = 5
        lower_range = numpy.array([target_color[2]-threshold, target_color[1]-threshold, target_color[0]-threshold])
        upper_range = numpy.array([target_color[2]+threshold, target_color[1]+threshold, target_color[0]+threshold])

        mask = cv2.inRange(roi, lower_range, upper_range)
        
        if numpy.count_nonzero(mask) > 0:
            return car_key
    return 'Unidentified'

def intersectionBetweenRectangles(r1_left, r1_right, r2_left, r2_right):
    x = 0
    y = 1

    x_dist = (min(r1_right[x], r2_right[x]) - max(r1_left[x], r2_left[x]))
    y_dist = (min(r1_right[y], r2_right[y]) - max(r1_left[y], r2_left[y]))

    area_of_intersection = 0

    if x_dist > 0 and y_dist > 0:
        area_of_intersection = x_dist * y_dist
    
    return area_of_intersection

def itemIdentification(new_object, all_objects):
    for test_object in all_objects:
        intersection = intersectionBetweenRectangles([test_object.contour[0], test_object.contour[1]], [test_object.contour[2] + test_object.contour[0], test_object.contour[3] + test_object.contour[1]], [new_object.contour[0], new_object.contour[1]], [new_object.contour[2] + new_object.contour[0], new_object.contour[3] + new_object.contour[1]])
        if intersection > 0:
            return test_object
    return None

def on_message(ws, message):
    if 'pong' not in message and message != 'A':
        data = json.loads(message)
        car = cars[data['Name']]
        time.sleep(.025)
        ws.send(json.dumps({"motors": car.motor_speeds}))
        car.greyscale = data['A']
        car.currentSpeed = data['B']
        car.mileage = data['C']
        car.sonar_angle = data['D'][0]
        car.sonar_distance = data['D'][1]
        

def on_error(ws, error):
    print(f"Error occurred: {error}")

def on_close(ws, status_code, close_msg):
    print("Connection closed")

def on_open(ws):
    print("Connection opened")

def main():
    cv2.namedWindow(window_name)
    camera = VideoCapture(0) 
    pedestrian_counter = 0
    current_pedestrians = []
    index = 0

    for uri in websocket_uris:
        ws = websocket.WebSocketApp(uri,
                                    on_message = on_message,
                                    on_error = on_error,
                                    on_close = on_close,
                                    on_open = on_open)
        wst = threading.Thread(target=ws.run_forever)
        wst.daemon = True
        wst.start()
        index += 1

    while True:
        frame = camera.read()
        frame = cv2.resize(frame, (640, 640))
        result = model.predict(frame, max_det=6, verbose=False, device=0, conf=0.5, vid_stride=True)[0]

        for car_key in cars:
            if cars[car_key].time_since_visible > 0:
                cars[car_key].is_visible = False
                cars[car_key].time_since_visible = 0
            else:
                cars[car_key].time_since_visible += 1

        control_queue.control(stop_lines)

        for box in result.boxes:
            object_class = 'car' if box.cls.detach().cpu().numpy()[0] == 0 else 'pedestrian'
            coords = [int(coord) for coord in box.xyxy.detach().cpu().numpy()[0]]
            x = coords[0]
            y = coords[1]
            w = coords[2] - coords[0]
            h = coords[3] - coords[1]   
            if object_class == 'car':

                name = identifyVehicle(frame, [x, y, w, h])
                vehicle_in_queue = False
                for object in control_queue.queue:
                    if isinstance(object, Vehicle) and object.id == name:
                        vehicle_in_queue = True
                if not vehicle_in_queue and name != "Unidentified":
                    if name == 'green-car':
                        control_queue.addCar(cars[name], 'right')
                    else:
                        control_queue.addCar(cars[name], 'left')
                area = 0
                current_lane = 'Undefined'

                for lane_key in lanes:
                    lane = lanes[lane_key]
                    lane_x = lane[0][0]
                    lane_y = lane[0][1]
                    lane_w = lane[1][0] - lane_x
                    lane_h = lane[1][1] - lane_y

                    if x + w >= lane_x and x <= lane_x + lane_w and y + h >= lane_y and y <= lane_y + lane_h:
                        new_intersection = intersectionBetweenRectangles((x, y),(x + w, y + h), lane[0], lane[1])
                        if new_intersection > area:
                            area = new_intersection
                            current_lane = lane_key

                    if name != 'Unidentified':
                        cars[name].contour = [x, y, w, h]
                        cars[name].previous_lane = cars[name].lane
                        cars[name].lane = current_lane

                        if cars[name].time_since_visible < 51:
                            cars[name].is_visible = True
                            cars[name].time_since_visible = 0

                cv2.rectangle(frame, (x, y, w, h), (0, 255, 0))
                cv2.putText(frame, current_lane, (x - 20, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            elif object_class == 'pedestrian':
                
                new_pedestrian = Pedestrian({'id': 'pedestrian_' + str(pedestrian_counter)})
                new_pedestrian.contour = [x,y,w,h]

                identify_pedestrian = itemIdentification(new_pedestrian, current_pedestrians)

                if identify_pedestrian != None:
                    identify_pedestrian.contour = new_pedestrian.contour
                    new_pedestrian = identify_pedestrian
                else:
                    current_pedestrians.append(new_pedestrian)
                    pedestrian_counter += 1
                    control_queue.addPedestrian(new_pedestrian, 'bottom')

                cv2.rectangle(frame, (coords[0], coords[1], coords[2] - coords[0], coords[3] - coords[1]), (191, 0, 191))
                cv2.putText(frame, new_pedestrian.id, (x - 20, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (191, 0, 191), 2)

        [cv2.line(frame, line[0], line[1], (0, 0, 255), 3) for line in boundary_lines]
        [cv2.line(frame, stop_lines[line_key][0], stop_lines[line_key][1], (255, 0, 0), 3) for line_key in stop_lines]

        cv2.imshow(window_name, frame)

        keyCode = cv2.waitKey(1) & 0xFF
        if keyCode == 27 or keyCode == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()
    os._exit(1)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, exit_handler)
    main()
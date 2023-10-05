from vehicle import Vehicle
from pedestrian import Pedestrian
from datetime import datetime

# map current lane and intended direction to a car's destination lane
LANE_MAPPINGS = {
    'top': {
        'forward': 'bottom-backward',
        'right': 'left-backward',
        'left': 'right-backward'
    },
    'bottom': {
        'forward': 'top-backward',
        'right': 'right-backward',
        'left': 'left-backward'
    },
    'right': {
        'forward': 'left-backward',
        'right': 'top-backward',
        'left': 'bottom-backward'
    },
    'left': {
        'forward': 'right-backward',
        'right': 'bottom-backward',
        'left': 'top-backward'
    }
}

# map lane names to the midpoint on the lane's stop lines
LANE_LINE_MIDPOINTS = {
    'bottom-forward': (298, 475),
    'bottom-backward': (243, 474),
    'top-forward': (244, 95),
    'top-backward': (297, 95),
    'left-forward': (168, 329),
    'left-backward': (168, 233),
    'right-forward': (379, 240),
    'right-backward': (377, 336),
}

# create ControlQueue, a class for controlling vehicles and lane closures for pedestrians
class ControlQueue:
    def __init__(self):
        self.queue = []
        self.crossing_lanes = {
            'top': False,
            'bottom': False,
            'left': False,
            'right': False
        }
        self.started_pedestrians = []
 
    # add a car to the queue
    def addCar(self, car, direction):
        car.direction = direction
        self.queue.append(car)
        print(f'Added vehicle "{car.id}" to the queue, moving: "{car.direction}"')

    # add a pedestrian to the queue
    def addPedestrian(self, pedestrian, direction):
        pedestrian.direction = direction
        self.queue.append(pedestrian)
        print(f'Added pedestrian "{pedestrian.id}" to the queue, moving: "{pedestrian.direction}"')

    # remove an object from the queue
    def remove(self):
        if len(self.queue) > 0:
            self.queue.pop(0)
        else:
            print('Queue removal failed, there is nothing in the queue.')

    def line_intersects_line(self, l1, l2):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        A, B = l1
        C, D = l2
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    def line_intersects_rect(self, line, rect):
        x, y, w, h = rect

        # Define the four sides of the rectangle
        rect_lines = [
            [(x, y), (x + w, y)],
            [(x, y), (x, y + h)],
            [(x + w, y), (x + w, y + h)],
            [(x, y + h), (x + w, y + h)],
        ]

        # If the line intersects any side of the rectangle, return True
        for rect_line in rect_lines:
            if self.line_intersects_line(line, rect_line):
                return True

        # If we haven't returned yet, the line doesn't intersect the rectangle
        return False

    # detect if the car is intersection with a lane's stop line
    def intersection_with_stop_line(self, car, stop_lines):
        for stop_line_key in stop_lines:
            stop_line = stop_lines[stop_line_key]
            if self.line_intersects_rect(stop_line, car.contour):
                return True
        return False

    # close a lane if a pedestrian is crossing
    def pedestrian_crossing(self, pedestrian):
        self.crossing_lanes[pedestrian.direction] = True
        print('lane ' + str(pedestrian.direction) + ' closed')
        pedestrian.start_time = datetime.now()
        self.started_pedestrians.append(pedestrian)
    
    # control lane closures, and when to reopen a lane
    def control_pedestrians(self):

        # if there is a pedestrian in the queue, perform a crossing and remove that pedestrian
        while len(self.queue) > 0 and isinstance(self.queue[0], Pedestrian):
            self.pedestrian_crossing(self.queue[0])
            print('Removing Pedestrian', self.queue[0].id)
            self.queue.pop(0)
        
        # if 15 seconds have passed, reopen the lane 
        for started_pedestrian in self.started_pedestrians:
            if (datetime.now() - started_pedestrian.start_time).total_seconds() >= 15.0:
                print('lane ' + str(started_pedestrian.direction) + ' open')
                self.crossing_lanes[started_pedestrian.direction] = False
                self.started_pedestrians.remove(started_pedestrian)

    # control the lane closures and cars' movements
    def control(self, stop_lines):
        self.control_pedestrians() 
        self.control_cars(stop_lines)

    # control the cars turning
    def control_cars(self, stop_lines):

        # iterate through the queue
        for object in self.queue:
            # ensure that the object is a car
            if isinstance(object, Vehicle):
                car = object

                # if the car is in a backward lane
                #if 'backward' in car.lane and car.turning != True:
                 #   car.direction_to_motor_power('forward', 1)
                  #  continue

                if car.lane != 'Undefined':
                    at_stop_line = self.intersection_with_stop_line(car, stop_lines)

                    # if the car's intended turn has been declared, but it does not yet have a destination lane, give the car a lane
                    if car.stop_lane == '':
                        car.stop_lane = LANE_MAPPINGS[car.lane.split('-')[0]][car.direction]
                    
                    # do not allow car to move if the crosswalk in front of the car is currently occupied
                    if self.crossing_lanes[car.lane.split('-')[0]] or self.crossing_lanes[car.stop_lane.split('-')[0]]:
                        car.direction_to_motor_power('forward', 0)
                        self.turning = False

                    # if the car is in the intersection and has not reached its destination lane
                    elif at_stop_line == True and car.lane != car.stop_lane:
                        if car.direction == 'forward':
                            car.direction_to_motor_power('forward', 100)
                        if car.direction == 'right':
                            car.direction_to_motor_power('right', 100)
                        if car.direction == 'left':
                            car.direction_to_motor_power('left', 100)
                        car.turning = True

                    # if the car has reached its destination lane and completed its turn
                    elif car.lane == car.stop_lane:
                        car.completed_turn = True
                        if car.is_visible == True:
                            car.direction_to_motor_power('forward', 55)
                        # if the camera could not detect the car, stop the car
                        else:
                            print('car is not visible')
                            car.direction_to_motor_power('forward', 0)
                            self.queue.remove(car)
                        self.turning = False
                    
                    else:
                        car.direction_to_motor_power('forward', 55)
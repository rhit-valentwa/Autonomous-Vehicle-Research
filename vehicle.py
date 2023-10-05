class Vehicle:
    # give each vehicle basic characteristics         
    def __init__(self, params):
        self.id = params['id']
        self.color = params['color']
        self.motor_speeds = [0, 0, 0, 0]
        self.direction = 'right'
        self.contour = [0, 0, 0, 0]
        self.lane = 'Undefined'
        self.completed_turn = False
        self.is_visible = False
        self.time_since_visible = 0
        self.stop_lane = ''
        self.greyscale = 0
        self.currentSpeed = 0
        self.mileage = 0
        self.sonar_angle = 0
        self.sonar_distance = 0
        self.STRAIGHT_PATH = [1, 1, 1, 1]
        self.RIGHT_PATH = [1, 0.13, 1, 0.13]
        self.turning = False
        self.LEFT_PATH = [0.35, 1, 0.35, 1]

    # map a direction to the motor speeds the car will need to have to complete that maneuver 
    def direction_to_motor_power(self, car_turns, speed):
        if (isinstance(car_turns, str)):
            car_instructions = []
            if(car_turns == "right"):
                for j in range(0,4):
                    car_instructions.append(self.RIGHT_PATH[j] * speed)
            elif(car_turns == "left"):
                for j in range(0,4):
                    car_instructions.append(self.LEFT_PATH[j] * speed)
            elif(car_turns == "forward"):
                for j in range(0,4):
                    car_instructions.append(self.STRAIGHT_PATH[j] * speed)
        else:
            print("direction_to_motor_power parameter is a string for direction['right', 'left', 'forward'] and a int for speed")
        self.motor_speeds = car_instructions

    # return key information about a vehicle
    def state(self):
        return [self.id, self.motor_speeds, self.direction, self.greyscale, self.currentSpeed, self.mileage, self.sonar_angle, self.sonar_distance]
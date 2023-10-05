class Pedestrian:
    def __init__(self, params):
        self.id = params['id']
        self.direction = ''
        self.contour = [0, 0, 0, 0]
        self.start_time = None
    
    def state(self):
        return [self.id, self.direction, self.location]
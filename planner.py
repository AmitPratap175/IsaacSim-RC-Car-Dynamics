import numpy as np

class Controller:
    def __init__(self, ) -> None:
        self.steer = 0
        self.throttle = 0
    
    def xy_from_depth(self, depth, k):
    
        # Get the shape of the depth tensor
        self.H, self.W = depth.shape
        
        # Grab required parameters from the K matrix
        f = k[0][0]
        c_u = k[0][2]
        c_v = k[1][2]
        
        # Generate a grid of coordinates corresponding to the shape of the depth map
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        
        # Compute x and y coordinates
        x = ((u - c_u) * depth) / f
        y = ((v - c_v) * depth) / f
        
        return x, y
    
    def lane_midpoint(self):

        return np.array([int((self.H + 1)/2 + 1), int((self.W + 1)/2 + 1)])
    

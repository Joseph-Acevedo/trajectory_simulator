from matplotlib.pyplot import draw_if_interactive
from intruder import V_INTR
from constants import DECIMAL_PLACES
from math import fmod, isclose, sqrt, sin, cos, atan2

# Average drone flies ~12mph and a human walks at 3-4mph, jogs 4-6mph, and runs 8-10mph.
# These are very loose values though and have quite a bit of variance
V_DRONE = 5 * V_INTR

class Drone:

    def move(self, time_step):
        raise NotImplementedError("Drone class is abstract and doesn't implement a trajectory")


    def has_finished_traj(self):
        raise NotImplementedError("Drone class is abstract and doesn't implement a trajectory")

class Traj3_Drone(Drone):
    """
    This drone will move in a 'square wave' pattern. num_crosses determines how many time
    the drone will swap over to the next path, going downwards until it reaches the 
    bottom, then moving back up. If there are more than 2 paths it will move downwards
    at each 'crossing point' until it reaches the bottom, then it will begind moving back up
    """

    def __init__(self, L, D, R, N, num_crosses=4):
        # Drone will always start at top right
        self.L = L
        self.D = D
        self.R = R
        self.N = N
        self.pos = [L, (D * (N - 1))]
        self.crossing_width = round(L / (num_crosses + 1), DECIMAL_PLACES)
        self.moving_horizontally = True
        self.cross_down = True

    """
    Returns the distance to the closest crossing point. Positive results are pointing to
    crossing points behind the drone (in the +x direction since the drone moves in -x)
    """
    def dist_from_closest_cross(self):
        dist_to_prev_cross = fmod(self.pos[0], self.crossing_width)

        # x coord of previous crossing
        closest_cross = (self.pos[0] - dist_to_prev_cross)

        # if the distance from the next crossing is closer, then assign it as closest_cross
        if self.pos[0] - closest_cross > closest_cross + self.crossing_width - self.pos[0]:
            closest_cross = closest_cross + self.crossing_width

        return closest_cross - self.pos[0]

    """
    Moves the drone horizontally accoring to trajectory 3. It will always move left, but 
    if it reaches a point where it must start going downwards it will move to that point, 
    change to moving downwards
    """
    def move_horizontal(self, dist):
        self.moving_horizontally = True
        closest_cross_before_move = self.dist_from_closest_cross()

        # stop at the left bound
        self.pos[0] = max(self.pos[0] - dist, 0)
        if isclose(self.pos[0], 0):
            return

        closest_cross_after_move = self.dist_from_closest_cross()
        if (closest_cross_after_move > 0 or closest_cross_after_move == 0) and closest_cross_before_move < 0:
            # We've passed a crossing point, move to the crossing point, then to the next track
            self.pos[0] = self.pos[0] + dist + closest_cross_before_move
            # Only move vertically with the remaining movement distance after reaching crossing point
            self.move_vertical(dist + closest_cross_before_move)

    """
    Returns the distance to the closest track. Positive results are pointing to tracks in the 
    +y direction
    """
    def dist_from_closest_track(self):
        dist_to_prev_track = fmod(self.pos[1], self.D)

        closest_track = (self.pos[1] - dist_to_prev_track)

        if self.pos[1] - closest_track > closest_track + self.D - self.pos[1]:
            closest_track = closest_track + self.D

        return closest_track - self.pos[1]

    """
    Moves the drone vertically. The drone starts on the top-most track, and will move vertically
    downwards until it reaches the bottom track. Once there it will then begin moving back up to
    the top track until reaching the top and changing directions again. Will move vertically until
    it reaches the next track then will move horizontally again
    """
    def move_vertical(self, dist):
        self.moving_horizontally = False

        # First check if we need to change directions
        if self.cross_down:
            # If we were previously moving downards, but are on the bottom track, change direction
            if isclose(self.pos[1], 0):
                self.cross_down = False
        else:
            # If we were previously moving upwards, but are on the top track, change direction
            if isclose(self.pos[1], (self.D * (self.N - 1))):
                self.cross_down = True
        
        # Find the track closest to the drone
        closest_track_before_move = self.dist_from_closest_track()
        if self.cross_down:
            # If we're crossing down, move in that direction, but don't go below the bottom track
            self.pos[1] = max(0, self.pos[1] - dist)

            # Check the new distance to the closest track. If it has changed sign then we've passed it,
            # get onto the track instead
            closest_track_after_move = self.dist_from_closest_track()
            if closest_track_before_move < 0 and (closest_track_after_move > 0 or closest_track_after_move == 0):
                # Moved past track, stop on the track and continue horizontally
                self.pos[1] = self.pos[1] + dist + closest_track_before_move
                self.move_horizontal(dist + closest_track_before_move)
        else:
            # If we're crossing upwards, move in that direction, but don't go past the top track
            self.pos[1] = min((self.D * (self.N - 1)), self.pos[1] + dist)

            # Check the new distance to the closest track. If it has changed sign then we've passed it,
            # get onto the track instead
            closest_track_after_move = self.dist_from_closest_track()
            if closest_track_before_move > 0 and (closest_track_after_move < 0 or closest_track_after_move == 0):
                # Moved past track, stop on the track and continue horizontally
                self.pos[1] = self.pos[1] + (dist - closest_track_before_move)
                self.move_horizontal(dist - closest_track_before_move)

    """
    Take the next step for the given trajectory. It will decide whether to move vertically or horizontally
    based on what it was doing last move and where it is at
    """
    def move(self, time_step):
        if self.moving_horizontally:
            self.move_horizontal(V_DRONE * time_step)
        else:
            self.move_vertical(V_DRONE * time_step)
        
        # Round the positions to avoid too much floating point error
        self.pos[0] = round(self.pos[0], DECIMAL_PLACES)
        self.pos[1] = round(self.pos[1], DECIMAL_PLACES)
            
    """
    Eucl. distance between two points
    """
    def distance(self, p1, p2):
        return sqrt( ( p1[0] - p2[0] )**2 + ( p1[1] - p2[1] )**2 ) 

    """
    If the distance to the intruder is less than R, the intruder is spotted
    """
    def is_intr_spotted(self, intr_pos):
        return abs(self.distance(self.pos, intr_pos)) <= self.R


class Traj2_Drone(Drone):
    """
    This drone will move to the end of the first path before moving straight down to the next path
    and traversing it in the reverse direciton of the previous path. It creates an 'S' shaped path.
    If there are are more than two tracks it will continue to snake along the paths until it reaches 
    the end

    TODO: There is a bug, this only works for N=2. If N>2 and is odd, then it completely breaks, 
    if N>2 and is even then it behaves as if N=2. Probably needs some refactoring
    """
    
    def __init__(self, L, D, R, N):
        self.L = L
        self.D = D
        self.R = R
        self.N = N
        self.pos = [L, (D * (N - 1))]
        self.moving_horizontally = True
        self.trajectory_done = False

    """
    Moves the drone horizontally. If the drone is on an 'even' numbered path, it will 
    """
    def move_horizontally(self, dist):
        self.moving_horizontally = True
        # If we are on an even track, we're moving left
        if isclose(fmod(self.pos[1], 2*self.D), self.D):
            if self.pos[0] - dist <= 0:
                # If this move would cause the drone to exceed the left bound
                vertical_move = dist - self.pos[0]
                self.pos[0] = 0
                self.move_vertically(vertical_move)
            else:
                self.pos[0] -= dist
        else:
            # Odd track, moving right
            if self.pos[0] + dist >= self.L:
                # If this move would cause the drone to exceed the right bound
                vertical_move = (dist + self.pos[0]) - self.L
                self.pos[0] = self.L
                self.move_vertically(vertical_move)
            else:
                self.pos[0] += dist
            
    def move_vertically(self, dist):
        # Always moves downwards
        if self.pos[1] == 0:
            # We've finished traversing the last track
            if isclose(fmod(self.N, 2), 0) and isclose(self.pos[0], self.L):
                self.trajectory_done = True
                return
            elif isclose(fmod(self.N, 2), 1) and isclose(self.pos[0], 0):
                self.trajectory_done = True
                return
            else:
                self.move_horizontally(dist)
                return

        dist_to_target = 0.
        if self.move_horizontally:
            # we just finished moving horizontally
            dist_to_target = self.D
        else:
            dist_to_target = fmod(self.pos[1], self.D)

        self.moving_horizontally = False
        if dist_to_target < dist:
            # Move vertically a little and horizontally
            horizontal_move = dist - dist_to_target
            self.pos[1] = self.pos[1] - dist_to_target
            self.move_horizontally(horizontal_move)
        else:
            self.pos[1] -= dist



    def move(self, time_step):
        if self.trajectory_done:
            return

        if self.moving_horizontally:
            self.move_horizontally(time_step * V_DRONE)
        else:
            self.move_vertically(time_step * V_DRONE)

        self.pos[0] = round(self.pos[0], DECIMAL_PLACES)
        self.pos[1] = round(self.pos[1], DECIMAL_PLACES)

    def distance(self, p1, p2):
        return sqrt( ( p1[0] - p2[0] )**2 + ( p1[1] - p2[1] )**2 ) 

    def is_intr_spotted(self, intr_pos):
        return self.distance(self.pos, intr_pos) <= self.R


class Traj1_Drone(Drone):
    """
    This drone will move to the end of the first path before moving diagonally to the  next track
    and moving left again. This forms a repeating backwards 'Z' pattern
    """
    
    def __init__(self, L, D, R, N):
        self.L = L
        self.D = D
        self.R = R
        self.N = N
        self.theta = atan2(D, L)
        self.pos = [L, (D * (N - 1))]
        self.moving_horizontally = True
        self.trajectory_done = False

    """
    Moves the drone horizontally, always left
    """
    def move_horizontally(self, dist):
        self.moving_horizontally = True
        # Always moving left
        if self.pos[0] - dist <= 0:
            # If this move would cause the drone to exceed the left bound
            vertical_move = dist - self.pos[0]
            self.pos[0] = 0
            self.move_diagonally(vertical_move)
        else:
            self.pos[0] -= dist
            
    """
    Moves the drone diagonally towards the start of the next track
    """
    def move_diagonally(self, dist):
        # We've finished traversing the last track
        if isclose(self.pos[0], 0) and isclose(self.pos[1], 0):
            self.trajectory_done = True
            return

        next_target = []
        if isclose(self.pos[0], 0):
            # we just finished moving horizontally, distance to next track start
            next_target = [self.L, self.pos[1] - self.D]
        else:
            # find the next lowest track
            next_target = [self.L, self.pos[1] - fmod(self.pos[1], self.D)]
        
        dist_to_target = self.distance(self.pos, next_target)


        self.moving_horizontally = False
        # theta = atan2( (self.pos[1] - next_target[1]), (self.L - self.pos[0]) )
        if dist_to_target < dist:
            # Move diagonally a little and horizontally
            horizontal_move = dist - dist_to_target
            self.pos[0] += dist_to_target*cos(self.theta)
            self.pos[1] -= dist_to_target*sin(self.theta)
            self.move_horizontally(horizontal_move)
        else:
            self.pos[0] += dist*cos(self.theta)
            self.pos[1] -= dist*sin(self.theta)

    """
    Forces the drone to take the next step of its trajectory
    """
    def move(self, time_step):
        if self.trajectory_done:
            return

        if self.moving_horizontally:
            self.move_horizontally(time_step * V_DRONE)
        else:
            self.move_diagonally(time_step * V_DRONE)

        self.pos[0] = round(self.pos[0], DECIMAL_PLACES)
        self.pos[1] = round(self.pos[1], DECIMAL_PLACES)

    def distance(self, p1, p2):
        return sqrt( ( p1[0] - p2[0] )**2 + ( p1[1] - p2[1] )**2 ) 

    def is_intr_spotted(self, intr_pos):
        return self.distance(self.pos, intr_pos) <= self.R
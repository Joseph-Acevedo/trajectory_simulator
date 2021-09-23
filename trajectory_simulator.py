# Constants
from intruder import Intruder
from drone import Traj3_Drone, Traj2_Drone, Traj1_Drone
from constants import TIME_STEP, DECIMAL_PLACES, POINTS_TO_TEST

import sys
import threading
import numpy as np
import matplotlib.pyplot as plt

"""
Notes: 
To improve speed I could probably just have one drone since it is taking the same path
in the mass simulations and just simulate all intruders at the same time, that way I dont
have to recompute the position of the drone for every intruder

To make creating drone trajectories simpler could change it to use piecewise parametric
functions for their positions as that would probably be simpler and faster
"""



"""
Assumes N tracks, the range of y values is[0, N+(D*N)], and the range of 
x values is [0, L]. (0,0) is at the bottom right.

trajectory: A Drone trajectory class
intr_start: The coordinate that the intruder starts at
L: The length of each track
D: The distance between tracks
R: The radius the drone can see in
N: The number of tracks
"""
def simulate(trajectory, intr_start, L=10., D=3., R=1., N=2.):
    intr = Intruder(intr_start)
    drone = trajectory(L=L, D=D, R=R, N=N)

    time = 0
    # assume that if the drone finishes it's trajectory it just waits at its current pos
    while not intr.has_escaped():
        intr.move(TIME_STEP)
        drone.move(TIME_STEP)
        time = round(time + TIME_STEP, DECIMAL_PLACES)

        if drone.is_intr_spotted( intr.get_pos() ):
            return True

    # the intruder has escaped without being spotted
    return False

"""
Simulates all possible starting positions of theintruder given the length of the
track, the number of tracks, and the minimum distance between starting positions.

The total number of successes vs failures is tracked and the proportion is returned

trajectory: A Drone trajectory class
visualize: True to visualize the starting positions of the intruder and their success/failure
L: The length of each track
D: The distance between tracks
R: The radius the drone can see in
N: The number of tracks

returns: The proportion of the starting positions where the intruder escaped
"""
def mass_simulate(trajectory, visualize=True, L=10., D=3., R=1., N=2.):
    found_pos = []
    escaped_pos = []
    # Simulate on a range of x values on each track
    step_size = float(L) / POINTS_TO_TEST
    for track in range( int(N) ):
        for i in range( POINTS_TO_TEST ):
            x = step_size * i
            if simulate(trajectory, [x, track*D], L, D, R, N):
                found_pos.append( [x, track*D] )
            else:
                escaped_pos.append( [x, track*D] )
            
    proportion_found = (len(found_pos) / ( N * POINTS_TO_TEST))

    if visualize:
        print(f"Intruder found in {proportion_found * 100}% of the runs")

        x_found = [p[0] for p in found_pos]
        y_found = [p[1] for p in found_pos]

        x_esc = [p[0] for p in escaped_pos]
        y_esc = [p[1] for p in escaped_pos]

        plt.scatter(x_found, y_found, c='green', alpha=0.5, marker='.')
        plt.scatter(x_esc, y_esc, c='red', alpha=0.5, marker='.')
        plt.show()

    return proportion_found

"""
Simulates a specific starting point for the intruder, and shows the position of the 
intruder and drone at each time step

trajectory: A Drone trajectory class
intr_start: The coordinate that the intruder starts at
L: The length of each track
D: The distance between tracks
R: The radius the drone can see in
N: The number of tracks
"""
def visually_simulate(trajectory, intr_start, L=10., D=3., R=1., N=3.):
    intr = Intruder(intr_start)
    drone = trajectory(L=L, D=D, R=R, N=N)
    print(f"Simulating intruder at {intr_start} and drone at {drone.pos}")

    plt.ion()
    fig, ax = plt.subplots()
    x, y = [intr.pos[0], drone.pos[0]], [intr.pos[1], drone.pos[1]]
    sc = ax.scatter(x, y)
    plt.xlim(-1, L + 1)
    plt.ylim(-1, D * N)

    plt.draw()
    # assume that if the drone finishes it's trajectory it just waits at its current pos
    while not intr.has_escaped():
        # time.sleep(TIME_STEP)
        intr.move(TIME_STEP)
        drone.move(TIME_STEP)

        x = [intr.pos[0], drone.pos[0]]
        y = [intr.pos[1], drone.pos[1]]
        sc.set_offsets(np.c_[x,y])
        fig.canvas.draw_idle()
        plt.pause(0.01)

        if drone.is_intr_spotted( intr.get_pos() ):
            print("Intruder found")
            return True

        
    print("Finished sim")
    # the intruder has escaped without being spotted
    return False


"""
 Mass simulates the given trajectory on a variety of track lengths and plots the proportion
 of the runs where the drone catches the intruder

 trajectory: A Drone trajectory class
"""
def simulate_success_vs_length(trajectory, min_length, max_length):
    
    def length_worker(trajectory, L):
        print(f"Testing on length {L}")
        proportion = mass_simulate(trajectory, visualize=False, L=L)
        lengths.append(L)
        props.append(proportion)
        print(f"\t\t\tFinished simulating {L}->{proportion}")

    lengths = []
    props = []
    threads = []

    step_size = (max_length - min_length) / float(POINTS_TO_TEST)
    for i in range( POINTS_TO_TEST ):
        t = threading.Thread(target=length_worker, args=(trajectory, (min_length + i * step_size) ) )
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    plt.scatter(lengths, props, c='blue', alpha=0.5, marker='.')
    plt.show()


"""
Mass simulates the given trajectory on a variety of distances between tracks and plots the proportion
 of the runs where the drone catches the intruder

trajectory: A Drone trajectory class
"""
def simulate_success_vs_distance(trajectory, min_dist, max_dist):
    def dist_worker(trajectory, D):
        print(f"Testing on dist {D}")
        proportion = mass_simulate(trajectory, visualize=False, D=D)
        dists.append(D)
        props.append(proportion)
        print(f"\t\t\tFinished simulating {D}")

    dists = []
    props = []
    threads = []

    step_size = (max_dist - min_dist) / POINTS_TO_TEST
    for i in range( POINTS_TO_TEST ):
        t = threading.Thread(target=dist_worker, args=(trajectory, (min_dist + i * step_size) ) )
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    plt.scatter(dists, props, c='blue', alpha=0.5, marker='.')
    plt.show()


def main(trajectory, sim_type):
    if sim_type == "mass":
        mass_simulate(trajectory, L=35.)
    elif sim_type == "visual":
        visually_simulate( trajectory, [0, 0] , L=35.)
    elif sim_type == "length_plot":
        simulate_success_vs_length(trajectory, 10., 40.)
    elif sim_type == "dist_plot":
        simulate_success_vs_distance(trajectory, 2., 10.)



# cmd args are trajectory_simulator.py [TRAJECTORY NUM]=(1/2/3) [SIM TYPE]=(visual, mass, length_plot, dist_plot)
if __name__ == "__main__":
    if len(sys.argv) > 1:
        trajectory_id = int(sys.argv[1])
        sim_type = sys.argv[2].lower()

        if trajectory_id == 1:
            main(Traj1_Drone, sim_type)
        elif trajectory_id == 2:
            main(Traj2_Drone, sim_type)
        elif trajectory_id == 3:
            main(Traj3_Drone, sim_type)
    else:
        main(Traj2_Drone, "mass")
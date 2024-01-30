import pandas as pd
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from pyproj import Proj, transform


def processTrajectory(inputPath, outputPath):
    # Read the txt file, time x y z qx qy qz qw
    slam_traj = pd.read_table(inputPath, sep=' ')
    slam_traj = slam_traj.iloc[:, :8].to_numpy()
    slam_traj = np.column_stack((slam_traj[:, 0], slam_traj[:, 1], slam_traj[:, 2], slam_traj[:, 3], slam_traj[:, 4], slam_traj[:, 5], slam_traj[:, 6], slam_traj[:, 7]))


    T_OI = np.array([[0.9997,   -0.0225,    0.0081,    0.4144],
                     [0.0223,    0.9997,    0.0131,    0.0084],
                     [ -0.0083,   -0.0129,    0.9999,   -0.3031],
                     [0, 0, 0, 1]])  # Replace with actual R_IO matrix

    
    # to calculate imu frame to lidar frame
    # T_L(j-1)L(j) = T_OI * T_I(j-1)I(j) * T_IO
    # slam_traj include T_I(0)I(j)
    # save lidar trajectory
    # whole python code


        


if __name__ == "__main__":
# User inputs
    inputCSV = input("Enter the path of the input CSV file: ")
    outputTXT = input("Enter the path of the output file: ")

    processTrajectory(inputCSV, outputTXT)

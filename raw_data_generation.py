# -*- coding: utf-8 -*-
# Filename: demo_no_algo.py

"""
The simplest demo of Sim.
Only generate reference trajectory (position, velocity, sensor output). No algorithm.
Created on 2018-01-23
@author: dongxiaoguang
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
import logging  # Import logging for debug information

# Setting up the logger
logging.basicConfig(level=logging.INFO)  # Configure logging level
logger = logging.getLogger(__name__)  # Create a logger object

# Global constant for converting degrees to radians
D2R = math.pi / 180

# Path to motion definition files
motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency in Hz
fs_gps = 10.0       # GPS sample frequency in Hz
fs_mag = fs         # Magnetometer sample frequency (not used currently)

def plot_image():
    """
    Function to plot an image related to the simulation.
    """
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Path to image
    img = mpimg.imread(image_path)  # Load the image
    
    plt.figure(figsize=(6, 6))  # Create a new figure with specified size
    plt.imshow(img)  # Display the image
    plt.axis('off')  # Hide axis to emphasize image
    plt.title("Reference Image")  # Title for the image
    plt.show()  # Display the image

def test_path_gen():
    '''
    Test only path generation in the Sim.
    '''
    # Choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'  # Specify the accuracy of the IMU
    # Generate GPS and magnetometer data using the specified IMU model
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # Start the simulation
    sim = ins_sim.Sim(
        [fs, fs_gps, fs_mag],  # Sample frequencies for IMU, GPS, and magnetometer
        motion_def_path + "//motion_def-3d.csv",  # Path to the motion definition CSV file
        ref_frame=1,  # Reference frame (1 for ENU)
        imu=imu,  # IMU model created earlier
        mode=None,  # Simulation mode (None for default)
        env=None,  # Environment settings (default)
        algorithm=None  # No algorithms used in this demo
    )

    # Run the simulation for a specified duration (1 second in this case)
    sim.run(1)

    # Save simulation data to files
    sim.results('')  # Save results to the default path

    # Log the simulation results
    logger.info('Simulation complete, plotting data...')
     # Plot the image after the simulation results
    plot_image()  # Call the image plotting function
    # Plot data: 3D plot of reference position, 2D plots of gyro and GPS visibility
    sim.plot(['ref_pos', 'gyro', 'gps_visibility'], opt={'ref_pos': '3d'})



if __name__ == '__main__':
    test_path_gen()  # Execute the test for path generation

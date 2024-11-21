# -*- coding: utf-8 -*-
# Filename: demo_ui_ans.py

"""
Demo of using ANS as GUI
Created on 2020-02-03
@author: dongxiaoguang
"""

import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from gnss_ins_sim.gui import gui_ans

def plot_image():
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Path to image
    img = mpimg.imread(image_path)
    
    plt.figure(figsize=(6, 6))
    plt.imshow(img)
    plt.axis('off')  # Hide axis to emphasize image
    plt.title("Inclinometer Image")
    plt.show()


def test_path_gen():
    '''
    Test ANS as GUI.
    '''
    # Simulation configuration
    motion_def_path = os.path.abspath('.//demo_motion_def_files//')  # Path to motion definition files
    fs = 100.0          # IMU sample frequency in Hz
    fs_gps = 10.0       # GPS sample frequency in Hz
    fs_mag = fs         # Magnetometer sample frequency (not used currently)

    # Choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'  # Specify the accuracy of the IMU
    # Generate GPS and magnetometer data using the specified IMU model
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # Start simulation
    with open(motion_def_path + "//motion_def-3d.csv", 'r') as f:
        motion_def = f.read()  # Read the motion definition from the CSV file

    # Create the simulation object
    sim = ins_sim.Sim(
        [fs, fs_gps, fs_mag],  # Sample frequencies for IMU, GPS, and magnetometer
        motion_def,  # Motion definition data
        ref_frame=0,  # Reference frame (0 for ECEF)
        imu=imu,  # IMU model created earlier
        mode=None,  # Simulation mode (None for default)
        env=None,  # Environment settings (default)
        algorithm=None  # No algorithms used in this demo
    )

    # Run the simulation for a specified duration (1 second in this case)
    sim.run(1)

    # Save simulation data to files
    sim.results('')  # Save results to the default path

    # Initialize and start the GUI for simulation visualization
    gui = gui_ans.GuiAns()  # Create a GUI instance
    gui.start(sim)  # Start the GUI with the simulation data

    # Optionally plot an image to provide visual context
    plot_image()  # Call the image plotting function

if __name__ == '__main__':
    test_path_gen()  # Execute the test for path generation

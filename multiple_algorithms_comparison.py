import os
import platform  # Importing platform module to check the OS
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
import logging  # Importing logging to add logs

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
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Path to image
    img = mpimg.imread(image_path)  # Load the image
    
    plt.figure(figsize=(6, 6))  # Create a new figure with specified size
    plt.imshow(img)  # Display the image
    plt.axis('off')  # Hide axis to emphasize image
    plt.title("Inclinometer Image")  # Title for the image
    plt.show()  # Display the image

def test_path_gen():
    '''
    Test only path generation in the Sim.
    '''
    # Choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # Generate GPS and magnetometer data using the specified IMU model
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # Load algorithms for data processing
    # ECF based inclinometer algorithm using Mahony filter
    from demo_algorithms import inclinometer_mahony
    algo1 = inclinometer_mahony.MahonyFilter()
    
    # Tilt-based accelerometer algorithm
    from demo_algorithms import inclinometer_acc
    algo2 = inclinometer_acc.TiltAcc()

    # Start the simulation
    sim = ins_sim.Sim(
        [fs, fs_gps, fs_mag],  # Sample frequencies for IMU, GPS, and magnetometer
        motion_def_path + "//motion_def-static.csv",  # Path to the motion definition CSV file
        ref_frame=1,  # Reference frame (1 for ENU)
        imu=imu,  # IMU model created earlier
        mode=None,  # Simulation mode (None for default)
        env=None,  # Environment settings (default)
        algorithm=[algo1, algo2]  # List of algorithms to use
    )
    
    # Run the simulation for a specified duration (2 seconds in this case)
    sim.run(2)

    # Save simulation data to files
    sim.results('')  # Save results to default path
    
    # Log and plot the image first
    logger.info('Plotting image')
    plot_image()  # Call the image plotting function
    
    # Plot relevant data from the simulation
    sim.plot(['att_euler', 'ab', 'wb'])  # Plotting Euler angles and bias data

if __name__ == '__main__':
    test_path_gen()  # Execute the test for path generation

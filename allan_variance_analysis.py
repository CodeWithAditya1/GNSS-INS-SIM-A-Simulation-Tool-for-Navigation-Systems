import os
import math
import numpy as np
import matplotlib.pyplot as plt  # Import matplotlib for displaying plots
import matplotlib.image as mpimg  # Import for loading images
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# Conversion factor from degrees to radians
D2R = math.pi / 180

# Path to motion definition files
motion_def_path = os.path.abspath('.//demo_motion_def_files//')

# Sampling frequency for IMU (Inertial Measurement Unit)
fs = 100.0  # IMU sample frequency in Hz

def plot_image(image_path):
    """ Function to plot an image """
    img = mpimg.imread(image_path)
    plt.figure(figsize=(6, 6))
    plt.imshow(img)
    plt.axis('off')  # Hide axis to emphasize image
    plt.title("Inclinometer Image")
    plt.show()

def test_allan():
    '''
    Demonstrates Allan variance analysis with IMU simulation.
    '''
    # Define IMU error model parameters for a typical IMU (e.g., IMU381)
    imu_err = {
        'gyro_b': np.array([0.0, 0.0, 0.0]),                   # Gyro bias
        'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,        # Gyro angle random walk
        'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,   # Gyro bias stability
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),        # Gyro bias correlation time
        'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),         # Accelerometer bias
        'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,  # Accelerometer velocity random walk
        'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,  # Accelerometer bias stability
        'accel_b_corr': np.array([200.0, 200.0, 200.0]),       # Accelerometer bias correlation time
        'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0             # Magnetometer standard deviation
    }
    
    # Initialize the IMU model with customized error parameters and disable GPS/magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)

    # Load the Allan variance analysis algorithm from demo algorithms
    from demo_algorithms import allan_analysis
    algo = allan_analysis.Allan()

    # Start the INS simulation with Allan variance motion profile
    sim = ins_sim.Sim(
        [fs, 0.0, 0.0],                                 # [IMU frequency, GPS frequency, magnetometer frequency]
        motion_def_path + "//motion_def-Allan.csv",     # Motion profile file for Allan analysis
        ref_frame=1,                                    # Set reference frame to 1
        imu=imu,                                        # Use the customized IMU model
        mode=None,                                      # Simulation mode
        env=None,                                       # Environment (default)
        algorithm=algo                                  # Assign the Allan analysis algorithm
    )
    
    # Plot the image before running the simulation
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Update with the path to your image
    plot_image(image_path)
    
    # Run the simulation
    sim.run()
    
    # Generate simulation results and save to files (if needed)
    sim.results()  # Generates and saves results

    # Plot Allan deviation results for accelerometer and gyro data
    sim.plot(['ad_accel', 'ad_gyro'])  # Plot Allan deviation for accelerometer and gyroscope
    plt.show()  # Explicitly show the plot

if __name__ == '__main__':
    # Run the Allan variance test
    test_allan()

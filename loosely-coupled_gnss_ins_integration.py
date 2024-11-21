import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# Global constants
D2R = math.pi / 180  # Degrees to radians conversion factor
motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0  # IMU sample frequency
fs_gps = 10  # GPS sample frequency

# Function to plot the image
def plot_image():
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Replace with your image path
    img = mpimg.imread(image_path)
    
    plt.figure(figsize=(6, 6))
    plt.imshow(img)
    plt.axis('off')  # Hide axes for the image
    plt.title("Inclinometer Image")
    plt.show()

# Function to run INS simulation
def test_ins_loose():
    '''
    Test INS simulation with loosely coupled INS algorithm.
    '''
    #### IMU model, typical for IMU381
    imu_err = {
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
        'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
        'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
        'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
        'accel_b_corr': np.array([200.0, 200.0, 200.0]),
        'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
    }
    # Generate IMU data, including GPS data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True)

    #### Load and configure INS algorithm
    from demo_algorithms import ins_loose
    algo = ins_loose.InsLoose()  # Loosely coupled INS algorithm

    #### Start simulation
    sim = ins_sim.Sim(
        [fs, fs_gps, 0.0],
        motion_def_path + "//motion_def-90deg_turn.csv",
        ref_frame=0,
        imu=imu,
        mode=None,
        env=None,
        algorithm=algo
    )
    # Run the simulation
    sim.run()

    # Generate and display simulation results without saving data
    sim.results(end_point=True)

    # Plot the image first
    plot_image()
    
    # Then, display the simulation plots
    sim.plot(['ref_pos', 'pos'], opt={'ref_pos': '3d'})

if __name__ == '__main__':
    print('Still under development. Please try demo_aceinna_ins.py.')
    # Uncomment the line below to test the INS loose coupling
    # test_ins_loose()

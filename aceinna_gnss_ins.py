import os
import platform  # Importing platform module to check the OS
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
import logging  # Importing logging to add logs

# Global constant for degree-to-radian conversion
D2R = math.pi / 180.0
motion_def_path = os.path.abspath('.//demo_motion_def_files//')  # Path to motion definition files
fs = 100.0  # IMU sample frequency in Hz

# Set up logging to capture events
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
file_handler = logging.FileHandler('ins_sim.log')  # Log to a file
stream_handler = logging.StreamHandler()  # Log to console

# Format logs with timestamps and log level
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
stream_handler.setFormatter(formatter)

# Add handlers to logger
logger.addHandler(file_handler)
logger.addHandler(stream_handler)

# Function to plot an image
def plot_image():
    image_path = os.path.abspath(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Path to image
    img = mpimg.imread(image_path)
    
    plt.figure(figsize=(6, 6))
    plt.imshow(img)
    plt.axis('off')  # Hide axis to emphasize image
    plt.title("Inclinometer Image")
    plt.show()

# Function to test the DMU380 simulation with IMU and GPS data
def test_dmu380_sim():
    '''
    Test simulation with the DMU380 algorithm.
    '''
    logger.info('Starting DMU380 simulation test')

    # Define IMU error characteristics
    imu_err = {
        'gyro_b': np.array([1.0, -1.0, 0.5]) * 1800.0 * 0.0e0,  # Gyro bias
        'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0e0,       # Gyro angular random walk
        'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0e0,   # Gyro bias stability
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),          # Gyro bias correlation time
        'accel_b': np.array([5.0e-3, 5.0e-3, 5.0e-3]) * 0.0e1,   # Accelerometer bias
        'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0e0,  # Accel velocity random walk
        'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0e0,  # Accel bias stability
        'accel_b_corr': np.array([200.0, 200.0, 200.0]),         # Accel bias correlation time
        'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0              # Magnetometer standard deviation
    }
    gps_err = {
        'stdp': np.array([5.0, 5.0, 7.0]) * 0.2,  # GPS position error
        'stdv': np.array([0.05, 0.05, 0.05]) * 1.0  # GPS velocity error
    }
    odo_err = {
        'scale': 0.999,  # Odometry scale factor
        'stdv': 0.01     # Odometry velocity error
    }
    logger.info('Generating GPS and magnetometer data')
    
    # Create IMU model with errors and options for GPS and odometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True, gps_opt=gps_err,
                        odo=True, odo_opt=odo_err)

    # Import the DMU380 algorithm and configuration file
    from demo_algorithms import aceinna_ins
    cfg_file = os.path.abspath('.//demo_algorithms//dmu380_sim_lib//ekfSim_ins.cfg')
    logger.info('Loading DMU380 algorithm configuration')
    algo = aceinna_ins.DMU380Sim(cfg_file)

    # Initialize the simulation with motion profile, IMU model, and algorithm
    logger.info('Starting simulation')
    sim = ins_sim.Sim([fs, 1, fs],
                      motion_def_path + "//motion_def-Holland_tunnel.csv",
                      ref_frame=0,  # Use NED (North-East-Down) frame
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run(1)  # Run simulation with 1 iteration
    logger.info('Saving simulation results')
    
    # Save simulation results and enable KML output for visualization in Google Earth
    sim.results('.//demo_saved_data//tmp', err_stats_start=210, gen_kml=True, extra_opt='ned')

    # Plot the image first
    logger.info('Plotting image')
    plot_image()
    
    # Plot simulation results for analysis
    logger.info('Plotting simulation results')
    sim.plot(['pos', 'vel', 'wb', 'ab', 'att_euler'], opt={'pos': 'error', 'vel': 'error', 'att_euler': 'error'})

# Main execution
if __name__ == '__main__':
    # Run only on Windows due to platform-specific DLLs used by the INS algorithm
    if platform.system() == 'Windows':
        logger.info('Running on Windows')
        test_dmu380_sim()
    else:
        logger.info('Unsupported platform')
        print("Because the INS algorithm is compiled into a DLL, only Windows are supported.")
        print("For other platforms, please use the C source code.")

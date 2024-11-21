import os
import math
import numpy as np
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import matplotlib.image as mpimg  # Import for loading images
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# Global constant for converting degrees to radians
D2R = math.pi / 180

# Path to motion definition files
motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0  # IMU sample frequency in Hz

def plot_image(image_path):
    """Function to load and display an image."""
    img = mpimg.imread(r"C:\Users\Aditya Bharti\Desktop\photo.jpg")  # Load image from specified path
    plt.figure(figsize=(6, 6))  # Create a new figure with specified size
    plt.imshow(img)  # Display the image
    plt.axis('off')  # Hide axis for better visual
    plt.title("Sensor Data Overview")  # Title for the image
    plt.show()  # Show the image plot

def test_free_integration():
    '''
    Test the free integration simulation using an IMU model and motion definitions.
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

    # Odometry error model
    odo_err = {
        'scale': 0.999,
        'stdv': 0.1
    }

    # Create an IMU model without GPS data, using odometry
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False, odo=True, odo_opt=odo_err)

    #### Load Algorithms
    from demo_algorithms import free_integration_odo
    from demo_algorithms import free_integration

    # Load initial states (position, velocity, and attitude) from a CSV file
    ini_pos_vel_att = np.genfromtxt(motion_def_path + "//motion_def-90deg_turn.csv",
                                     delimiter=',', skip_header=1, max_rows=1)
    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R  # Convert latitude from degrees to radians
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R  # Convert longitude from degrees to radians
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R  # Convert attitude from degrees to radians

    # Add initial states error if needed
    ini_vel_err = np.array([0.0, 0.0, 0.0])  # Initial velocity error in body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0])  # Initial Euler angles error, degrees
    ini_pos_vel_att[3:6] += ini_vel_err  # Update initial velocity with error
    ini_pos_vel_att[6:9] += ini_att_err * D2R  # Update initial attitude with error in radians

    # Create algorithm objects for free integration using odometry and regular integration
    algo1 = free_integration_odo.FreeIntegration(ini_pos_vel_att)
    algo2 = free_integration.FreeIntegration(ini_pos_vel_att)

    #### Start Simulation
    sim = ins_sim.Sim(
        [fs, 0.0, 0.0],  # IMU frequency, GPS frequency, magnetometer frequency
        motion_def_path + "//motion_def-90deg_turn.csv",  # Path to motion definition CSV
        ref_frame=1,  # Reference frame (1 for ENU)
        imu=imu,  # IMU model created earlier
        mode=None,  # Simulation mode (None for default)
        env=None,  # Environment (default)
        algorithm=[algo1, algo2]  # List of algorithms to use
    )

    # Run the simulation for 10 time steps
    sim.run(10)

    # Generate simulation results and summary
    # Do not save data since the simulation runs for 1000 times and generates too many results
    sim.results(err_stats_start=-1, gen_kml=True)

    # Plot position error (uncomment the next line to plot)
    sim.plot(['pos'], opt={'pos': 'error'})
    plt.show()  # Ensure to show the plot after generating it

    # Plot an image to provide additional context or information
    image_path = r"C:\Users\Aditya Bharti\Desktop\photo.jpg"  # Update with the path to your image
    plot_image(image_path)  # Call the image plotting function

if __name__ == '__main__':
    test_free_integration()  # Execute the free integration test

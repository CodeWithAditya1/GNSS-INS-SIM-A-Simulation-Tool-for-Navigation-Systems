
import os
import math
import numpy as np
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import matplotlib.image as mpimg  # Import for loading images
from gnss_ins_sim.attitude import attitude
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# Directory containing logged data files
log_dir = "./demo_data_files/bosch/"
fs = 100.0          # IMU sample frequency in Hz
using_external_g = True  # Flag for using external gravity data

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
    Test the free integration simulation.
    '''
    #### No need for an IMU model in this case
    imu = None

    #### Load the Free Integration algorithm
    from demo_algorithms import free_integration
    ini_pos_vel_att = np.genfromtxt(log_dir + "ini.txt", delimiter=',')  # Load initial position, velocity, and attitude
    ini_pos_vel_att[0:2] *= attitude.D2R  # Convert latitude and longitude from degrees to radians
    ini_pos_vel_att[6:9] *= attitude.D2R  # Convert attitude from degrees to radians
    
    # If not using external gravity, keep only the first nine elements
    if not using_external_g:
        ini_pos_vel_att = ini_pos_vel_att[0:9]
    
    # Create the Free Integration algorithm object
    algo = free_integration.FreeIntegration(ini_pos_vel_att, earth_rot=False)

    # Load the Tilt Accelerometer algorithm
    from demo_algorithms import inclinometer_acc
    algo2 = inclinometer_acc.TiltAcc()

    #### Start the simulation
    sim = ins_sim.Sim(
        [fs, 0.0, 0.0],  # IMU frequency, GPS frequency, magnetometer frequency
        log_dir,  # Directory containing logged data
        ref_frame=0,  # Reference frame (0 for NED)
        imu=imu,  # IMU model (None in this case)
        mode=None,  # Simulation mode (None for default)
        env=None,  # Environment (default)
        algorithm=[algo, algo2]  # List of algorithms to use
    )

    # Run the simulation for one time step
    sim.run(1)

    # Generate simulation results and summary
    sim.results('', err_stats_start=-1, extra_opt='ned')
    # Plot an image to provide additional context or information
    image_path = os.path.abspath('path/to/your/image.jpg')  # Update with the path to your image
    plot_image(image_path)  # Call the image plotting function
    # Plot the results: position, velocity, attitude (Euler angles), acceleration, gyroscope data
    sim.plot(['pos', 'vel', 'att_euler', 'accel', 'gyro'],
             opt={'pos': 'error', 'vel': 'error', 'att_euler': 'error'})



if __name__ == '__main__':
    test_free_integration()  # Execute the free integration test

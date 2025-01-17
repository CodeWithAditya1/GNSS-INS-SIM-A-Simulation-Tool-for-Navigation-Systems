

import os
import math
import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 200.0          # IMU sample frequency

def test_free_integration():
    '''
    test Sim
    '''
    #### IMU model, typical for IMU381
    imu_err = {'gyro_b': np.array([0.0, 0.0, 0.0]),
               'gyro_arw': np.array([0.25, 0.25, 0.25]) * 0.0,
               'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 0.0,
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 0.0,
               'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 0.0,
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),
               'mag_std': np.array([0.2, 0.2, 0.2]) * 0.0
              }
    # do not generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)

    #### Algorithm
    # Free integration in a virtual inertial frame
    from demo_algorithms import free_integration
    '''
    Free integration requires initial states (position, velocity and attitude).
    You should provide theses values when you create the algorithm object.
    '''
    ini_pos_vel_att = np.genfromtxt(motion_def_path+"//motion_def-long_drive.csv",\
                                    delimiter=',', skip_header=1, max_rows=1)
    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R
    # add initial states error if needed
    ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] += ini_att_err * D2R
    # create the algorith object
    algo = free_integration.FreeIntegration(ini_pos_vel_att)

    #### start simulation
    sim = ins_sim.Sim([fs, 0.0, 0.0],
                      motion_def_path+"//motion_def-long_drive.csv",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    # run the simulation once
    sim.run(1)
    # generate simulation results, summary
    # do not save data, generate .kml file
    sim.results('', err_stats_start=-1, gen_kml=True)

if __name__ == '__main__':
    test_free_integration()

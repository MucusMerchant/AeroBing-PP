#from ecl import Ekf, Vector3f
#%%
import ecl
import numpy as np
#import random


class EkfWrapper:
    def __init__(self):
        self.ekf = ecl.Ekf()
        self.params = self.ekf.getParamHandle()

    def begin(self, last_time = 0):
        self.ekf.init(last_time)
        self.last_time = last_time
        self.params.fusion_mode = 1
        self.params.vdist_sensor_type = 0
        self.params.gps_check_mask = 0
        self.params.terrain_fusion_mode = 0
        self.params.mag_fusion_type = 2
        self.gps_delay_ms = 95
        self.params.sensor_interval_min_ms = 4
        self.params.accel_noise = 3.38445e-2
        self.params.gyro_noise = 3.92699e-3
        #self.params.baro_noise = 0.5
        #self.params.mag_heading_noise = 1
        self.params.mag_declination_deg = -12.93
        #self.params.pos_noaid_noise = 100
        #self.params.mag_declination_source = 0
        #self.params.gyro_bias_p_noise = .01
        #self.params.accel_bias_p_noise = 0.8
        #self.params.vel_Tau = 0.01 # according to manual, lower values decrease error but increase noise
        #self.params.pos_Tau = 10
        #self.params.acc_bias_lim = 0.9
        #self.params.acc_bias_learn_tc = 0.00001
        #self.params.baro_innov_gate = 10.0
        #self.params.gps_pos_innov_gate = 10.0
        #self.params.gps_vel_innov_gate = 10.0

        self.params.imu_pos_body = ecl.Vector3f(np.array([0,0,0], dtype=np.float32)[:,np.newaxis])
        self.params.gps_pos_body = ecl.Vector3f(np.array([0,0,0], dtype=np.float32)[:,np.newaxis])
        # other stuff here

    def setIMU(self, timestamp, ang, vel):
        dt = (timestamp - self.last_time) / 1e6
        #print(dt)
        self.last_time = timestamp
        imudata = ecl.imuSample()
        imudata.time_us = int(timestamp)
        imudata.delta_ang = ecl.Vector3f(ang * dt)
        imudata.delta_vel = ecl.Vector3f(vel * dt)
        #print(np.array(imudata.delta_vel).dtype)
        #imudata.delta_ang = ecl.Vector3f(np.array([[0,0,0]], dtype=np.float32).transpose() *dt)
        #imudata.delta_vel = ecl.Vector3f(np.array([[0,0,9.81]], dtype=np.float32).transpose() *dt)
        imudata.delta_ang_dt = dt
        imudata.delta_vel_dt = dt
        self.ekf.setIMUData(imudata)

    def setMag(self, timestamp, mag):
        magdata = ecl.magSample()
        magdata.time_us = int(timestamp)
        magdata.mag = ecl.Vector3f(mag)
        self.ekf.setMagData(magdata)

    def setBaro(self, timestamp, pres):
        barodata = ecl.baroSample()
        barodata.time_us = int(timestamp)
        barodata.hgt = np.float32(44330.0 * (1.0 - pow((pres / 100 / 1013.25), 0.1903)))
        #print(barodata.hgt)
        self.ekf.setBaroData(barodata)

    def setGPS(self, timestamp, lon, lat, alt, vel_n, vel_e, vel_d, eph, epv, sacc, vel, pdop, nsats, fix_type, _, flags):
        gpsdata = ecl.gps_message()
        gpsdata.time_usec = int(timestamp)
        gpsdata.lon = int(lon)
        gpsdata.lat = int(lat)
        gpsdata.alt = int(alt)
        gpsdata.yaw = float('nan')
        gpsdata.yaw_offset = 0.0
        gpsdata.eph = eph
        gpsdata.epv = epv
        gpsdata.sacc = sacc
        gpsdata.vel_m_s = vel
        gpsdata.vel_ned = ecl.Vector3f(np.array([vel_n, vel_e, vel_d], dtype=np.float32)[:,np.newaxis])
        gpsdata.vel_ned_valid = flags & 0x01 # this is from the datasheet, gnssFixOK
        gpsdata.nsats = nsats
        gpsdata.fix_type = fix_type
        gpsdata.pdop = pdop
        self.ekf.setGpsData(gpsdata)

    def getPosition(self):
        return self.ekf.getPosition()
    
    def getVelocity(self):
        return self.ekf.getVelocity()
    
    def getQuaternion(self):
        return self.ekf.getQuaternion()

    def update(self):
        self.ekf.update()
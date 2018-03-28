#!/usr/bin/env python
import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import rospy
from sensor_msgs.msg import Imu


def Read_IMU():

    # Set the name of the settings file:
    SETTINGS_FILE = "RTIMULib_Mod"

    # Initialize a ROS node:
    rospy.init_node("IMU_reader", anonymous = True)

    # Create a publisher:
    pubIMU = rospy.Publisher("/rover/imu/data", Imu, queue_size = 10)


    # Indicate which settings file is being used:
    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
      print("Settings file does not exist, will be created")

    # Import the settings here:
    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)
    pressure = RTIMU.RTPressure(s)

    # Print some info
    print("IMU Name: " + imu.IMUName())
    print("Pressure Name: " + pressure.pressureName())


    # Check if IMU is initialized:
    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded");


    # Set data fusion parameters:
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    if (not pressure.pressureInit()):
        print("Pressure sensor Init Failed")
    else:
        print("Pressure sensor Init Succeeded")

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)


    rate = rospy.Rate(50) # 50hz


    # Loop through and read IMU data and publish to the IMU topic:
    while not rospy.is_shutdown():

      # If data was successfully read:
      if imu.IMURead():

        data = imu.getIMUData()
        #fusionPose = data["fusionPose"]

        # Extract the raw gyroscope and accelerometer data:
        gyroData = data["gyro"]
        accelData = data["accel"]

        imuData = Imu()

        imuData.angular_velocity.x = gyroData[0]
        imuData.angular_velocity.y = gyroData[1]
        imuData.angular_velocity.z = gyroData[2]


        imuData.linear_acceleration.x = accelData[0]
        imuData.linear_acceleration.y = accelData[1]
        imuData.linear_acceleration.z = accelData[2]


        pubIMU.publish(imuData)


        rate.sleep()


    time.sleep(poll_interval*1.0/1000.0)

# Run the main function:
if __name__ == '__main__':
    Read_IMU()


# -------------- FOR LATER -------------#

# #  computeHeight() - the conversion uses the formula:
# #
# #  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
# #
# #  where:
# #  h  = height above sea level
# #  T0 = standard temperature at sea level = 288.15
# #  L0 = standard temperatur elapse rate = -0.0065
# #  p  = measured pressure
# #  P0 = static pressure = 1013.25
# #  g0 = gravitational acceleration = 9.80665
# #  M  = mloecular mass of earth's air = 0.0289644
# #  R* = universal gas constant = 8.31432
# #
# #  Given the constants, this works out to:
# #
# #  h = 44330.8 * (1 - (p / P0)**0.190263)
#
# def computeHeight(pressure):
#     return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));

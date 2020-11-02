# Total Capture MoCap Trajectory
import numpy as np

from imusim.platforms.imus import Orient3IMU
from imusim.platforms.imus import IdealIMU
from imusim.environment.base import Environment
from imusim.simulation.calibrators import ScaleAndOffsetCalibrator
from imusim.trajectories.rigid_body import SplinedBodyModel
from imusim.simulation.base import Simulation
from imusim.behaviours.imu import BasicIMUBehaviour

from imusim.testing.random_data import RandomTrajectory

from imusim.io.bvh import CM_TO_M_CONVERSION
from imusim.io.bvh import loadBVHFile

from imusim.visualisation.plotting import plot
# model = loadBVHFile("/home/lisca/data/mocap/01_01.bvh", CM_TO_M_CONVERSION)
model = loadBVHFile("/home/lisca/data/total/s1/acting1_BlenderZXY_YmZ.bvh", CM_TO_M_CONVERSION)
splinedModel = SplinedBodyModel(model)
env = Environment()
# imu = Orient3IMU()
imu = IdealIMU()
samples = 1000
rotationalVelocity = 20
samplingPeriod = 0.01
calibrator = ScaleAndOffsetCalibrator(env, samples, samplingPeriod, rotationalVelocity)
calibration = calibrator.calibrate(imu)
sim = Simulation(environment=env)
# r_ft_trjctr = splinedModel.getJoint('rFoot')
# r_ft_trjctr.name
splinedModel.jointNames
imu.simulation = sim
imu.trajectory = splinedModel.getJoint('LeftHandEnd')
sim.time = splinedModel.startTime
behaviour = BasicIMUBehaviour(imu, samplingPeriod, calibration, initialTime=sim.time)
# sim.run(splinedModel.endTime)
sim.run(splinedModel.startTime + 2.5)
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

frms_cnt = 50
dt = 1 / 60.0
tm_sqnc = np.arange(0, frms_cnt*dt, dt)

# x = model.positionKeyFrames.values[0][:trim]
# y = model.positionKeyFrames.values[1][:trim]
# z = model.positionKeyFrames.values[2][:trim]

# l_hnd_trjctry = model.getJoint('LeftHand')

l_hnd_trjctry = splinedModel.getJoint('LeftHand')
r_hnd_trjctry = splinedModel.getJoint('RightHand')

l_hnd_trjctry_pts = np.empty([3, tm_sqnc.shape[0]])
r_hnd_trjctry_pts = np.empty([3, tm_sqnc.shape[0]])

for idx, tm_stmp in enumerate(tm_sqnc, 0):
    l_hnd_trjctry_pts[:, idx] = l_hnd_trjctry.position(tm_stmp).reshape(3)
    r_hnd_trjctry_pts[:, idx] = r_hnd_trjctry.position(tm_stmp).reshape(3)

x = l_hnd_trjctry_pts[0][:]
y = l_hnd_trjctry_pts[1][:]
z = l_hnd_trjctry_pts[2][:]
    
ax.scatter(x, y, z, marker="*")

x = r_hnd_trjctry_pts[0][:]
y = r_hnd_trjctry_pts[1][:]
z = r_hnd_trjctry_pts[2][:]
    
ax.scatter(x, y, z, marker="*")

plt.show()
l_hnd_trjctry
l_hnd_trjctry.position(0.1)
# figure()
plot(imu.accelerometer.calibratedMeasurements)
# plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values)
# plot(imu.gyroscope.calibratedMeasurements.timestamps, imu.gyroscope.calibratedMeasurements.values)
# title("Accelerometer Readings")
# xlabel("Time (s)")
# ylabel("Acceleration (m/s^2)")
# legend()
# figure()
plot(imu.gyroscope.calibratedMeasurements)
# plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values)
# plot(imu.gyroscope.calibratedMeasurements.timestamps, imu.gyroscope.calibratedMeasurements.values)
# title("Accelerometer Readings")
# xlabel("Time (s)")
# ylabel("Acceleration (m/s^2)")
# legend()

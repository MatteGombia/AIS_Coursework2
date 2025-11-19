#!/usr/bin/env python3

from frame2d import Frame2D
import math

import numpy as np


# a few different values for odometry noise. Try the last
# set first.

#cozmoOdomNoiseX = 0.001
#cozmoOdomNoiseY = 0.001
#cozmoOdomNoiseTheta = 0.0001

# norm values. high values *=10
#cozmoOdomNoiseX = 0.5
#cozmoOdomNoiseY = 0.5
#cozmoOdomNoiseTheta = 0.00003

cozmoOdomNoiseX = 0.2
cozmoOdomNoiseY = 0.2
cozmoOdomNoiseTheta = 0.001

#you may need to find a sensible wheel distance parameter (determine experimentally)
# this is a reasonable starting point
wheelDistance=80

# Forward kinematics: compute coordinate frame update as Frame2D from left/right track speed and time of movement
def track_speed_to_pose_change(left, right, time):
    # for forward facing x-axis, left facing y-axis
    l = left*time
    r = right*time
    a = (r-l)/wheelDistance
    if abs(a) > 0.0001:
        rad = (l+r)/(2*a)
        return Frame2D.fromXYA(rad*math.sin(a),-rad*(math.cos(a)-1),a)
    else:
        return Frame2D.fromXYA((l+r)/2,0,a)    # TODO
    return Frame2D()

# Differential inverse kinematics: compute left/right track speed from desired angular and forward velocity
def velocity_to_track_speed(forward, angular):
    # (l+r)/2 = forward
    #a l+r = 2*forward
    # (l-r)/wheelDistance = angular
    #b l-r = angular*wheelDistance
    #a+b  2*l = 2*forward + angular*wheelDistance
    l = forward - angular*wheelDistance/2
    #a-b  2*r = 2*forward - angular*wheelDistance
    r = forward + angular*wheelDistance/2
    return [l,r]   

# Trajectory planning: given target (ralative to robot frame), determine next forward/angular motion 
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(relativeTarget: Frame2D):
		"""Simple proportional controller to drive towards a target given in the robot frame.

		Inputs:
			relativeTarget: Frame2D giving target pose in robot coordinates (x forward, y left).

		Returns: [forward_mm_s, angular_rad_s]

		Behaviour:
			- If the angular heading to the target is large, rotate in place (forward=0).
			- Otherwise drive forward with a velocity proportional to distance and a small angular correction.
			- If very close to the target, rotate to match target orientation (use relativeTarget.angle()).
		"""
		# gains and limits (tune as needed)
		max_forward = 120.0   # mm/s
		max_angular = 1.5     # rad/s
		k_dist = 0.8          # forward gain per mm
		k_ang = 2.0           # angular gain
		angle_threshold = 0.35  # rad (~20deg)
		close_dist = 30.0     # mm

		xya = relativeTarget.toXYA()
		tx = xya[0]
		ty = xya[1]
		# heading to target in robot frame
		heading = math.atan2(ty, tx)
		distance = math.hypot(tx, ty)

		# If very close to the point, try to orient to the target's angle
		if distance < close_dist:
				# desired orientation is the frame angle
				ang_err = xya[2]
				forward = 0.0
				angular = max(-max_angular, min(max_angular, k_ang * ang_err))
				return [forward, angular]

		# If heading is large, rotate in place first
		if abs(heading) > angle_threshold:
				forward = 0.0
				angular = max(-max_angular, min(max_angular, k_ang * heading))
				return [forward, angular]

		# otherwise drive forward with a small angular correction
		forward = max(-max_forward, min(max_forward, k_dist * distance))
		angular = max(-max_angular, min(max_angular, k_ang * heading))
		return [forward, angular]

# Trajectory planning: given target (ralative to robot frame), determine next forward/angular motion 
# Implement by means of cubic spline interpolation 
def target_pose_to_velocity_spline(relativeTarget: Frame2D):
	"""A slightly smoother controller using a cubic-like scaling for forward speed.

	This is still a simple local controller: it reduces forward speed when close and
	keeps angular corrections proportional to heading.
	Returns [forward_mm_s, angular_rad_s].
	"""
	max_forward = 120.0
	max_angular = 1.5
	k_dist = 0.01
	k_ang = 2.0

	xya = relativeTarget.toXYA()
	tx = xya[0]
	ty = xya[1]
	heading = math.atan2(ty, tx)
	distance = math.hypot(tx, ty)

	# cubic-like smooth scaling (small distances -> much smaller speed)
	forward = k_dist * (distance**1.5)
	forward = max(-max_forward, min(max_forward, forward))
	angular = max(-max_angular, min(max_angular, k_ang * heading))
	return [forward, angular]

# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific measure position (relative to robot frame)
def cube_sensor_model(trueCubePosition, visible, measuredPosition):
	# check visibility angle
	angle = math.atan2(trueCubePosition.y(),trueCubePosition.x())
	midAngle = 23.0/180.0*math.pi
	relativeAngle = abs(angle)/midAngle
	relativeTolerance = 0.2
	angleVisibilityProb = 1
	if 1+relativeTolerance < relativeAngle:
		##angleVisibilityProb = 0.2 ## slide examples value
		angleVisibilityProb = 0.2
	elif 1-relativeTolerance < relativeAngle:
		angleVisibilityProb = 0.5
	else:
		#angleVisibilityProb = 0.9 ## slide examples value
		angleVisibilityProb = 0.8
	#angleVisibilityProb = 1

	# check distance angle - all sorts of parameters to experiment with here
	distance = math.sqrt(trueCubePosition.y()*trueCubePosition.y()+trueCubePosition.x()*trueCubePosition.x())
	minDistance = 100
	minTolerance = 50
	maxDistance = 450
	maxTolerance = 100
	distanceProb = 1
	minProb = 0.2
	maxProb = 0.8
	if distance < minDistance - minTolerance:
		distanceProb = minProb
	elif distance < minDistance + minTolerance:
		distanceProb = minProb + (maxProb-minProb) * (distance-(minDistance-minTolerance))/(2*minTolerance)
	elif distance < maxDistance - maxTolerance:
		distanceProb = maxProb
	elif distance < maxDistance + maxTolerance:
		distanceProb = maxProb - (maxProb-minProb) * (distance-(maxDistance-maxTolerance))/(2*maxTolerance)
	else:
		distanceProb = minProb
	#distanceProb = 1

	# location probability
	positionProb = 1
        # you could experiment with different values for the sigmas associated with location
	sigma = np.array([20.0,20.0,10.0/180*math.pi])
	sigSquareInv = 1.0 / (sigma*sigma)
	if visible:
		#deviation = trueCubePosition.mult(measuredPosition.inverse())
		deviation = measuredPosition.inverse().mult(trueCubePosition)
		xya = deviation.toXYA()
		error = 0.5 * np.sum(xya * sigSquareInv * xya)
		positionProb = math.exp(-error)
		
	if visible:
		return positionProb # note: even smallest misjudgements in the visibility amplify very quickly, focus on position!
                # try this if you get good results
#		return angleVisibilityProb*distanceProb * positionProb
	else:
		return 1
#		return 1-angleVisibilityProb*distanceProb    return 1.0


import numpy as np
import rpi_abb_irc5
import time
import general_robotics_toolbox as rox
import collections
import scipy.io
import matplotlib.pyplot as plt
import QuadProg as qp
import sys
#import rpi_ati_net_ft
from scipy.optimize import minimize_scalar
from scipy import interpolate

def abb_irb6700_robot():
    """Return a Robot instance for the ABB IRB6700 robot"""
    
    x = np.array([1,0,0])
    y = np.array([0,1,0])
    z = np.array([0,0,1])
    a = np.array([0,0,0])
    
    H = np.array([z,y,y,x,y,x]).T
    P = np.array([780*z, 320*x, 1135*z, 200*z, 1182.5*x, 200*x, a]).T
    joint_type = [0,0,0,0,0,0]
    joint_min = np.deg2rad(np.array([-170, -65, -180, -300, -130, -360]))
    joint_max = np.deg2rad(np.array([170, 85, 70, 300, 130, 360]))
    
    p_tool = np.array([0,0,0])
    R_tool = rox.rot([0,1,0], np.pi/2.0)
    
    return rox.Robot(H, P, joint_type, joint_min, joint_max, R_tool=R_tool, p_tool=p_tool)
	
## does not track motion trajectory until reach the Fd
def first_half(input, num_iter):

    # stop the active RAPID program
    rapid.stop()
    # reset the pointer to main
    rapid.resetpp()
    print 'first half'
    print 'reset PP to main'
    time.sleep(2) 
    # start the RAPID program
    rapid.start()
    
	# determine if robot has reached the initial configuration
    tag = True
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res: 
            #print np.fabs(sum(np.rad2deg(state.joint_angles)) - sum(init))
            if np.fabs(sum(state.joint_angles) - sum(init)) < 1e-4:
                tag = False
	
    time.sleep(1)	
    
    # out is composed of 5 velocity and 1 force in z
    out = np.zeros((6, n))
    force_out = np.zeros((1, n))
    # pos of eef
    eef_pos = np.zeros((3, n))	
    # orientation of eef (quaternion)
    eef_orien = np.zeros((4, n))
    # timestamp
    tim = np.zeros((1, n))

    q_hat = np.zeros((6, 1))
    qhat_dot = np.zeros((6, 1))
    # for observer k should be symmetric and positive definite
    kl = 0.5

    ### drain the EGM buffer ###
    for i in range(1000):
        res, state = egm.receive_from_robot(.1)
    
    time.sleep(2) 

    cnt = 0
    step_done = False
    while cnt < n:
        # receive EGM feedback
        res, state = egm.receive_from_robot(.1)
		
        if not res:
            continue
			
        q_new = np.deg2rad(state.joint_angles)
        
        # step-over
        if not step_done:
            print '--------start step-over motion--------'
            # do step-over of 0.25 mm in +x in world frame
            # current eef pose

            pose_cur = rox.fwdkin(abb_robot, q_new)
            pose_cur.p[0] = pose_cur.p[0] + num_iter*2*step_over
            # solve for inverse kinematics and pick the one that is closest to current configuration
            sol = rox.robot6_sphericalwrist_invkin(abb_robot, pose_cur, q_new)
            try:
                tar = sol[0] # raise exception if no solution
            except:
                tar = q_new
            # move to the new position after step-over
            egm.send_to_robot(tar)
            step_done = True

            q_new = tar
            ### drain the EGM buffer, or it will use the obsolete EGM feedback###
            for i in range(1000):
                res, state = egm.receive_from_robot(.1)
          
            print '--------step-over motion done--------'
            time.sleep(2)


	    # forward kinematics to calculate current position of eef
        pose = rox.fwdkin(abb_robot, q_new)
        R = pose.R
        Fd0 = 50
        if pose.p[2] >= pos_obj:
            F = 0          
            v_z = Kp*30*(F-Fd0)*1000#-Ki*(z-z_ref)-Kd*acce#-Ki*pos
            # formalize entire twist
            spatial_velocity_command = np.array([0, 0, 0, 0, 0, v_z])      			
        else:
            F = -k*(pose.p[2] - pos_obj)
            if F < Fdz-0.5 and cnt == 0:
                v_z = Kp*(F-Fdz)*1000#-Ki*(z-z_ref)-Kd*acce#-Ki*pos
                # formalize entire twist
                spatial_velocity_command = np.array([0, 0, 0, 0, 0, v_z])            
            else:
				# formalize entire twist
				# nominal input composed of F and v
				spatial_velocity_command = input[:, cnt]

				v_z = Kp*(F-spatial_velocity_command[5])*1000
				# nominal input only contains v
				spatial_velocity_command[5] = v_z
				# approximation of joint velocity
				#q_new = np.deg2rad(state.joint_angles)
				
				######### change here, use observer instead of approximation to calculate q_dot ########
				if cnt == 0:
					q_hat = q_new
				qhat_dot = joints_vel + kl*(q_new-q_hat)
				
				#q_dot = (q_new - q_pre)/delta_t
				
				J = rox.robotjacobian(abb_robot, q_new)
				# estimate velocity
				v_est = J.dot(qhat_dot)
				# formalize the nominal output composed of F and v
				out[:, cnt] = np.append(v_est[0:5], F)
							
				force_out[:, cnt] = F

				eef_pos[:, cnt] = pose.p
			
				R = pose.R
				quat = rox.R2q(R)
				eef_orien[:, cnt] = quat
				tim[0, cnt] = time.time()
				cnt = cnt+1

        print F
        
        # solve for joint velocity
        # Jacobian inverse
        J = rox.robotjacobian(abb_robot, q_new)		
        joints_vel = np.linalg.pinv(J).dot(spatial_velocity_command)
        
        # QP
        #joints_vel = quadprog.compute_joint_vel_cmd_qp(q_new, spatial_velocity_command)
        
        # commanded joint position setpoint to EGM
        q_c = q_new + joints_vel*delta_t
        egm.send_to_robot(q_c)
        # joint angle at previous time step
        
        ######### change here ########
        q_hat = q_hat + qhat_dot*delta_t
			
    error = out - desired
    # flip the error
    err_flip = np.fliplr(error)
    print np.linalg.norm(error, 'fro')
	
    return out, err_flip, np.linalg.norm(error, 'fro'), force_out, eef_pos, eef_orien, tim
    
def second_half(x, out_pre, num_iter):

    rapid.stop()
    rapid.resetpp()
    time.sleep(2) 
    print 'restart'
    rapid.start()
    print 'second half'
    time.sleep(2) 
    # determine if robot has reached the initial configuration init
    tag = True
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res: 
            #print np.fabs(sum(np.rad2deg(state.joint_angles)) - sum(init))
            if np.fabs(sum(state.joint_angles) - sum(init)) < 1e-4:
                tag = False

    time.sleep(1)	
    
    out = np.zeros((6, n))

    q_hat = np.zeros((6, 1))
    qhat_dot = np.zeros((6, 1))
    # for observer k should be symmetric and positive definite
    kl = 0.5
	
    ### drain the EGM buffer ###
    for i in range(1000):
        res, state = egm.receive_from_robot(.1)

    time.sleep(2)

    cnt = 0
    step_done = False
    while cnt < n:
        # receive EGM feedback
        res, state = egm.receive_from_robot(.1)
        
        if not res:
            continue
       
        q_new = np.deg2rad(state.joint_angles)
		
        # step-over
        if not step_done:
            print '--------start step-over motion--------'
            # do step-over of 0.25 mm in +x in world frame
            # current eef pose
            pose_cur = rox.fwdkin(abb_robot, q_new)
            pose_cur.p[0] = pose_cur.p[0] + (num_iter*2+1)*step_over
            # solve for inverse kinematics and pick the one that is closest to current configuration
            sol = rox.robot6_sphericalwrist_invkin(abb_robot, pose_cur, q_new)
            try:
                tar = sol[0] # raise exception if no solution
            except:
                tar = q_new
            # move to the new position after step-over
            egm.send_to_robot(tar)
            step_done = True

            q_new = tar
            ### drain the EGM buffer, or it will use the obsolete EGM feedback###
            for i in range(1000):
                res, state = egm.receive_from_robot(.1)

            print '--------step-over motion done--------'
            time.sleep(2)

        # forward kinematics to calculate current position of eef
        pose = rox.fwdkin(abb_robot, q_new)
        R = pose.R
        Fd0 = 50		
        if pose.p[2] >= pos_obj:
            F = 0          
            v_z = Kp*30*(F-Fd0)*1000#-Ki*(z-z_ref)-Kd*acce#-Ki*pos
            # formalize entire twist
            spatial_velocity_command = np.array([0, 0, 0, 0, 0, v_z])      			
        else:
            F = -k*(pose.p[2] - pos_obj)
            if F < Fdz-0.5 and cnt == 0:
                v_z = Kp*(F-Fdz)*1000#-Ki*(z-z_ref)-Kd*acce#-Ki*pos
                # formalize entire twist
                spatial_velocity_command = np.array([0, 0, 0, 0, 0, v_z])            
            else:
				# formalize entire twist
				# nominal input composed of F and v
				spatial_velocity_command = x[:, cnt]
				
				v_z = Kp*(F-spatial_velocity_command[5])*1000
				# nominal input only contains v
				spatial_velocity_command[5] = v_z
				# approximation of joint velocity
				#q_new = np.deg2rad(state.joint_angles)
				
				######### change here, use observer instead of approximation to calculate q_dot ########
				if cnt == 0:
					q_hat = q_new
				qhat_dot = joints_vel + kl*(q_new-q_hat)
								
				#q_dot = (q_new - q_pre)/delta_t
				
				J = rox.robotjacobian(abb_robot, q_new)
				# estimate velocity
				v_est = J.dot(qhat_dot)
				# formalize the nominal output composed of F and v
				out[:, cnt] = np.append(v_est[0:5], F) 
				cnt = cnt+1
		
        print F
        # solve for joint velocity
        # Jacobian inverse
        J = rox.robotjacobian(abb_robot, q_new)		
        joints_vel = np.linalg.pinv(J).dot(spatial_velocity_command)
	
        # QP
        #joints_vel = quadprog.compute_joint_vel_cmd_qp(q_new, spatial_velocity_command)
			
        # commanded joint position setpoint to EGM
        q_c = q_new + joints_vel*delta_t
		
        egm.send_to_robot(q_c)
        # joint angle at previous time step
        #q_pre = q_new
        #t_pre = t_new

        ######### change here ########
        q_hat = q_hat + qhat_dot*delta_t
		
    err = out-out_pre
    err_flip2 = np.fliplr(err)
    
    return err_flip2


# initialize EGM interface instance
egm = rpi_abb_irc5.EGM()

# initialize a robot instance
abb_robot = abb_irb6700_robot()

# desired force
Fdz = 1000

# desired velocity in y (mm/s)
vdy = -300

# spring constant N/mm (the real one is 2.4378e7)
k = 750

# position of object in z (mm)
pos_obj = 1.05*1000

# feedback gain
Kp = 0.0003
Kd = 0.0008
Ki = 0.0004

# time step
delta_t = 0.004

# initial configuration in degree
init = [-91.08,2.54,38.18,0.0,49.27,-1.07]

step_over = 0.25 # in mm

# quadprog to solve for joint velocity
quadprog = qp.QuadProg(abb_robot)

pos = 0
acce = 0
v_l_pre = 0

############ change here, how long the process lasts ############
n = 1000

####### trapezoidal desired force in z and trapezoidal motion in y #######
"""
tt = np.linspace(0, 4*np.pi, n)
Fdz = np.zeros((n, ))
vdy = np.zeros((n, ))
# form trap force and trap motion
for i in range(n):
    if tt[i] >= 0 and tt[i] < np.pi:
        Fdz[i] = 50+302*tt[i]
        vdy[i] = -0.2*tt[i]
    elif tt[i] >= np.pi and tt[i] < 3*np.pi:
        Fdz[i] = 50+302*np.pi
        vdy[i] = -0.2*np.pi
    else:
        Fdz[i] = 50+302*np.pi-302*(tt[i]-3*np.pi)
        vdy[i] = -0.2*np.pi + 0.2*(tt[i]-3*np.pi)
"""        
# desired output (composed of force and velocity)
desired = np.zeros((6, n))

# form the desired output by ud
for i in range(n):
    ud = np.array([0, 0, 0, 0, vdy, Fdz])
    desired[:, i] = ud
    
# referece height of coupon that achieves desired force
z_ref = 0.89226
x_ref = 0
y_ref = -1.35626
	
# RobotStudio network service
if (len(sys.argv) >= 2):
    rapid=rpi_abb_irc5.RAPID(sys.argv[2])
else:
    rapid=rpi_abb_irc5.RAPID()

# input to the nominal dynamical system that is subject to update by ILC
# at the beginning, it is just desired output
desired_cp = desired.copy()
x_in = desired_cp

fro_err_old = 0

# max number of ILC iterations
iter = 20
for i in range(iter):
    # first pass into the dynamical system
    x_in_cp = x_in.copy()
    out, err_flip1, fro_err, force_out, eef_pos, eef_orien, tim = first_half(x_in_cp, i)
    
    # save all data after each iteration
    csv_dat=np.hstack((desired.T, x_in.T, out.T, force_out.T, eef_pos.T, eef_orien.T, tim.T))
    np.savetxt('ILC_trap_force_trap_motion_control_with_' + str(i) + '_iteration.csv', csv_dat, fmt='%6.5f', delimiter=',')
	
    time.sleep(2)   
    
    x = x_in+err_flip1	
    x_cp = x.copy()
	
    # second pass into the dynamical system	
    errflip2 = second_half(x_cp, out, i)
    #time.sleep(2)  
    
    # use fixed learning rate
    res = 0.25
    x_in = x_in - res*errflip2
	
    fro_err_old = fro_err


from quadrotor import Quadrotor

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Vector3
from matplotlib import pyplot as plt
from polynomialTrjNonlinear.Optimizer_nonlinear import PolynomialOptNonlinear
from polynomialTrjNonlinear.vertex import Vertex
from basic_functions import *
import copy
import math
from gazebo_msgs.msg import ModelStates

class Mellinger(Quadrotor):
    """
    Mellinger Controller with Polynomial Trajectory Generation
    Using nonlinear optimizations.
    """

    def __init__(self, mav_name, index, x, y, z, dimension=3, N=10):
        Quadrotor.__init__(self, mav_name, index)

        self.k_T = 6.7
        self.k_phi_theta = 1.7
        self.k_psi = 1.7
        self.set_initial_pos(x, y, z)
        self.M_UtoMotor = np.array([
            [self.k_T,        0.0,         -self.k_phi_theta,    self.k_psi],
            [self.k_T,    self.k_phi_theta,        0.0,        -self.k_psi],
            [self.k_T,        0.0,          self.k_phi_theta,   self.k_psi],
            [self.k_T,   -self.k_phi_theta,       0.0,         -self.k_psi]
        ])

        # controller parameters for controlling a single robot
        # hzyu
        k_pxy = 50.0
        k_pz = 70.0
        k_pxy = 25.0
        k_pz = 35.0
        k_vxy = 60.7
        k_vz = 60.7
        k_omega = 18.5
        k_R = 85.5

	self.f1=mav_name+'pos'+ '_' + str(index)+'.txt'
	self.f2=mav_name+'vel'+ '_' + str(index)+'.txt'
	self.f3=mav_name+'despos'+ '_' + str(index)+'.txt'
	self.f4=mav_name+'desvel'+ '_' + str(index)+'.txt'
	self.f5=mav_name+'paypos'+ '_' + str(index)+'.txt'
	self.f6=mav_name+'payvel'+ '_' + str(index)+'.txt'
	self.f7=mav_name+'paydespos'+ '_' + str(index)+'.txt'
	self.f8=mav_name+'desT'+ '_' + str(index)+'.txt'
	self.f9=mav_name+'payangvel'+ '_' + str(index)+'.txt'
	self.f10=mav_name+'payorien'+ '_' + str(index)+'.txt'
	self.f11=mav_name+'input'+'_' + str(index)+'.txt'

        self.xi=x
        self.yi=y
        self.zi=z

        self.k_pI = 2.0
        self.K_p = np.eye(3)
        self.K_p[0, 0] = k_pxy
        self.K_p[1, 1] = k_pxy
        self.K_p[2, 2] = k_pz

        self.k_vI = 2.0
        self.K_v = np.eye(3)
        self.K_v[0, 0] = k_vxy
        self.K_v[1, 1] = k_vxy
        self.K_v[2, 2] = k_vz
        self.K_omega = k_omega
        self.K_R = k_R

        # Timer
        self.initial_time = None
        self.t = None
        self.hover_begin_time = None
        self.hover_duration = 0.0

        # Polynomial trajectory planner
        self.isPolynomialSolved = False
        self.NL_planner = PolynomialOptNonlinear(N=N, dimension=dimension)

        self.trj_xyz = np.zeros((3, 1))
        self.trj_v = np.zeros((3, 1))
        self.trj_acc = np.zeros((3, 1))
        self.trj_jerk = np.zeros((3, 1))
        self.trj_snap = np.zeros((3, 1))

        self.trj_x_list = []
        self.trj_y_list = []
        self.trj_z_list = []

        self.trj_x_list_payload = []
        self.trj_y_list_payload = []
        self.trj_z_list_payload = []

        self.trj_vx_list_payload = []
        self.trj_vy_list_payload = []
        self.trj_vz_list_payload = []

        self.trj_vx_list = []
        self.trj_vy_list = []
        self.trj_vz_list = []

        self.trj_accx_list = []
        self.trj_accy_list = []
        self.trj_accz_list = []

        self.trj_jerkx_list = []
        self.trj_jerky_list = []
        self.trj_jerkz_list = []

        self.trj_snapx_list = []
        self.trj_snapy_list = []
        self.trj_snapz_list = []

        self.abs_trj_pos = None
        self.rel_trj_pos = None
        # self.rel_trj_vel = None
        # self.rel_trj_acc = None
        # self.rel_trj_jerk = None
        # self.rel_trj_snap = None

        self.offset_added = False
	self.payload_inertia=np.array([[0.3342, 0, 0],[0, 0.3342, 0],[0, 0, 1.3333]])
	self.payload_mass=1.0

        #self.dt=0.01

    def cb_trajectory(self, data):
        print (data.segments[0].x)
        self.trj_poly_coeffs_x = data.segments[0].x
        self.trj_poly_coeffs_y = data.segments[0].y
        self.trj_poly_coeffs_z = data.segments[0].z
        self.trj_received = True

    def set_control_gains(self, k_pxy=14.0, k_pz=55.0, k_vxy=10.7, k_vz=10.7, k_omega=22.5, k_R=40.5):
        self.K_p[0, 0] = k_pxy
        self.K_p[1, 1] = k_pxy
        self.K_p[2, 2] = k_pz

        self.K_v[0, 0] = k_vxy
        self.K_v[1, 1] = k_vxy
        self.K_v[2, 2] = k_vz

        self.K_omega = k_omega
        self.K_R = k_R


    # updates
    def update_offset(self):
        self.NL_planner.init_offset(self.positions_quads.T)
        self.offset_added = True

    def update_offset_xyz(self):
        rel_trj_pos = copy.deepcopy(self.rel_trj_pos)
        self.abs_trj_pos = rel_trj_pos + np.array([self.xi, self.yi, self.zi])
        self.offset_added = True

    def update_omega_err(self):
        self.e_omegas = self.angular_vel_quads - np.dot(
            np.dot(self.R_.T, self.desired_R_), self.desired_omegas)
        # self.e_omegas = self.angular_vel_quads - self.desired_omegas
        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.e_omegas[0, 0], self.e_omegas[1, 0], self.e_omegas[2, 0])
        self.publisher_err_omegas.publish(pub_vec3)

    def update_R_err(self):
        e_R = np.dot(self.desired_R_.transpose(), self.R_) - \
                             np.dot(self.R_.transpose(), self.desired_R_)
        self.e_R_ = vee(e_R) / 2.0

        vec3_euler_des = Vector3Stamped()
        vec3_euler_des.header.stamp = rospy.Time.now()

        euler_des = rotationMatrixToEulerAngles(self.desired_R_)
        vec3_euler_des.vector = Vector3(euler_des[0], euler_des[1], euler_des[2])
        self.publisher_euler_des.publish(vec3_euler_des)

        vec3_e_R = Vector3Stamped()
        vec3_e_R.header.stamp = rospy.Time.now()
        vec3_e_R.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        self.publisher_err_R.publish(vec3_e_R)

        pub_vec3 = Vector3Stamped()
        pub_vec3.header.stamp = rospy.Time.now()
        pub_vec3.vector = Vector3(self.euler_quads[0, 0], self.euler_quads[1, 0], self.euler_quads[2, 0])
        self.publisher_eulers.publish(pub_vec3)

        # pub_vec3 = Vector3Stamped()
        # pub_vec3.header.stamp = rospy.Time.now()
        # pub_vec3.vector = Vector3(self.e_R_[0, 0], self.e_R_[1, 0], self.e_R_[2, 0])
        # self.publisher_err_angles.publish(pub_vec3)

    def set_hover_des(self, target_height):
        if not self.isHovering:
            if self.positions_quads[2, 0] > self.inital_position[2, 0] + target_height:
                self.isHovering = True
                self.hover_begin_time = rospy.Time.now()
            else:
                self.desired_positions[0, 0] = self.inital_position[0, 0]
                self.desired_positions[1, 0] = self.inital_position[1, 0]

                if self.desired_velocities[2, 0] < 0.5:
                    self.desired_velocities[2, 0] = self.desired_velocities[2, 0] + 0.01
                else:
                    self.desired_velocities = np.array([[0.0], [0.0], [0.5]])
                self.desired_positions[2, 0] = self.desired_positions[2, 0] + self.desired_velocities[2, 0] * self.dt
        else:
            self.hover_duration = rospy.Time.now().to_sec() - self.hover_begin_time.to_sec()
            #print self.name, 'hovering duration: ', self.hover_duration
            if self.desired_velocities[2, 0] > 0.0:
                self.desired_velocities[2, 0] = self.desired_velocities[2, 0] - 0.01
                self.desired_positions[2, 0] = self.desired_positions[2, 0] + self.desired_velocities[2, 0] * self.dt
            else:
                self.desired_velocities[2, 0] = 0.0
        self.publish_desired_trj()

    def construct_vertices(self, re_goal_position):
        vertices = []
        vertex0 = Vertex(dimension=3, index=0)
        vertex0.makeStartOrEnd(position=self.positions_quads, up_to_order=4)

        vertex1 = Vertex(dimension=3, index=1)
        vertex1.makeStartOrEnd(position=re_goal_position, up_to_order=4)

        vertices.append(vertex0)
        vertices.append(vertex1)
        self.NL_planner.setupFromVertices(vertices=vertices)

        self.NL_planner.add_max_vel(2.0)
        self.NL_planner.add_max_acc(1.0)

        self.optimize()

        self.getPlanUpToSnap(frequency=self.frequency)
        self.getPlanUpToSnap(frequency=100.0)


    def optimize(self):
        result = self.NL_planner.optimizeTimeAndFreeConstraints()
        self.isPolynomialSolved = True

    def getPlanUpToSnap(self, frequency):
        self.NL_planner.linear_opt.get_d_trajectory(order=0, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=1, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=2, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=3, sample_frequency=frequency)
        self.NL_planner.linear_opt.get_d_trajectory(order=4, sample_frequency=frequency)

        self.rel_trj_pos = copy.deepcopy(self.NL_planner.linear_opt.poly_pos)
        # self.rel_trj_vel = copy.deepcopy(self.NL_planner.linear_opt.poly_velocity)
        # self.rel_trj_acc = copy.deepcopy(self.NL_planner.linear_opt.poly_acc)
        # self.rel_trj_jerk = copy.deepcopy(self.NL_planner.linear_opt.poly_jerk)
        # self.rel_trj_snap = copy.deepcopy(self.NL_planner.linear_opt.poly_snap)

    def update_current_state(self):
        self.x_B = self.R_[:, 0].reshape((1, 3))
        self.y_B = self.R_[:, 1].reshape((1, 3))
        self.z_B = self.R_[:, 2].reshape((1, 3))

        # actual values
        # acc_vec = Vector3Stamped()
        # acc_vec.header.stamp = rospy.Time.now()
        # acc_vec.vector = Vector3(self.acc[0, 0], self.acc[1, 0], self.acc[2, 0])
        # self.pub_actual_acc.publish(acc_vec)


    def update_desired_inputs(self):
        self.update_desired_F()
        self.update_desired_values()
        self.update_desired_M()

   # def findloadvel(self):
#	rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback1)

  #  def callback1(self,data):
#	l=data.twist
#	self.linearvel=l[1].angular
#	self.angularvel=l[1].linear

    def getdestrj(self):
	pos_value=[[0.0],[0.0],[2.3]]
	vel_value=[[0.0],[0.0],[0.0]]
	acc_value=[[0.0],[0.0],[0.0]]
	jer_value=[[0.0],[0.0],[0.0]]
	sna_value=[[0.0],[0.0],[0.0]]
	for t in range(0,501,1):
            pos_value = np.concatenate(
                (pos_value, np.array([[2*t/500],[0.0],[2.0]]))
            )
	    vel_value = np.concatenate(
                (vel_value, np.array([[2/500],[0.0],[0.0]]))
            )
	    acc_value = np.concatenate(
                (acc_value, np.array([[0.0],[0.0],[0.0]]))
            )
	    jer_value = np.concatenate(
                (jer_value, np.array([[0.0],[0.0],[0.0]]))
            )
	    sna_value = np.concatenate(
                (sna_value, np.array([[0.0],[0.0],[0.0]]))
            )
	for t in range(0,4001,1):
            pos_value = np.concatenate(
                (pos_value, np.array([[2*math.cos(math.pi*t/1000)],[2*math.sin(math.pi*t/1000)],[2.0]]))
            )
	    vel_value = np.concatenate(
                (vel_value, np.array([[-0.628*math.sin(math.pi*t/1000)],[0.628*math.cos(math.pi*t/1000)],[0.0]]))
            )
	    acc_value = np.concatenate(
                (acc_value, np.array([[-0.197*math.cos(math.pi*t/1000)],[-0.197*math.sin(math.pi*t/1000)],[0.0]]))
            )
	    jer_value = np.concatenate(
                (jer_value, np.array([[0.062*math.sin(math.pi*t/1000)],[-0.062*math.cos(math.pi*t/1000)],[0.0]]))
            )
	    sna_value = np.concatenate(
                (sna_value, np.array([[0.019*math.cos(math.pi*t/1000)],[0.019*math.sin(math.pi*t/1000)],[0.0]]))
            )
        pos_value = pos_value.reshape((4503, 3))
	vel_value = vel_value.reshape((4503, 3))
	acc_value = acc_value.reshape((4503, 3))
	jer_value = jer_value.reshape((4503, 3))
	sna_value = sna_value.reshape((4503, 3))
	self.rel_trj_pos = copy.deepcopy(pos_value)
	self.rel_trj_vel = copy.deepcopy(vel_value)
	self.rel_trj_acc = copy.deepcopy(acc_value)
	self.rel_trj_jer = copy.deepcopy(jer_value)
	self.rel_trj_sna = copy.deepcopy(sna_value)

    def getdestrj2(self):
	pos_value=[[0.0],[0.0],[2.3]]
	vel_value=[[0.0],[0.0],[0.0]]
	acc_value=[[0.0],[0.0],[0.0]]
	jer_value=[[0.0],[0.0],[0.0]]
	sna_value=[[0.0],[0.0],[0.0]]
	for t in range(0,10001,1):
            pos_value = np.concatenate(
                (pos_value, np.array([[5*math.sin(math.pi*t/5000)],[4*math.sin(math.pi*t/2500)],[2.0]]))
            )
	    vel_value = np.concatenate(
                (vel_value, np.array([[0.314*math.cos(math.pi*t/5000)],[0.5024*math.cos(math.pi*t/2500)],[0.0]]))
            )
	    acc_value = np.concatenate(
                (acc_value, np.array([[-0.0197*math.sin(math.pi*t/5000)],[-0.0631*math.sin(math.pi*t/2500)],[0.0]]))
            )
	    jer_value = np.concatenate(
                (jer_value, np.array([[-0.0012*math.cos(math.pi*t/5000)],[-0.0079*math.cos(math.pi*t/2500)],[0.0]]))
            )
	    sna_value = np.concatenate(
                (sna_value, np.array([[0.000078*math.sin(math.pi*t/5000)],[0.001*math.sin(math.pi*t/2500)],[0.0]]))
            )
        pos_value = pos_value.reshape((10002, 3))
	vel_value = vel_value.reshape((10002, 3))
	acc_value = acc_value.reshape((10002, 3))
	jer_value = jer_value.reshape((10002, 3))
	sna_value = sna_value.reshape((10002, 3))
	self.rel_trj_pos = copy.deepcopy(pos_value)
	self.rel_trj_vel = copy.deepcopy(vel_value)
	self.rel_trj_acc = copy.deepcopy(acc_value)
	self.rel_trj_jer = copy.deepcopy(jer_value)
	self.rel_trj_sna = copy.deepcopy(sna_value)

    def finddesT(self):
	rho1=np.array([0.95, 0.0, 0.15])
	rho2=np.array([0.0, 0.95, 0.15])
	rho3=np.array([-0.95, 0.0, 0.15])
	rho4=np.array([0.0, -0.95, 0.15])
	J00=np.dot(self.payload_inertia,self.payload_angvelocity)
	sigma2=np.cross(J00,self.payload_angvelocity,axis=0)
	mat=np.dot(skewsymetric(self.payload_angvelocity),rho1+rho2+rho3+rho4)
	R0=rotation_matrix_from_quaternion(self.payload_orientation).reshape(3,3)
	sigma3=self.payload_mass*np.dot(np.dot(skewsymetric(mat),R0.transpose()),self.payload_velocity)/4;
	skrho=np.dot(skewsymetric(rho1+rho2+rho3+rho4),skewsymetric(self.payload_angvelocity))
	sigma4=self.payload_mass*np.dot(np.dot(skrho,R0.transpose()),self.payload_velocity)/4;
	skrho2=np.dot(np.dot(skewsymetric(rho1+rho2+rho3+rho4),R0.transpose()),self.payload_velocity)
	sigma5=self.payload_mass*np.cross(skrho2,self.payload_angvelocity,axis=0)/4
	#garray=np.array([[0.0],[0.0],[9.81]])
	#sigma6=np.dot(np.dot(skewsymetric(rho1+rho2+rho3+rho4),R0.transpose()),garray)/4
	M0=-sigma2+sigma3+sigma4+sigma5  #-sigma6
	pseud=np.linalg.pinv(skewsymetric(rho1+rho2+rho3+rho4))
	self.desT=4*np.dot(np.dot(R0,pseud),M0)/self.payload_mass
	#self.er=np.identity(3)-R0	

    def getdesT(self):
	return self.er

    def update_desired_F(self):
	#self.findloadvel()
	#self.finddesT()
	self.finddesT()
        self.update_pos_err()
        self.update_vel_err()
        self.desired_F = -(np.dot(self.K_p, self.e_positions) + np.multiply(self.k_pI, self.e_p_integral)) - \
                          (np.dot(self.K_v, self.e_velocities)) + np.array([[0.0], [0.0], [self.mass * self.g]]) + \
                           np.multiply(self.mass, self.desired_acceleratons)+self.desT/4
	self.logvalues()

    def logvalues(self):
	self.filepos=open(self.f1,"a")
	self.filevel=open(self.f2,"a")
	self.filedespos=open(self.f3,"a")
	self.filedesvel=open(self.f4,"a")
	self.filepaypos=open(self.f5,"a")
	self.filepayvel=open(self.f6,"a")
	self.filepaydespos=open(self.f7,"a")
	self.filedesT=open(self.f8,"a")
	self.filepayangvel=open(self.f9,"a")
	self.filepayorien=open(self.f10,"a")
	self.fileinput=open(self.f11,"a")
	self.filepos.write(str(self.positions_quads[0].item())+"\n"+str(self.positions_quads[1].item())+"\n"+str(self.positions_quads[2].item())+"\n")
	self.filevel.write(str(self.velocities_quads[0].item())+"\n"+str(self.velocities_quads[1].item())+"\n"+str(self.velocities_quads[2].item())+"\n")
	self.filedespos.write(str(self.desired_positions[0].item())+"\n"+str(self.desired_positions[1].item())+"\n"+str(self.desired_positions[2].item())+"\n")
	self.filedesvel.write(str(self.desired_velocities[0].item())+"\n"+str(self.desired_velocities[1].item())+"\n"+str(self.desired_velocities[2].item())+"\n")
	self.filepaypos.write(str(self.payload_position[0].item())+"\n"+str(self.payload_position[1].item())+"\n"+str(self.payload_position[2].item())+"\n")
	self.filepayvel.write(str(self.payload_velocity[0].item())+"\n"+str(self.payload_velocity[1].item())+"\n"+str(self.payload_velocity[2].item())+"\n")
	self.filepaydespos.write(str(self.des_payload_position[0].item())+"\n"+str(self.des_payload_position[1].item())+"\n"+str(self.des_payload_position[2].item())+"\n")
	self.filedesT.write(str(self.desT[0].item())+"\n"+str(self.desT[1].item())+"\n"+str(self.desT[2].item())+"\n")
	R0=rotation_matrix_from_quaternion(self.payload_orientation).reshape(3,3)
	self.filepayangvel.write(str(self.payload_angvelocity[0].item())+"\n"+str(self.payload_angvelocity[1].item())+"\n"+str(self.payload_angvelocity[2].item())+"\n")
	self.filepayorien.write(str(np.linalg.norm(np.identity(3)-R0))+"\n")
	self.fileinput.write(str(self.u[0].item())+"\n"+str(self.u[1].item())+"\n"+str(self.u[2].item())+"\n"+str(self.u[3].item())+"\n")
	self.filepos.close()
	self.filevel.close()
	self.filedespos.close()
	self.filedesvel.close()
	self.filepaypos.close()
	self.filepayvel.close()
	self.filepaydespos.close()
	self.filedesT.close()
	self.filepayangvel.close()
	self.filepayorien.close()
	self.fileinput.close()

    def update_desired_values(self):
        # desired values
        #print "z_B: ", self.z_B
        self.u[0, 0] = np.dot(self.z_B, self.desired_F)

        # float_u = Float64()
        # float_u.data = self.u[0, 0]/10.0 # for better visualization
        # self.pub_desired_u1.publish(float_u)

        self.z_B_des = (self.desired_F / np.linalg.norm(self.desired_F)).transpose()
        self.x_C_des = np.array([np.cos(self.desired_yaw), np.sin(self.desired_yaw), 0.0])
        self.y_B_des = np.cross(self.z_B_des, self.x_C_des) / \
                       np.linalg.norm(np.cross(self.z_B_des, self.x_C_des))
        self.x_B_des = np.cross(self.y_B_des, self.z_B_des)
        self.desired_R_ = np.concatenate((self.x_B_des.transpose(), self.y_B_des.transpose(),
                                          self.z_B_des.transpose()), axis=1)

        # print "desired jerk: ", self.desired_jerk[0, 0], self.desired_jerk[1, 0], self.desired_jerk[2, 0]
        u1 = self.u[0, 0]
        dot_u1 = np.dot(self.z_B, self.desired_jerk)
        ddot_u1 = np.dot(self.z_B, self.desired_snap)
        # print "self.desired_jerk: ", self.desired_jerk
        # print "dot_u1: ", dot_u1
        # print "self.z_B: ", self.z_B
        h_omega = self.mass / self.u[0, 0] * (self.desired_jerk -
                                                (
                                                    np.multiply(
                                                        dot_u1, self.z_B.transpose()
                                                    )
                                                )
                                              )
        self.desired_omegas[0, 0] = -np.dot(self.y_B_des, h_omega)
        self.desired_omegas[1, 0] = np.dot(self.x_B_des, h_omega)
        self.desired_omegas[2, 0] = np.dot(self.z_B_des, np.array([[0.0], [0.0], [self.desired_d_yaw]]))

        h_angular_acc = self.mass / self.u[0, 0] * (self.desired_snap -
                                                    np.multiply(ddot_u1, self.z_B.T)
                                                  - np.multiply(dot_u1, np.cross(
                                                            self.angular_vel_quads.T, self.z_B).T
                                                        )
                                                  - np.cross(self.angular_vel_quads.T,
                                                             np.multiply(dot_u1, self.z_B)
                                                             + np.multiply(u1, np.cross(
                                                                     self.angular_vel_quads.T, self.z_B)
                                                               )
                                                             ).T
                                                    )

        self.desired_angular_acc[0, 0] = -np.dot(self.y_B_des, h_angular_acc)
        self.desired_angular_acc[1, 0] = np.dot(self.x_B_des, h_angular_acc)
        self.desired_angular_acc[2, 0] = np.dot(self.z_B_des, np.array([[0.0], [0.0], [self.desired_dd_yaw]]))

        vec3_omega_des = Vector3Stamped()
        vec3_omega_des.header.stamp = rospy.Time.now()
        vec3_omega_des.vector = Vector3(self.desired_omegas[0, 0], self.desired_omegas[1, 0],
                                        self.desired_omegas[2, 0])
        self.pub_desired_omegas.publish(vec3_omega_des)

    def update_desired_M(self):
        self.update_omega_err()
        self.update_R_err()
        angular_acc_tmp = \
            np.dot(
                np.dot(
                    np.dot(
                        skewsymetric(self.angular_vel_quads), self.R_.T
                    )
                , self.desired_R_
                )
            , self.desired_omegas
            )

        angular_acc_tmp = angular_acc_tmp - \
            np.dot(
                np.dot(
                    self.R_.T, self.desired_R_
                )
            , self.desired_angular_acc
            )
        self.desired_M = - np.multiply(self.K_R, self.e_R_) - np.multiply(self.K_omega, self.e_omegas)\
                      + np.cross(self.angular_vel_quads.T, np.dot(self.J, self.angular_vel_quads).T).T\
                      - np.dot(self.J, angular_acc_tmp)
        self.u[1:4] = self.desired_M

    def motorSpeedFromU(self):
        aa = self.u
        self.motor_speed = np.dot(self.M_UtoMotor, self.u)

    def multiply_motor_speed(self, k):
        self.motor_speed = np.multiply(self.motor_speed, k)

    def load_ref_trj_payload(self, dimension='xyz'):
        # load payload reference trajectories into lists:
        if dimension == 'xyz':
            self.trj_x_list_payload = self.rel_trj_pos[:, 0].tolist()
            self.trj_y_list_payload = self.rel_trj_pos[:, 1].tolist()
            self.trj_z_list_payload = self.rel_trj_pos[:, 2].tolist()
	    #plt.plot(self.trj_z_list_payload)
	    #plt.show()

            self.trj_vx_list_payload = self.rel_trj_vel[:, 0].tolist()
            self.trj_vy_list_payload = self.rel_trj_vel[:, 1].tolist()
            self.trj_vz_list_payload = self.rel_trj_vel[:, 2].tolist()
        elif dimension == 'x':
            self.trj_x_list_payload = self.abs_trj_pos[:, 0].tolist()
            self.trj_y_list_payload = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()
            self.trj_z_list_payload = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()

            self.trj_vx_list_payload = self.NL_planner.linear_opt.poly_velocity[:, 0].tolist()
            self.trj_vy_list_payload = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()
            self.trj_vz_list_payload = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()

        elif dimension == 'y':
            self.trj_x_list_payload = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list_payload = self.abs_trj_pos[:, 1].tolist()
            self.trj_z_list_payload = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()

            self.trj_vx_list_payload = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()
            self.trj_vy_list_payload = self.NL_planner.linear_opt.poly_velocity[:, 1].tolist()
            self.trj_vz_list_payload = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()

        elif dimension == 'z':
            self.trj_x_list_payload = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list_payload = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_z_list_payload = self.abs_trj_pos[:, 2].tolist()

            self.trj_vx_list_payload = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vy_list_payload = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vz_list_payload = self.NL_planner.linear_opt.poly_velocity[:, 2].tolist()

    def load_trj_lists(self, dimension='xyz'):
        if dimension == 'xyz':
            self.trj_x_list = self.abs_trj_pos[:, 0].tolist()
            self.trj_y_list = self.abs_trj_pos[:, 1].tolist()
            self.trj_z_list = self.abs_trj_pos[:, 2].tolist()
	    #plt.plot(self.trj_x_list)
	    #plt.show()
	    #plt.plot(self.trj_y_list)
	    #plt.show()
	    #plt.plot(self.trj_z_list)
	    #plt.show()

            self.trj_vx_list = self.rel_trj_vel[:, 0].tolist()
            self.trj_vy_list = self.rel_trj_vel[:, 1].tolist()
            self.trj_vz_list = self.rel_trj_vel[:, 2].tolist()

            self.trj_accx_list = self.rel_trj_acc[:, 0].tolist()
            self.trj_accy_list = self.rel_trj_acc[:, 1].tolist()
            self.trj_accz_list = self.rel_trj_acc[:, 2].tolist()

            self.trj_jerkx_list = self.rel_trj_jer[:, 0].tolist()
            self.trj_jerky_list = self.rel_trj_jer[:, 1].tolist()
            self.trj_jerkz_list = self.rel_trj_jer[:, 2].tolist()

            self.trj_snapx_list = self.rel_trj_sna[:, 0].tolist()
            self.trj_snapy_list = self.rel_trj_sna[:, 1].tolist()
            self.trj_snapz_list = self.rel_trj_sna[:, 2].tolist()

        elif dimension == 'x':
            self.trj_x_list = self.abs_trj_pos[:, 0].tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 0].shape)).tolist()

            self.trj_vx_list = self.NL_planner.linear_opt.poly_velocity[:, 0].tolist()
            self.trj_vy_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()
            self.trj_vz_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 0].shape).tolist()

            self.trj_accx_list = self.NL_planner.linear_opt.poly_acc[:, 0].tolist()
            self.trj_accy_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 0].shape).tolist()
            self.trj_accz_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 0].shape).tolist()

            self.trj_jerkx_list = self.NL_planner.linear_opt.poly_jerk[:, 0].tolist()
            self.trj_jerky_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 0].shape).tolist()
            self.trj_jerkz_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 0].shape).tolist()

            self.trj_snapx_list = self.NL_planner.linear_opt.poly_snap[:, 0].tolist()
            self.trj_snapy_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 0].shape).tolist()
            self.trj_snapz_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 0].shape).tolist()

        elif dimension == 'y':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list = self.abs_trj_pos[:, 1].tolist()
            self.trj_z_list = np.multiply(self.positions_quads[2, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()

            self.trj_vx_list = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()
            self.trj_vy_list = self.NL_planner.linear_opt.poly_velocity[:, 1].tolist()
            self.trj_vz_list = np.zeros(self.NL_planner.linear_opt.poly_pos[:, 1].shape).tolist()

            self.trj_accx_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 1].shape).tolist()
            self.trj_accy_list = self.NL_planner.linear_opt.poly_acc[:, 1].tolist()
            self.trj_accz_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 1].shape).tolist()

            self.trj_jerkx_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 1].shape).tolist()
            self.trj_jerky_list = self.NL_planner.linear_opt.poly_jerk[:, 1].tolist()
            self.trj_jerkz_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 1].shape).tolist()

            self.trj_snapx_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 1].shape).tolist()
            self.trj_snapy_list = self.NL_planner.linear_opt.poly_snap[:, 1].tolist()
            self.trj_snapz_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 1].shape).tolist()

        elif dimension == 'z':
            self.trj_x_list = np.multiply(self.positions_quads[0, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_y_list = np.multiply(self.positions_quads[1, 0], np.ones(self.NL_planner.linear_opt.poly_pos[:, 1].shape)).tolist()
            self.trj_z_list = self.abs_trj_pos[:, 2].tolist()

            self.trj_vx_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vy_list = np.zeros(self.NL_planner.linear_opt.poly_velocity[:, 2].shape).tolist()
            self.trj_vz_list = self.NL_planner.linear_opt.poly_velocity[:, 2].tolist()

            self.trj_accx_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 2].shape).tolist()
            self.trj_accy_list = np.zeros(self.NL_planner.linear_opt.poly_acc[:, 2].shape).tolist()
            self.trj_accz_list = self.NL_planner.linear_opt.poly_acc[:, 2].tolist()

            self.trj_jerkx_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 2].shape).tolist()
            self.trj_jerky_list = np.zeros(self.NL_planner.linear_opt.poly_jerk[:, 2].shape).tolist()
            self.trj_jerkz_list = self.NL_planner.linear_opt.poly_jerk[:, 2].tolist()

            self.trj_snapx_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 2].shape).tolist()
            self.trj_snapy_list = np.zeros(self.NL_planner.linear_opt.poly_snap[:, 2].shape).tolist()
            self.trj_snapz_list = self.NL_planner.linear_opt.poly_snap[:, 2].tolist()

    def publish_poly3d_point(self, xyz_payload, xyz_arr, v_xyz_arr, acc_xyz_arr, jerk_xyz_arr, snap_xyz_arr):
        self.des_payload_position = xyz_payload
        self.desired_positions = xyz_arr
	#print self.name
	#print self.desired_positions
        self.desired_velocities = v_xyz_arr
        self.desired_acceleratons = acc_xyz_arr
        self.desired_jerk = jerk_xyz_arr
        self.desired_snap = snap_xyz_arr
        self.publish_desired_trj()

    def publish_poly3d_trj(self):
        #if self.isPolynomialSolved:
            if len(self.trj_x_list) == 1:
                # hzyu
                # trj_payload_tmp = np.zeros((3, 1))
                trj_payload_tmp = np.array([self.trj_x_list_payload[0], self.trj_y_list_payload[0], self.trj_z_list_payload[0]]).reshape((3, 1))
                trj_tmp = np.array([self.trj_x_list[0], self.trj_y_list[0], self.trj_z_list[0]]).reshape((3, 1))
                trj_v_tmp = np.array([self.trj_vx_list[0], self.trj_vy_list[0], self.trj_vz_list[0]]).reshape((3, 1))
                trj_acc_tmp = np.array([self.trj_accx_list[0], self.trj_accy_list[0], self.trj_accz_list[0]]).reshape((3, 1))
                trj_jerk_tmp = np.array([self.trj_jerkx_list[0], self.trj_jerky_list[0], self.trj_jerkz_list[0]]).reshape((3, 1))
                trj_snap_tmp = np.array([self.trj_snapx_list[0], self.trj_snapy_list[0], self.trj_snapz_list[0]]).reshape((3, 1))
            else:
                # hzyu
                # trj_payload_tmp = np.zeros((3, 1))
                trj_payload_tmp = np.array([self.trj_x_list_payload.pop(0), self.trj_y_list_payload.pop(0), self.trj_z_list_payload.pop(0)]).reshape((3, 1))
                trj_tmp = np.array([self.trj_x_list.pop(0), self.trj_y_list.pop(0), self.trj_z_list.pop(0)]).reshape((3, 1))
                trj_v_tmp = np.array([self.trj_vx_list.pop(0), self.trj_vy_list.pop(0), self.trj_vz_list.pop(0)]).reshape((3, 1))
                trj_acc_tmp = np.array([self.trj_accx_list.pop(0), self.trj_accy_list.pop(0), self.trj_accz_list.pop(0)]).reshape((3, 1))
                trj_jerk_tmp = np.array([self.trj_jerkx_list.pop(0), self.trj_jerky_list.pop(0), self.trj_jerkz_list.pop(0)]).reshape((3, 1))
                trj_snap_tmp = np.array([self.trj_snapx_list.pop(0), self.trj_snapy_list.pop(0), self.trj_snapz_list.pop(0)]).reshape((3, 1))
            self.publish_poly3d_point(trj_payload_tmp, trj_tmp, trj_v_tmp, trj_acc_tmp, trj_jerk_tmp, trj_snap_tmp)
        #else:
            #print "Polynomial has not been solved yet! Please solve the poly coefficients first!"

    def hover(self, target_height):
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)
        while not rospy.is_shutdown():
            self.set_hover_des(target_height)
            self.publish_err()
            self.update_current_state()
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def run(self):
        self.optimize()
        self.getPlanUpToSnap(frequency=self.frequency)
        self.load_trj_lists()

        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()

    def hover_and_trj_xy(self, dimension='x'):
        self.optimize()
        self.getPlanUpToSnap(frequency=self.frequency)
        # self.calculatePosVelAcc()
        # self.NL_planner.linear_opt.plot_derivatives(order=0)
        # self.NL_planner.linear_opt.plot_derivatives(order=1)
        # self.NL_planner.linear_opt.plot_derivatives(order=2)
        # self.NL_planner.linear_opt.plot_derivatives(order=3)
        # self.NL_planner.linear_opt.plot_derivatives(order=4)

        '''
        init ROS node
        '''
        rospy.init_node(self.name, anonymous=True)
        # Timer
        self.initial_time = rospy.Time.now()
        self.t = self.initial_time
        self.rate = rospy.Rate(self.frequency)
        rospy.sleep(3.0)

        '''
        ROS Loop
        '''
        while not rospy.is_shutdown():
            if self.hover_duration < 5.0:
                self.set_hover_des(target_height=1.5)
            else:
                if not self.offset_added:
                    print "hovering finished, going into the next phase..."
                    self.update_offset()
                    self.update_offset_xyz()
                    self.load_trj_lists(dimension=dimension)
                    self.load_ref_trj_payload(dimension=dimension)
                    print "offset added"
                self.publish_poly3d_trj()
            self.publish_err()
            self.update_current_state()
            self.update_desired_inputs()
            self.motorSpeedFromU()
            self.send_motor_command()
            self.rate.sleep()


if __name__ == '__main__':
    dimension = 3
    mellinger_nl = Mellinger('hummingbird','',0.0, 0.0, 0.0, dimension=dimension, N=10)

    mellinger_nl.NL_planner.setVerticesPosVel(
                               positions=[
                                        [0.0, 0.0, 0.0],
                                        [2.0, 0.0, 2.0],
                                        [4.0, 0.0, 4.0]
                               ], velocities=[
                                 [0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0]
                                  ])
    mellinger_nl.hover_and_trj_xy()






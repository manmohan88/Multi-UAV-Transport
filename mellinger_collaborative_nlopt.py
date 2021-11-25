from mellinger_trj_nlopt import Mellinger
import rospy
import numpy as np

class CentrolizedController(object):
    def __init__(self, name='controller'):
        dimension = 3
        self.name = name
        self.controllers = [
                      Mellinger('hummingbird', 0, 0.95, 0.0, 1.20, dimension, N=10),
                      Mellinger('hummingbird', 1, 0.0, 0.95, 1.20, dimension, N=10),
                      Mellinger('hummingbird', 2, -0.95, 0.0, 1.20, dimension, N=10),
                      Mellinger('hummingbird', 3, 0.0, -0.95, 1.20, dimension, N=10)
                      ]
        self.num_of_quads = 4
	#self.filen=open("logfile4.txt","w")
        self.initial_time = None
        self.rate = None
        self.t = None
	self.str1=""
        self.hover_begin_time = None
        self.hover_duration = 0.0
        self.frequency = 50.0

        positions = [[0.0, 0.0, 0.0],
		     [2.0, 0.0, 2.0],
		     [1.414, 1.414, 2.0],
                     [0.0, 2.0, 2.0],
		     [-1.414, 1.414, 2.0],
                     [-2.0, 0.0, 2.0],
		     [-1.414, -1.414, 2.0],
		     [0.0, -2.0, 2.0],
		     [1.414, -1.414, 2.0],
		     [2.0, 0.0, 2.0]]
        #velocities = [[0.0, 0.0, 0.0],
		      #[0.0, 1.57, 0.0],
		      #[-1.11, 1.11, 0.0],
                      #[-1.57, 0.0, 0.0],
		      #[-1.11, -1.11, 0.0],
		      #[0.0, -1.57, 0.0],
		      #[1.11, -1.11, 0.0],
		      #[1.57, 0.0, 0.0],
		      #[1.11, 1.11, 0.0],
                      #[0.0, 1.57, 0.0]]
	velocities = [[0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
		      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0]]

        #for controller in self.controllers:
        #    controller.NL_planner.setVerticesPosVel(positions, velocities)

    def hover_and_trj_xy(self, dimension='xyz'):
        for nl_opt in self.controllers:
	    nl_opt.getdestrj2()
            #nl_opt.optimize()
            #nl_opt.getPlanUpToSnap(frequency=50)

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
            for i_mellinger in self.controllers:
                if i_mellinger.hover_duration < 5.0:
                    i_mellinger.set_hover_des(target_height=1.5)
                else:
                    if not i_mellinger.offset_added:
                        print "hovering finished, going into the next phase..."
                        #i_mellinger.update_offset()
                        i_mellinger.update_offset_xyz()
                        i_mellinger.load_trj_lists(dimension=dimension)
			i_mellinger.load_ref_trj_payload(dimension=dimension)
                        print "offset added"
                    i_mellinger.publish_poly3d_trj()
                i_mellinger.publish_err()
                i_mellinger.update_current_state()
                i_mellinger.update_desired_inputs()
                i_mellinger.motorSpeedFromU()
                i_mellinger.send_motor_command()
		#data=np.linalg.norm(i_mellinger.getdesT())
		#self.filen.write(str(data)+"\n")
		#self.filen.write("\n")
		#print data
            self.rate.sleep()
	#self.filen.write(self.str1)
	#self.filen.close()


if __name__ == '__main__':
    central_controller = CentrolizedController()
    central_controller.hover_and_trj_xy()


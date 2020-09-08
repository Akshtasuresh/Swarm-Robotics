import numpy as np
import pybullet as p
import itertools

class Robot():
    """ 
    The class is the interface to a single robot
    """
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()
        self.passedtime=0
        self.converttime=10
        self.is_formation=0
        self.formation=[]
        
        self.x=None
        self.y=None

        # No friction between body and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)
            
        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        

    def reset(self):
        """
        Moves the robot back to its initial position 
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))
            
    def set_wheel_velocity(self, vel):
        """ 
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel) 
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
            targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]
    
    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received
        
    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])
        
    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors
    
    def compute_controller(self):
        """ 
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED
        
        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """
        
        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()
        
        #send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)
            
        self.passedtime+=self.dt
        
        #Form Square
        if self.passedtime< self.converttime:
            self.formation=[[0.5,-1.5],[0.5,0.5],[1.5,-1.5],[1.5,0.5],[2.5,-1.5],[2.5,0.5]]
            self.x,self.y=np.average(self.formation,axis=0)
            
        # Form L
        elif self.passedtime > self.converttime and self.passedtime < 2*self.converttime:
            self.formation = [[0.5,-1.25],[2.5,-0.25],[1.5,-1.25],[2.5,0.75],[2.5,-1.25],[2.5,1.75]]
            self.x,self.y = np.average(self.formation,axis=0)  
            
        #Form Horizontal Line
        elif self.passedtime > 2*self.converttime and self.passedtime < 5.75*self.converttime:
            self.formation = [[2.5,3.5],[2.5,5.5],[2.5,2.5],[2.5,6.5],[2.5,4.5],[2.5,7.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
        
        #Form Circle around ball 1
        elif self.passedtime > 6*self.converttime and self.passedtime < 7*self.converttime:
            self.formation = [[2.25,3.5],[2.25,4.5],[1.75,3.5],[1.75,4.5],[2.5,4],[1.5,4]]
            self.x,self.y = np.average(self.formation,axis=0)
        
        #Form Circle at Goal1
        elif self.passedtime > 7*self.converttime and self.passedtime < 9*self.converttime:
            
            self.formation = [[2.75,4.95],[2.75,5.95],[2.25,4.95],[2.25,5.95],[3,5.45],[2,5.45]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Move away
        elif self.passedtime > 9.5*self.converttime and self.passedtime < 11.5*self.converttime:
            self.formation = [[2.75,4.5],[2.75,6.5],[2.25,4.5],[2.25,6.5],[3.5,5.5],[1.5,5.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form vertical lines
        elif self.passedtime > 11.5*self.converttime and self.passedtime < 13*self.converttime:
            self.formation = [[3.5,3.5],[2.5,7.5],[2.5,3.5],[1.5,7.5],[3.5,7.5],[1.5,3.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Horizontal Lines
        elif self.passedtime > 13*self.converttime and self.passedtime < 14.5*self.converttime:
            self.formation = [[3.5,2.5],[3.5,6.5],[3.5,4.5],[3.5,7.5],[3.5,5.5],[3.5,3.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Horizontal Lines
        elif self.passedtime > 14.5*self.converttime and self.passedtime < 16*self.converttime:
            self.formation = [[4.5,2.5],[4.5,6.5],[4.5,4.5],[4.5,7.5],[4.5,5.5],[4.5,3.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
        
        # Form Horizontal Line
        elif self.passedtime > 16*self.converttime and self.passedtime < 17.5*self.converttime:
            self.formation = [[4.5,-0.5],[4.5,3.5],[4.5,1.5],[4.5,4.5],[4.5,2.5],[4.5,0.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
        
        # Form Circle around ball 2
        elif self.passedtime > 18*self.converttime and self.passedtime < 19.5*self.converttime:
            
            self.formation = [[3.5,2],[4.25,2.5],[4.25,1.5],[3.75,2.5],[4.5,2],[3.75,1.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Circle at the position
        elif self.passedtime > 20*self.converttime and self.passedtime < 21.5*self.converttime:
            self.formation = [[3.5,3.5],[4.25,4],[4.25,3],[3.75,4],[4.5,3.5],[3.75,3]]
            self.x,self.y = np.average(self.formation,axis=0) 
                
        # Form Circle at the position
        elif self.passedtime > 21.5*self.converttime and self.passedtime < 23*self.converttime:
            self.formation = [[1.75,3.5],[2.5,4],[2.5,3],[2,4],[2.75,3.5],[2,3]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Circle at the position
        elif self.passedtime > 23*self.converttime and self.passedtime < 25*self.converttime:
            self.formation = [[0,3.5],[0.75,4],[0.75,3],[0.25,4],[1,3.5],[0.25,3]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Circle at Goal2
        elif self.passedtime > 25*self.converttime and self.passedtime < 28.5*self.converttime:
            self.formation = [[0,5.55],[0.75,6.05],[0.75,5.05],[0.25,6.05],[1,5.55],[0.25,5.05]]
            self.x,self.y = np.average(self.formation,axis=0)  
            
        # Move away
        elif self.passedtime > 29*self.converttime and self.passedtime < 31.5*self.converttime:
            self.formation = [[-0.5,5.5],[0.75,6.5],[0.75,4.5],[0.25,6.5],[1.5,5.5],[0.25,4.5]]
            self.x,self.y = np.average(self.formation,axis=0) 
            
        # Form Vertical Lines
        elif self.passedtime > 32*self.converttime and self.passedtime < 34.5*self.converttime:
            self.formation = [[-0.5,3.5],[0.5,7.5],[1.5,3.5],[-0.5,7.5],[1.5,7.5],[0.5,3.5]]
            self.x,self.y = np.average(self.formation,axis=0)
            
        # Form Vertical Lines
        elif self.passedtime > 34.5*self.converttime and self.passedtime < 36*self.converttime:
            self.formation = [[-2.5,3.5],[-1.5,7.5],[-0.5,3.5],[-2.5,7.5],[-0.5,7.5],[-1.5,3.5]]
            self.x,self.y = np.average(self.formation,axis=0)
            
        # Form Horizontal Line
        elif self.passedtime > 36*self.converttime and self.passedtime < 37.5*self.converttime:
            self.formation = [[-0.5,1.5],[-0.5,5.5],[-0.5,3.5],[-0.5,4.5],[-0.5,6.5],[-0.5,2.5]]
            self.x,self.y = np.average(self.formation,axis=0)
            
        # Form Horizontal Lines
        elif self.passedtime > 37.5*self.converttime and self.passedtime < 39*self.converttime:
            self.formation = [[-0.5,-7.5],[-0.5,-3.5],[-0.5,-5.5],[-0.5,-4.5],[-0.5,-2.5],[-0.5,-6.5]]
            self.x,self.y = np.average(self.formation,axis=0)
        
        # Form Horizontal Lines
        elif self.passedtime > 39*self.converttime and self.passedtime < 40.5*self.converttime:
            self.formation = [[0.5,-7.5],[0.5,-3.5],[0.5,-5.5],[0.5,-4.5],[0.5,-2.5],[0.5,-6.5]]
            self.x,self.y = np.average(self.formation,axis=0)
        
        # Form Horizontal Lines
        elif self.passedtime > 40.5*self.converttime and self.passedtime < 44.5*self.converttime:
            self.formation = [[0.75,-3.5],[0.75,0.5],[0.75,-1.5],[0.75,-0.5],[0.75,1.5],[0.75,-2.5]]
            self.x,self.y = np.average(self.formation,axis=0)
            
        # Form Horizontal Lines
        elif self.passedtime > 44.5*self.converttime and self.passedtime < 48.5*self.converttime:
            self.formation = [[0.75,-3.5],[2.5,0.5],[0.75,-0.5],[0.75,0.5],[1.5,1.5],[0.75,-2.5]]
            self.x,self.y = np.average(self.formation,axis=0)
        
        # Form Horizontal Lines    
        elif self.passedtime > 48.5*self.converttime and self.passedtime < 52.5*self.converttime:
            self.formation = [[0.75,-1.5],[2.5,0.5],[2.5,-0.5],[0.75,0.5],[1.5,1.5],[0.75,-0.5]]
            self.x,self.y = np.average(self.formation,axis=0)
            
        # Form Diamond    
        elif self.passedtime > 52.5*self.converttime and self.passedtime < 55.5*self.converttime:
            self.formation = [[1.5,-1.5],[2.5,0.5],[2,-0.5],[0.5,0.5],[1.5,1.5],[1,-0.5]]
            self.x,self.y = np.average(self.formation,axis=0)
        
        # check if position of the neighbors have been received and compute the desired change in position
        dx = 0.
        dy = 0.
        
        ax=0
        ay=0
        max_dist=1
        KF=10
        KT=2.5
        DT=np.sqrt(KT)
        KO=7
        
        if messages:
            for m in messages:
                curr_obs_dist = np.linalg.norm(pos-m[1][0])

                if(curr_obs_dist<max_dist):
                    ax += (-KO*(curr_obs_dist-max_dist)/(curr_obs_dist ** 3))*(pos[0]-m[1][0])  # for x coordinate
                    ay += (-KO*(curr_obs_dist-max_dist)/(curr_obs_dist ** 3))*(pos[1]-m[1][1])  # for y coordinate
                else:
                    ax += 0
                    ay += 0
        
        
        if messages:
            for m in messages:
                dx += KF*((m[1][0]-pos[0]) - (self.formation[m[0]][0]-self.formation[self.id][0])) + KT*np.minimum(self.formation[self.id][0]-pos[0],1) - DT*(p.getBaseVelocity(self.pybullet_id)[0][0]) + self.dt*ax + p.getBaseVelocity(self.pybullet_id)[0][0]
                dy += KF*((m[1][1]-pos[1]) - (self.formation[m[0]][1]-self.formation[self.id][1])) + KT*np.minimum(self.formation[self.id][1]-pos[1],1) - DT*(p.getBaseVelocity(self.pybullet_id)[0][1]) + self.dt*ay + p.getBaseVelocity(self.pybullet_id)[0][1]
                #dx += m[1][0] - pos[0]
                #dy += m[1][1] - pos[1]
            
            #compute velocity change for the wheels
            vel_norm = np.linalg.norm([dx, dy]) #norm of desired velocity
            if vel_norm < 0.01:
                vel_norm = 0.01
            des_theta = np.arctan2(dy/vel_norm, dx/vel_norm)
            right_wheel = np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            left_wheel = -np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            self.set_wheel_velocity([left_wheel, right_wheel])
        

    
       

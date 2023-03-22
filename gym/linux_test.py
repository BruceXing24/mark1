import pybullet as p
import pybullet_data as pd
from pybullet_utils import bullet_client
import numpy as np
import time
from mark_leg import Leg
from mark_robot import Robot
from  CPGenerator import  CPG

np.set_printoptions(suppress=True)
LF_FOOT = 6
RF_FOOT = 11
LB_FOOT = 16
RB_FOOT = 21

class Spot:
    def __init__(self):
        self.pybullet_client = bullet_client.BulletClient(connection_mode = p.GUI)
        self.counter = 0
        self.show_fre = 1./240
        self.leg_controller = Leg()
        self.stand_pose = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]
        self.sit_pose = [[0,0,0],[0,0,0],[0,np.pi/3,-np.pi*2/3],[0,np.pi/3,-np.pi*2/3]]
        self.gait_generator = CPG(step_length=0.025,ground_clearance=0.025,ground_penetration=0.001,Tswing=0.3,Tstance=0.3,initial_x=-0.0)
        self.control_fre = 200
        self.robot_id = self.pybullet_client.loadURDF("../urdf/rex.urdf",
                                                   [0, 0.0, 0.7],
                                                   baseOrientation = self.pybullet_client.getQuaternionFromEuler([0, 0, np.pi]),
                                                #    flags=p.URDF_USE_INERTIA_FROM_FILE,
                                                #    useFixedBase = 1
                                                   )
        self.mark = Robot(self.robot_id,self.pybullet_client)
        self.draw_line_foot  = LF_FOOT



    def run(self):
        self.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = self.pybullet_client.loadURDF("plane.urdf")


        num = p.getNumJoints(self.robot_id)
        print(f'num=={num}')
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        # self.pybullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # p.setRealTimeSimulation(1)
        while True:
                if self.counter<400:
                    self.pybullet_client.setGravity(0, 0, -9.8)
                    # for i in range(30):
                    #     link_info = p.getLinkState(self.robot_id,i)
                    #     print(f'{i}link_info{link_info[0]}')
                    self.leg_controller.positions_control2(self.robot_id,self.stand_pose[0],
                                                                      self.stand_pose[1],
                                                                      self.stand_pose[2],
                                                                      self.stand_pose[3],
                    )
                    
                    pre_position_1 = p.getLinkState(self.robot_id, self.draw_line_foot )[0]
                    pre_position_2 = p.getLinkState(self.robot_id, self.draw_line_foot+5 )[0]
                    pre_position_3 = p.getLinkState(self.robot_id, self.draw_line_foot+10 )[0]
                    pre_position_4 = p.getLinkState(self.robot_id, self.draw_line_foot+15 )[0]



                elif self.counter >200:
                    self.pybullet_client.setGravity(0, 0, -9.8)
                    height_nosie = np.random.uniform(0, 20)

                    angles = self.gait_generator.trot_generator(self.gait_generator.trot_timer,'straight',0.8) # degreee
                    angles = angles*np.pi/180.  # radius
                    #self.mark.motor_angle = angles
                    #print(f'angles={angles}')
                    #angles_8 = self.mark.get_8motor_angle()
                    #print(angles_8)
                    #wxyz= self.mark.get_angularV()
                    #print(f"wxyz={wxyz}")

                    self.gait_generator.trot_timer += 1./self.control_fre
                    self.leg_controller.positions_control2(self.robot_id, angles[0:3], angles[3:6], angles[6:9], angles[9:12])
                    # self.leg_controller.positions_control2(self.robot_id,self.stand_pose[0],
                    #                                                   self.stand_pose[1],
                    #                                                   self.stand_pose[2],
                    #                                                   self.stand_pose[3],
                    # )



                    cur_positio_1 = p.getLinkState(self.robot_id, self.draw_line_foot )[0]
                    cur_positio_2 = p.getLinkState(self.robot_id, self.draw_line_foot+5 )[0]
                    cur_positio_3 = p.getLinkState(self.robot_id, self.draw_line_foot+10 )[0]
                    cur_positio_4 = p.getLinkState(self.robot_id, self.draw_line_foot+15 )[0]



                    p.addUserDebugLine(pre_position_1, cur_positio_1, lineColorRGB=[1, 0, 0],lineWidth=3,lifeTime = 10)
                    # p.addUserDebugLine(pre_position_2, cur_positio_2, lineColorRGB=[0, 1, 0],lineWidth=3,lifeTime = 10)
                    # p.addUserDebugLine(pre_position_3, cur_positio_3, lineColorRGB=[0, 1, 0],lineWidth=3,lifeTime = 10)
                    # p.addUserDebugLine(pre_position_4, cur_positio_4, lineColorRGB=[1, 0, 0],lineWidth=3,lifeTime = 10)

                    pre_position_1 = cur_positio_1
                    # pre_position_2 = cur_positio_2
                    # pre_position_3 = cur_positio_3
                    # pre_position_4 = cur_positio_4


                    ori = self.mark.get_ori()


                self.counter+=1
                time.sleep(0.001)
                p.stepSimulation()



if __name__ == '__main__':
    spot = Spot()
    spot.run()
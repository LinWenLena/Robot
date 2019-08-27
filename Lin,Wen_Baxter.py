#!/usr/bin/env python

import baxter_interface
import rospy
from baxter_interface import CHECK_VERSION
import argparse
import struct

import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
l_limb = "left"
r_limb = "right"

L = 0.278
h = 0.064
H = 1.104 + 0.27035

L0 = 270.35 /1000
L1 = 69 /1000
L2 = 364.35/1000
L3 = 69 /1000
L4 = 374.29/1000
L5 = 10 /1000
L6 = 368.30 /1000

class Waypoints(object):
    def __init__(self, savedRecord=None):
	l_limb = "left"
	r_limb = "right"
	self._l_limb_name = l_limb
	self._r_limb_name = r_limb

        # Initialisierung der beiden Arme
        self._l_limb = baxter_interface.Limb("left")
        self._r_limb = baxter_interface.Limb("right")

        # Liste zum speichern der waypoints
        self._waypoints = []  # type: list[(str, dict)]
        self._Angle = []# type: list[(str, dict)]

	hover_distance = 0.15
	self._hover_distance =hover_distance
	verbose = True
	self._verbose = verbose
	

        self._is_recording = False
        self._is_playing_back = False

        # Pruefung ob robot enabled ist oder nicht
        self._robot = baxter_interface.RobotEnable()
        self._init_state = self._robot.state().enabled

        if not self._init_state:
            self._robot.enable()

        # Initialisieren der Navigator Buttons --> zum lernen der Aktionen
        self._l_navigator_io = baxter_interface.Navigator("left")
        self._r_navigator_io = baxter_interface.Navigator("right")

        # Initialisieren der Torso und Schulter Buttons
        self._left_torso_navigator = baxter_interface.Navigator('torso_left')
        self._right_torso_navigator = baxter_interface.Navigator('torso_right')
        self._left_shoulder_button = baxter_interface.DigitalIO('left_shoulder_button')
        self._right_shoulder_button = baxter_interface.DigitalIO('right_shoulder_button')

        # Initialisieren der Gripper buttons
        self._lelf_gripper_dash = baxter_interface.DigitalIO("left_upper_button")
        self._right_gripper_dash = baxter_interface.DigitalIO("right_upper_button")
        self._lelf_gripper_circle = baxter_interface.DigitalIO("left_lower_button")
        self._right_gripper_circle = baxter_interface.DigitalIO("right_lower_button")

        # Pruefvariablen
        self._l_gripper_closed = False
        self._r_gripper_closed = False

        self._l_gripper = baxter_interface.Gripper("left", baxter_interface.CHECK_VERSION)
        self._r_gripper = baxter_interface.Gripper("right", baxter_interface.CHECK_VERSION)

        # Verbinden der Gripper buttons mit den entsprechenden Funktionen zum oeffnen und schliessen der gripper
        self._lelf_gripper_dash.state_changed.connect(self._l_gripper_open)
        self._right_gripper_dash.state_changed.connect(self._r_gripper_open)
        self._lelf_gripper_circle.state_changed.connect(self._l_gripper_close_and_check)
        self._right_gripper_circle.state_changed.connect(self._r_gripper_close_and_check)

        # Verbinden der Torso buttons mit der entsprechenden Funktionen fahren in die neutrale Position
        self._left_torso_navigator.button1_changed.connect(self._move_to_neutral_position)
        self._right_torso_navigator.button1_changed.connect(self._move_to_neutral_position)

        # kallibrieren der Gripper
        self._calibrate_gripper(self._l_gripper)
        self._calibrate_gripper(self._r_gripper)

    def record(self):
        rospy.loginfo("Started recording")
        print "---------------------------------------"
        print "Recording started"
        print("Press Navigator 'OK/Wheel' button to record a new joint "
              "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")

        # setzen der button aktionen --> wenn button0  gedrueckt, wird _l_record_waypoint ausgefuehrt
        self._l_navigator_io.button0_changed.connect(self._l_record_waypoint)
        self._r_navigator_io.button0_changed.connect(self._r_record_waypoint)

        # setzen der button aktionen --> wenn button2  gedrueckt, wird _stop_recording ausgefuehrt
        self._l_navigator_io.button2_changed.connect(self._stop_recording)
        self._r_navigator_io.button2_changed.connect(self._stop_recording)

        # oeffnet alle gripper zur initialisierung
        self._open_grippers()

        self._is_recording = True


        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)


        # Verbindung der buttons mit den Funktionen aufheben
        self._l_navigator_io.button0_changed.disconnect(self._l_record_waypoint)
        self._r_navigator_io.button0_changed.disconnect(self._r_record_waypoint)
        self._l_navigator_io.button2_changed.disconnect(self._stop_recording)
        self._r_navigator_io.button2_changed.disconnect(self._stop_recording)

    def playback(self):
        ''' methode fuer playback modus '''

        rospy.sleep(1.0)

        rospy.loginfo("Playback started")
        print ("----------------------")
        print("Playback started")
        print("Shoulder Buttons to Restart Recording - Ctrl-C to stop script execution")


        # setzen der button aktionen auf _stop_playback
        self._left_torso_navigator.button2_changed.connect(self._stop_playback)
        self._right_torso_navigator.button2_changed.connect(self._stop_playback)
        self._left_shoulder_button.state_changed.connect(self._stop_playback)
        self._right_shoulder_button.state_changed.connect(self._stop_playback)

        # bewegungs geschwindigkeit auf 30% setzen
        self._l_limb.set_joint_position_speed(0.3)
        self._r_limb.set_joint_position_speed(0.3)

        self._is_playing_back = True


        gripperDict = {"left_gripper_close" :   self._l_gripper_close_and_check,
                       "right_gripper_close":   self._r_gripper_close_and_check,
                       "left_gripper_open"  :   self._l_gripper_open,
                       "right_gripper_open" :   self._r_gripper_open}


        armDict = {"left_arm"   :   self._l_limb,
                   "right_arm"   :   self._r_limb}

        while not rospy.is_shutdown() and self._is_playing_back:
            self._open_grippers()  # --> oeffnen der gripper am anfang einer sequenz -> da record auch immer mit geoeffneten grippern beginnt

            # nacheinander ablaufen der wegepunkte
            for waypoint in self._waypoints:
                if rospy.is_shutdown():
                    break
                if waypoint[0] in armDict.keys():
                    armDict[waypoint[0]].move_to_joint_positions(waypoint[1], timeout=20.0, threshold=0.008726646)
                    print 'move %s' % waypoint[0]
                elif waypoint[0] in gripperDict.keys():
                    gripperDict[waypoint[0]](1)
                    print waypoint[0]

        # Verbindung der buttons mit den Funktionen aufheben
        self._left_torso_navigator.button2_changed.disconnect(self._stop_playback)
        self._right_torso_navigator.button2_changed.disconnect(self._stop_playback)
        self._left_shoulder_button.state_changed.disconnect(self._stop_playback)
        self._right_shoulder_button.state_changed.disconnect(self._stop_playback)

    def _calibrate_gripper(self, gripper):
        ''' Kallibrieren der Gripper '''

        if gripper.type() != 'custom':
            if not (gripper.calibrated() or
                            gripper.calibrate() == True):
                rospy.logwarn("%s (%s) calibration failed.",
                              gripper.name.capitalize(),
                              gripper.type())
        else:
            msg = (("%s (%s) not capable of gripper commands."
                    " Running cuff-light connection only.") %
                   (gripper.name.capitalize(), gripper.type()))
            rospy.logwarn(msg)


    # abspeichern der waypoints
    def _l_record_waypoint(self, value):
        ''' abspeichern eines Wegpunkts am linken Arm'''
        if value:
            self._l_position()
            print("Waypoint for left arm recorded")
            self._waypoints.append(["left_arm", self._l_limb.joint_angles()])


    def _r_record_waypoint(self, value):
        ''' abspeichern eines Wegpunkts am rechten Arm'''
        if value:
            self._r_position()
            print("Waypoint for right arm recorded")
            self._waypoints.append(["right_arm", self._r_limb.joint_angles()])

    def _stop_recording(self, value):
        ''' signalisiert dass das Recording der wegpunkte beendet ist '''
        if value:
            self._is_recording = False

    def _stop_playback(self, value):
        ''' signalisiert dass der Playback der wegpunkte beendet ist '''
        if value:
            self._is_playing_back = False
            self._waypoints = []

# Lin, Wen                       
    def _l_gripper_open(self):
        ''' linken gripper oeffnen und im record modus abspeichern '''
        print "left gripper opening recorded"
        self._l_gripper.open()
        rospy.sleep(1.0)

    def _r_gripper_open(self):
        ''' rechten gripper oeffnen und im record modus abspeichern '''
        print "right gripper opening recorded"
        self._r_gripper.open()
        rospy.sleep(1.0)

    def _l_gripper_close(self):
        ''' linken gripper schliessen und im record modus abspeichern '''
        print "left gripper closing recorded"
        self._l_gripper.close()
        rospy.sleep(1.0)

    def _r_gripper_close(self):
        ''' rechten gripper schliessen und im record modus abspeichern '''
        print "right gripper closing recorded"

        self._r_gripper.close()
        rospy.sleep(1.0)

    def _l_gripper_close_and_check(self, value):
        ''' linken gripper schliessen, checken ob etwa gegriffen wurde und im record modus abspeichern '''
        print "left gripper closing recorded"
        self._l_gripper.close()

        if self._l_gripper.force() <= 20:
            print "nothing picked on left gripper"
            self._l_gripper.open()
        rospy.sleep(0.5)

    def _r_gripper_close_and_check(self, value):
        ''' rechten gripper schliessen, checken ob etwa gegriffen wurde und im record modus abspeichern '''
        self._r_gripper.close()

        if self._r_gripper.force() <= 20:
            print "nothing picked on left gripper"
            self._r_gripper.open()
        rospy.sleep(1)

#-------------------------------------------------------------------------------------------------------------------
    def _open_grippers(self):
        ''' oeffnen aller gripper '''
        if self._l_gripper.ready() and self._r_gripper.ready():
            self._l_gripper.open()
            self._r_gripper.open()
            self._l_gripper_closed = False
            self._r_gripper_closed = False

    def _move_to_neutral_position(self, value):
        ''' setzt beite arme in die neutrale position '''
        if value and self._is_recording:
            print(" Clean record and move to neutral pose...")
            rospy.sleep(2)
            self._waypoints = []
            self._l_limb.move_to_neutral()
            self._r_limb.move_to_neutral()
            self._open_grippers()

    def clean_shutdown(self):
        ''' fuehrt einen sauberen skript shutdown durch '''
        print("\nExiting example...")
        if not self._init_state:
            self._robot.disable()
        return True

#Lin, Wen
    def _l_position(self):
        current_pose = self._l_limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        self._Angle.append(ik_pose)
        print(ik_pose)

    def _r_position(self):
        current_pose = self._r_limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        self._Angle.append(ik_pose)
        print(ik_pose)

    def ik_request(self, pose,limb):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        rx = pose.orientation.x
        ry = pose.orientation.y
        rz = pose.orientation.z
	if(limb == l_limb):
		ns = "ExternalTools/" + l_limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		rospy.wait_for_service(ns, 5.0)
	elif(limb == r_limb):
		ns = "ExternalTools/" + r_limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		rospy.wait_for_service(ns, 5.0)

        theta1 = math.atan2(y,x)
                m1 = (-2)*(math.sqrt(L2**2 + L3**2))*z
                m2 = (2*math.sqrt(L2**2 + L3**2)*(L1-(x/math.cos(theta1))))**2 + 4*(L2**2 + L3**2)*(z**2)
                
                m3 = (x**2/(math.cos(theta1)**2)) + L1**2 + L2**2 +L3**2 - L4**2 + z**2 - 2*((L1*x)/math.cos(theta1))
                n2 = m3 - 2*x*math.sqrt(L2**2 + L3**2)*(L1 - x/math.cos(theta1))
                m3 = m3**2
                if(m2-m3>=0):
                        theta2 = math.atan((m1 - math.sqrt(m2-m3))/n2)
                else:
                        theta2 = math.atan((m1 - math.sqrt(m3-m2))/n2)
                theta3 = 0
                theta4 = math.atan((-z-(math.sqrt(L2**2+L3**2))*math.sin(theta2) )/ ((x/math.cos(theta1))-L1-math.sqrt(L2**2 + L3**2)*math.cos(theta2)) ) - theta2
                if (math.cos(theta1) == 0 ):
                        raise ValueError('no valid theta 1')
                if (theta1 > math.radians(51) or theta1 < math.radians(-141) ):
                        raise ValueError('no valid theta 1')
                if (theta2 > math.radians(60) or theta2 < math.radians(-123) ):
                        raise ValueError('no valid theta 2')
                if (theta4 > math.radians(150) or theta4 < math.radians(-3) ):
                        raise ValueError('no  valid theta 4')

                rz = math.degrees(oz)
                ry = math.degrees(oy)
                rx = math.degrees(ox)
                t11 = math.cos(rz) * math.cos(ry)
                t12 = -math.sin(rz)*math.cos(rx)+math.cos(rz)*math.sin(ry)*math.sin(rx)
                t13 = math.sin(rz) * math.sin(rx) +math.cos(rz)*math.sin(ry)*math.cos(rx) 
                t21 = math.sin(rz) * math.cos(ry) 
                t22 = math.cos(rz) * math.cos(rx)+ math.sin(rz) * math.sin(ry) * math.sin(rx)
                t23 = -math.cos(rz) * math.sin(rx) + math.sin(rz) * math.sin(ry) * math.cos(rx)
                t31 = -math.sin(ry)
                t32 = math.cos(ry) * math.sin(rx)
                t33 = math.cos(ry) * math.cos(rx)

                print("matrix T70")
                print(round(t11,3), round(t12,3), round(t13,3), x)
                print(round(t21,3), round(t22,3), round(t23,3), y)
                print(round(t31,3), round(t32,3), round(t33,3), z)
                print("   0       0       0      1")
                T07 = [[t11,t12,t13,x],[t21,t22,t23,y],[t31,t32,t33,z],[0,0,0,1]]


                print("matrix R70")
                print(round(t11,3), round(t12,3), round(t13,3), x)
                print(round(t21,3), round(t22,3), round(t23,3), y)
                print(round(t31,3), round(t32,3), round(t33,3), z)
                print("  0      0        0        1")
                R70 = [[t11,t12,t13],[t21,t22,t23],[t31,t32,t33]]


                tr = math.sqrt(2)/2
                TL = [[tr,tr,0,L],[-tr,tr,0,-h],[0,0,1,H],[0,0,0,1]]
                RL = [[tr,tr,0],[-tr,tr,0],[0,0,1]]
                RL_T = np.linalg.inv(RL)
                TR = [[-tr,tr,0,-L],[-tr,-tr,0,-h],[0,0,1,H],[0,0,0,1]]
                RR = [[-tr,tr,0],[-tr,-tr,0],[0,0,1]]
                RR_T = np.linalg.inv(RR)

                v11 = -math.cos(theta1)* math.sin(theta2+theta4)
                v12 = -math.cos(theta1)*math.cos(theta2+theta4)
                v13 = -math.sin(theta1) # eigentlich negativ
                v21 = -math.sin(theta1)*math.sin(theta2+theta4)
                v22 = -math.sin(theta1)*math.cos(theta2+theta4)
                v23 = +math.cos(theta1) # eigentlich positiv
                v31 = -math.cos(theta2+theta4)
                v32 = math.sin(theta2+theta4)

                V = [[v11,v12,v13],[v21,v22,v23],[v31,v32,0]]
                V_T = np.linalg.inv(V)

                if(limb == l_limb):
                        N = np.dot(R70,RL_T)
                        N = np.dot(N,V_T)
                elif(limb == r_limb):
                        N = np.dot(R70,RR_L)
                        N = np.dot(N,V_T)
                else:
                     raise Error('EXCEPTION: no valid arm joint')

                print("matrix N")
                print(round(N [0][0],3), round(N [0][1],3), round(N [0][2],3))
                print(round(N [1][0],3), round(N [1][1],3), round(N [1][2],3))
                print(round(N [2][0],3), round(N [2][1],3), round(N [2][2],3))


                theta5 = math.atan2(N[2][2],N[1][0])
                if(theta5 == math.pi or theta5 == -math.pi):
                        theta5 = 0

                theta7 = math.atan2(-N[1][1],N[1][0])
                if(theta7 == math.pi or theta7 == -math.pi):
                        theta7 = 0
                
                theta6 = math.atan2(N[1][0]/math.cos(theta7), -N[1][2])

                if (theta5 > math.radians(175) or theta5 < math.radians(-175) ):
                        raise ValueError('no valud theta 5')
                if (theta6 > math.radians(120) or theta6 < math.radians(-90) ):
                        raise ValueError('no valid theta 6')
                if (theta7 > math.radians(175) or theta7 < math.radians(-175) ):
                        raise ValueError('no valid theta 7')

    def _l_approach(self,pose):
        joint_angles = self.ik_request(pose,self._l_limb_name)
        self._l_guarded_l_move_to_joint_position(joint_angles)


    def _r_approach(self,pose):
        joint_angles = self.ik_request(pose,self._r_limb_name)
        self._r_guarded_r_move_to_joint_position(joint_angles)

    def _l_pick(self, joint_angle):
        self._l_gripper_open()
        self._l_approach(joint_angle)
        self._l_gripper_close()

    def _r_pick(self, joint_angle):
        self._r_gripper_open()
        self._r_approach(joint_angle)
        self._r_gripper_close()

    def _l_guarded_l_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._l_limb.move_to_joint_positions(joint_angles)

    def _r_guarded_r_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._r_limb.move_to_joint_positions(joint_angles)

    def _r_place(self, joint_angle):
        self._r_approach(joint_angle)
        self._r_gripper_open()

    def play(self):
            self._l_pick(self._Angle[0])
            self._l_approach(self._Angle[1])
            self._r_approach(self._Angle[2])
            self._r_gripper_close()
            self._l_gripper_open()
            self._r_place(self._Angle[3])


def main():
    rospy.init_node("baxter_manual_teaching_node")

    if len(sys.argv) < 2:
        waypoints = Waypoints()
    else:
        waypoints = Waypoints(str(sys.argv[-1]))

    rospy.on_shutdown(waypoints.clean_shutdown)

    while not rospy.is_shutdown():

        waypoints.record()

        waypoints.play()

if __name__ == '__main__':
    main()

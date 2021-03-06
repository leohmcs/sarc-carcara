#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Polygon, Point
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import sys
from distancefield.msg import Path, PathEq
from tf.transformations import euler_from_quaternion, quaternion_from_euler


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
Laboratorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
João F. R. Baião, <baiaojfr@gmail.com>
"""

class Paths:

    def __init__(self):
        # Cria node, publisher e subscriber
        rospy.init_node("path_generator", anonymous=True)

        self.read_params()

        # Variables of swarm
        self.pos = []
        self.seen_it = []
        self.pos_des = []
        self.pos_des_now = []
        self.got_there = []
        self.fire_extinguished = []
        self.pos_subscriber = []
        self.vis_subscriber = []
        self.reunited = 0

        for i in range(self.number_of_robots):
            self.pos.append([0,0,0])
            self.seen_it.append(0)
            self.pos_des.append([])
            self.pos_des_now.append([])
            self.got_there.append(0)
            self.fire_extinguished.append([])
            self.pos_subscriber.append([])
            self.vis_subscriber.append([])

        # Variables of simulation states and set constants
        self.saw_it = 0
        self.got_there = 0
        self.got_there2 = 0
        self.predefined = [6,6,0]
        self.height = 2

        self.freq = 10.0
        self.rate = rospy.Rate(10)

        # Publishers
        self.pub_path = rospy.Publisher("/example_path" + f"{self.robot_number}", Path, queue_size=10)
        self.pub_path_equation = rospy.Publisher("/example_path_equation" + f"{self.robot_number}", PathEq, queue_size=10)
        self.pub_rviz_curve = rospy.Publisher("/visualization_path" + f"{self.robot_number}", MarkerArray, queue_size=1)
        self.pub_state = rospy.Publisher("/trajectory_state" + f"{self.robot_number}", Float32, queue_size=1)

        # Subscribers
        if self.number_of_robots > 0:
            for i in range(self.number_of_robots):
                self.pos_subscriber[i] = rospy.Subscriber(f"/drone/odom{i}",  Odometry, self.odometry_cb, (i))
                self.vis_subscriber[i] = rospy.Subscriber(f"/vision_state0",  Float32MultiArray, self.vision_cb, (i))



        self.rate.sleep()


    # Callback of robots positions data
    def odometry_cb(self,data,args):
        self.pos[args] = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

    # Callback of robots sensor data
    def vision_cb(self,data, args):
        self.seen_it[args] = data.data[0]
        self.pos_des[args] = list(data.data[1:4])

        self.fire_extinguished[args] = data.data[4]
        

    # ----------  ----------  ----------  ----------  ----------


    # Function to generate path of search
    def refference_path_1(self,N,p_0):

        # Parameter
        dp = 5*2*pi/N
        p = -dp

        # Loop to sample the curve
        path = [[],[],[]]
        for k in range(N):

            # Increment parameter
            p = p + dp

            # Compute a point of the ellipse in a local frame
            x_ref0 = 0.02*k * self.a * cos(p+3)
            y_ref0 = 0.02*k * self.b * sin(p+3)


            # Rotate and displace the point
            x_ref = cos(self.phi) * x_ref0 - sin(self.phi) * y_ref0 + p_0[0] + 0.4
            y_ref = sin(self.phi) * x_ref0 + cos(self.phi) * y_ref0 + p_0[1] + 0.4

            # Save the computed point
            path[0].append(x_ref)
            path[1].append(y_ref)
            path[2].append(self.height)

        return (path)



    # ----------  ----------  ----------  ----------  ----------

    # Function to generate a "go to" path - That is an open path
    def refference_path_2(self,N,p_0,p_des):

        # Parameter
        dp = 2*pi/N
        p = -dp
        #p_0 = [posi[0]+0.005*p_des[0],posi[1]+0.005*p_des[1],posi[2]+0.005*p_des[2]]

        path = [[],[],[]]
        for k in range(N):

            # Increment parameter
            p = p + dp

            # Compute a point of the "rectangular" in a local frame
            '''
            x_ref = p_des[0] + 0.0001*k
            y_ref = p_des[1] + 0.0001*k
            z_ref = p_des[2] + 0.0001*k

            x_ref = (p_0[0] - p_des[0])*k/N + p_des[0]
            y_ref = (p_0[1] - p_des[1])*k/N + p_des[1]
            z_ref = (p_0[2] - p_des[2])*k/N + p_des[2]
            '''
            x_ref = (p_des[0] - p_0[0])*k/N + p_0[0]
            y_ref = (p_des[1] - p_0[1])*k/N + p_0[1]
            z_ref = (p_des[2] - p_0[2])*k/N + p_0[2]
            '''
            # Compute a point of the ellipse in a local frame
            a = np.sqrt((p_des[0] - p_0[0])**2 + (p_des[1] - p_0[1])**2)/2
            x_ref0 = a * cos(p)
            y_ref0 = 0.02* self.b * sin(p)

            gama =np.arctan((p_des[0] - p_0[0])/(p_des[1] - p_0[1])) 

            # Rotate and displace the point
            x_ref = cos(gama) * x_ref0 - sin(gama) * y_ref0 + a*np.cos(gama)
            y_ref = sin(gama) * x_ref0 + cos(gama) * y_ref0 + a*np.sin(gama)
            z_ref = self.height
            '''

            # Save the computed point
            path[0].append(x_ref)
            path[1].append(y_ref)
            path[2].append(z_ref)

        return (path)

    # ----------  ----------  ----------  ----------  ----------


    # Function to generate the path of action
    def refference_path_3(self,N,posi):
        # Parameter
        dp = 2*pi/N
        p = -dp

        # Loop to sample the curve
        path = [[],[],[]]
        for k in range(N):

            # Increment parameter
            p = p + dp

            # Compute a point of the ellipse in a local frame
            x_ref0 = (5 - self.first_y[self.robot_number])*self.a * cos(p)
            y_ref0 = (5 - self.first_y[self.robot_number])*self.b * sin(p)

            # Rotate and displace the point
            x_ref = cos(self.phi) * x_ref0 - sin(self.phi) * y_ref0 + posi[0] #+ 3*self.a/2
            y_ref = sin(self.phi) * x_ref0 + cos(self.phi) * y_ref0 + posi[1] + (5)*self.b

            # Save the computed point
            path[0].append(x_ref)
            path[1].append(y_ref)
            path[2].append(self.height)

        return (path)

    # ----------  ----------  ----------  ----------  ----------



    # Function to create a message of the type polygon, which will carry the points of the curve
    def create_path_msg(self,path):

        # Create 'Polygon' message (array of messages of type 'Point')
        self.path_msg = Path()
        p = Point()
        for k in range(len(path[0])):
            # Create point
            p = Point()
            # Atribute values
            p.x = path[0][k]
            p.y = path[1][k]
            p.z = path[2][k]
            # Append point to polygon
            self.path_msg.path.points.append(p)

        self.path_msg.header.stamp = rospy.Time.now()

        self.path_msg.closed_path_flag = self.closed_path_flag
        self.path_msg.insert_n_points = self.insert_n_points
        self.path_msg.filter_path_n_average = self.filter_path_n_average


        return self.path_msg
    # ----------  ----------  ----------  ----------  ----------





    # Function to send a array of markers, representing the curve, to rviz
    def send_curve_to_rviz(self,path,pub_rviz):

        # Create messsage
        points_marker = MarkerArray()
        marker = Marker()
        # Iterate over the points
        for k in range(len(path[0])):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = k
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.lifetime = rospy.Duration(3)
            # Size of sphere
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            # Color and transparency
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            # Pose
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = path[0][k]
            marker.pose.position.y = path[1][k]
            marker.pose.position.z = path[2][k]
            # Append marker to array
            points_marker.markers.append(marker)

        pub_rviz.publish(points_marker)

        return (points_marker)
    # ----------  ----------  ----------  ----------  ----------


    def arrange(self):
        deltinha = 2
        self.first_x = []
        self.first_y = []

        for nu in range(self.number_of_robots):
            self.first_x.append(0)
            self.first_y.append(0)


        for nu in range(self.number_of_robots):
            if self.number_of_robots < 6:
                if nu == 0:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = 1*deltinha

                elif  0 < nu < 4:
                    self.first_x[nu] = (nu - 2)*deltinha
                    self.first_y[nu] = 0*deltinha

                elif nu == 4:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = -1*deltinha

                else:
                    print("Invalid Robot Number")
                    a = 1/0

            elif 6 <= self.number_of_robots < 14:
                if nu == 0:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = 2*deltinha

                elif 0 < nu < 4:
                    self.first_x[nu] = (nu - 2)*deltinha
                    self.first_y[nu] = 1*deltinha

                elif  3 < nu < 9:
                    self.first_x[nu] = (nu - 6)*deltinha
                    self.first_y[nu] = 0*deltinha

                elif 8 < nu < 12:
                    self.first_x[nu] = (nu - 10)*deltinha
                    self.first_y[nu] = -1*deltinha


                elif nu == 12:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = -2*deltinha

                else:
                    print("Invalid Robot Number")
                    a = 1/0

            elif 14 <= self.number_of_robots < 26:
                if nu == 0:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = 3*deltinha

                elif 0 < nu < 4:
                    self.first_x[nu] = (nu - 2)*deltinha
                    self.first_y[nu] = 2*deltinha

                elif 3 < nu < 9:
                    self.first_x[nu] = (nu - 6)*deltinha
                    self.first_y[nu] = 1*deltinha

                elif  8 < nu < 16:
                    self.first_x[nu] = (nu - 12)*deltinha
                    self.first_y[nu] = 0*deltinha

                elif 15 < nu < 21:
                    self.first_x[nu] = (nu - 18)*deltinha
                    self.first_y[nu] = -1*deltinha

                elif 20 < nu < 24:
                    self.first_x[nu] = (nu - 22)*deltinha
                    self.first_y[nu] = -2*deltinha


                elif nu == 24:
                    self.first_x[nu] = 0*deltinha
                    self.first_y[nu] = -3*deltinha

                else:
                    print("Invalid Robot Number")
                    a = 1/0

            else:
                print("too many robots, my condolences")


    def path(self):


        self.rate.sleep()

        self.arrange()
        # Generate one of the curve types
        path = self.refference_path_1(self.number_of_samples,self.pos[self.robot_number])
        
        # Create message with the points of the curve
        path_msg = self.create_path_msg(path)

        # Wait a bit
        self.rate.sleep()

        sleep(2.0)
        
        # Publish the message

        print ("\33[92m----------------------------\33[0m")
        print ("\33[92mCurve created and publhished\33[0m")
        print ("Sampled samples: ", self.number_of_samples)
        print ("\33[92m----------------------------\33[0m")

        aux = 1
        self.etapa = 0

        self.pub_state.publish(self.etapa)

        # Simulate Process on Rviz
        while not rospy.is_shutdown():

            #print(aux)

            #print(f"Etapa: {self.etapa}")
            #print(f"seeenit{self.seen_it}")
            #print(f"posdes{self.pos_des}")
            #print(f"fireextinguished{self.fire_extinguished}")
            #print(f"subscriber{self.vis_subscriber}")
            if self.robot_number == 1:
                print(self.pos)


            # Wait 5 seconds Stage
            if (self.etapa == 0):
                rospy.sleep(5)
                self.etapa += 1
                print(f"a{self.robot_number}")



            # Search Fire Stage
            elif (not self.saw_it and self.etapa == 1):
                self.pub_state.publish(self.etapa)
                self.send_curve_to_rviz(path, self.pub_rviz_curve)
                self.pub_path.publish(path_msg)
                print(f"b{self.robot_number}")

                for i in self.seen_it:
                    if i == 1.0:
                        self.saw_it = 1



            # Centralize Above Fire Stage
            elif (self.saw_it and self.etapa == 1):
                self.etapa += 1

                for i in range(self.number_of_robots):
                    self.pos_des_now[i] = self.pos_des[i]
                    self.pos_des_now[i][0] += self.first_x[i]
                    self.pos_des_now[i][1] += self.first_y[i]

                self.path2 = self.refference_path_2(self.number_of_samples,self.pos[self.robot_number],self.pos_des_now[self.robot_number])
                self.path_msg2 = self.create_path_msg(self.path2)
                print("c\n")


            elif (self.saw_it and self.etapa == 2 and not self.got_there):

                x = np.linalg.norm(np.array(self.pos[self.robot_number]) - np.array(self.pos_des_now[self.robot_number]))

                if (x <= 0.2):
                    self.got_there = 1
                    self.etapa += 1

                self.pub_state.publish(self.etapa)
                self.pub_path.publish(self.path_msg2)
                self.send_curve_to_rviz(self.path2, self.pub_rviz_curve)
                print("d")


            # Wait for reunion
            elif (self.etapa == 3 and self.got_there and not self.reunited):
                x = []
                for i in range(self.number_of_robots):
                    x.append(np.linalg.norm(np.array(self.pos[i]) - np.array(self.pos_des_now[i])))

                self.reunited = 1

                for i in x:
                    if (i > 0.2):
                        self.reunited = 0
                        break    

                print("e")

            
            # Wait 5 seconds Stage
            elif (self.etapa == 3 and self.got_there and self.reunited):
                rospy.sleep(5)
                self.etapa += 1
                print("f")
            



            # Extinguish Fire Stage
            elif (self.got_there and self.etapa == 4):
                self.etapa += 1

                if self.number_of_robots <= 5:
                    self.number_of_robots_max = 5
                elif 5 < self.number_of_robots <= 13:
                    self.number_of_robots_max = 13
                else:
                    self.number_of_robots_max = 25


                self.pub_state.publish(self.etapa)
                self.pos2 = [self.pos[self.robot_number][0] - self.first_x[self.robot_number], self.pos[self.robot_number][1] - self.first_y[self.robot_number]]
                self.path3 = self.refference_path_3(self.number_of_samples,self.pos2)
                self.path_msg3 = self.create_path_msg(self.path3)
                print("g")
                print(self.etapa)
                print(self.fire_extinguished)

            elif (self.etapa == 5 and not self.fire_extinguished[self.robot_number]):
                print(int((self.number_of_robots-1)/2))
                self.pub_state.publish(self.etapa)
                self.pub_path.publish(self.path_msg3)
                self.send_curve_to_rviz(self.path3, self.pub_rviz_curve)
                print("h")





            # Retreat to Pre-defined Location 
            elif (self.etapa == 5 and self.fire_extinguished[self.robot_number]):
                self.etapa += 1

                self.predefined[0] += self.first_x[self.robot_number]
                self.predefined[1] += self.first_y[self.robot_number]

                self.pos3 = self.pos[self.robot_number]
                self.path2 = self.refference_path_2(self.number_of_samples,self.pos3,self.predefined)
                self.path_msg2 = self.create_path_msg(self.path2)
                print("i")

            elif (self.etapa == 6 and not self.got_there2):
                self.got_there2 = 0
                x = np.linalg.norm(np.array(self.pos[self.robot_number]) - np.array(self.predefined))
                if (x <= 0.2):
                    self.got_there2 = 1
                    self.etapa += 1
                self.pub_state.publish(self.etapa)
                self.pub_path.publish(self.path_msg2)
                self.send_curve_to_rviz(self.path2, self.pub_rviz_curve)
                print("j")
                print(x)

            elif (self.etapa == 7):
                print("Simulation Finished")
                rospy.sleep(5)
                pass
            else:
                print("Something Went Wrong in path generator node")
            self.rate.sleep()
        

    # ---------- !! ---------- !! ---------- !! ---------- !! ----------



    def read_params(self):
        # Obtain the parameters
        # try:
        self.robot_number = int(rospy.get_param("~Robot_number"));
        self.number_of_robots = int(rospy.get_param("~Number_of_robots"));
        self.number_of_samples = int(rospy.get_param("~N_points"));
        self.a = float(rospy.get_param("~a"));
        self.b = float(rospy.get_param("~b"));
        self.phi = float(rospy.get_param("~phi"))*(3.1415926535/180.0);
        self.cx = float(rospy.get_param("~cx"));
        self.cy = float(rospy.get_param("~cy"));
        self.closed_path_flag = bool(rospy.get_param("~closed_path_flag"))
        self.insert_n_points = int(rospy.get_param("~insert_n_points"))
        self.filter_path_n_average = int(rospy.get_param("~filter_path_n_average"))
        self.u_i = float(rospy.get_param("~u_i"))
        self.u_f = float(rospy.get_param("~u_f"))
        self.equation_str = rospy.get_param("~equation")


        # print ("u_lim: ", u_lim)
        print ("u_i: ", self.u_i)
        print ("u_f: ", self.u_f)
        print ("equation_str: ", self.equation_str)

        print("\n\33[92mParameters loaded:\33[0m")
        print("\33[94mnumber_of_samples: " +  str(self.number_of_samples) +"\33[0m")
        print("\33[94ma: " +  str(self.a) +"\33[0m")
        print("\33[94mb: " + str(self.b) +"\33[0m")
        print("\33[94mphi: " + str(self.phi) +"\33[0m")
        print("\33[94mcx: " + str(self.cx) +"\33[0m")
        print("\33[94mcy: " + str(self.cy) +"\33[0m")
        print("\33[94mclosed_path_flag: " + str(self.closed_path_flag) +"\33[0m")
        print("\33[94minsert_n_points: " + str(self.insert_n_points) +"\33[0m")
        print("\33[94mfilter_path_n_average: " +  str(self.filter_path_n_average) +"\33[0m")







# Main function
if __name__ == '__main__':

    #args = rospy.myargv(argv = sys.argv)_

    try:
        caminho = Paths()
        caminho.path()

    except rospy.ROSInterruptException:
        pass

    rospy.sleep(.5)

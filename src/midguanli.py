# !/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float64

flag = 0  # 控制停车，0不停车，1停车
tlag = 0  # 按键停车
plag1 = 0  # 这三个标志位是控制每个停车点只停一次
plag2 = 0
plag3 = 0
plag = 0
kount = 0
distance = 0.0

class PubAndSub1:
    def __init__(self):
        self.pub_control = rospy.Publisher('/car/cmd_vel', Twist, queue_size=10)
        self.sub_cmd_vel = rospy.Subscriber('/car/vel', Twist, self.amclcallback)
        self.sub_teleop_vel = rospy.Subscriber('teleop', Int8, self.teleopback)
        self.sub_dis = rospy.Subscriber('/distance_traveled', Float64, self.distanceCallback)

    def amclcallback(self, guide_twist):
        global flag
        if flag == 0:
            self.pub_control.publish(guide_twist)  # 发布导航速度，车跑
        if flag == 1:
            # 满足停车点范围（一共3个）
            twist1 = Twist()
            twist1.linear.x = 1500
            twist1.linear.y = 0
            twist1.linear.z = 0
            twist1.angular.x = 0
            twist1.angular.y = 0
            twist1.angular.z = 90
            self.pub_control.publish(twist1)
            # stop car 3s
            flag = 0
            rospy.sleep(3)

    def teleopback(self, tele):
        global tlag
        tlag = tele.data
        if tlag == 1:
            twist2 = Twist()
            twist2.linear.x = 1500
            twist2.linear.y = 0
            twist2.linear.z = 0
            twist2.angular.x = 0
            twist2.angular.y = 0
            twist2.angular.z = 90
            self.pub_control.publish(twist2)
            # stop car 3s
            tlag = 0
            rospy.sleep(10)
        if tlag == 3:
            # 添加相应的处理代码
            pass

    def distanceCallback(self, distance_msg):
        global distance, flag, plag1, plag2, plag3, kount, plag
        distance = distance_msg.data
        min_distance1 = 1.92  # 设置第一个停车点最小距离
        max_distance1 = 2.60  # 设置第一个停车点最大距离
        min_distance2 = 13.50 # 设置第二个停车点最小距离
        max_distance2 = 13.80  # 设置第二个停车点最大距离
        min_distance3 = 4.45  # 设置第三个停车点最小距离
        max_distance3 = 5.0  # 设置第三个停车点最大距离
        min_distance = 8.45  # kount最小距离
        max_distance = 9.0  # kount最大距离
        if min_distance1 <= distance <= max_distance1 and plag1 == 0:
            # 一旦满足第一个停车点的条件
            flag = 1  # 控制停车，0不停车，1停车
            plag1 = 1  # 再也不进第一个停车点了

        if min_distance2 <= distance <= max_distance2 and plag2 == 0:
            # 一旦满足第二个停车点的条件
            flag = 1  # 控制停车，0不停车，1停车
            plag2 = 1  # 再也不进第二个停车点了
            kount = 1

        if min_distance <= distance <= max_distance and plag == 0:
            # 第二个没停好点不影响第三个点
            plag = 1  # 再也不进了
            kount = 1

        if min_distance3 <= distance <= max_distance3 and plag3 == 0 and kount == 1:
            # 一旦满足第三个停车点的条件
            flag = 1  # 控制停车，0不停车，1停车
            plag3 = 1  # 再也不进第三个停车点了

if __name__ == '__main__':
    rospy.init_node('midguanli')
    myPAS = PubAndSub1()
    rospy.spin()

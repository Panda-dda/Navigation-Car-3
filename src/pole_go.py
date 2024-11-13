#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

def extract_and_find_closest(data):
    indices = []
    values = []

    # 提取大于3的值及其索引
    for i in range(len(data)):
        if data[i] > 0.3:
            values.append(data[i])
            indices.append(i)
    # print(data)
    # 如果没有找到大于3的值，返回适当的值
    if len(values) < 2:
        return None, [], []  # 或者返回 -1

    # 找到索引最接近的两个值
    min_distance = float('inf')  # 初始化为正无穷大
    closest_pair = (None, None)  # 存储索引对
    closest_values = (None, None)  # 存储值对

    for i in range(len(indices) - 1):
        distance = indices[i + 1] - indices[i]  # 计算相邻索引之间的距离
        if distance < min_distance:
            min_distance = distance  # 更新最小距离
            closest_pair = (indices[i], indices[i + 1])  # 更新最小距离的索引
            closest_values = (values[i], values[i + 1])  # 更新值对

    return closest_pair, closest_values, indices



def detect_outliers_change_rate(data, threshold=1.5):
    outliers = []
    for i in range(1, len(data)):
        # 计算变化率
        change_rate = abs(data[i] - data[i-1]) / max(abs(data[i]), abs(data[i-1]), 1e-10)  # 避免除以零
        if change_rate > threshold:
            outliers.append(data[i])
    return outliers



def calculate_differences(data):
    # 确保输入是 NumPy 数组
    data = np.array(data)
    
    # 计算一阶差分
    first_diff = np.diff(data)
    first_diff=abs(first_diff)
    end=data[-1]
    start=data[0]
    
    absolute_difference = abs(start - end)
    
    # 将绝对差和一阶差分组合在一起
    first_diff_combined = np.array([absolute_difference] + list(first_diff))

    return first_diff_combined

class PoleGo:
    def __init__(self, k_rho, k_alpha, k_p, k_d, k_i, allow_reverse_parking):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        
        self.allow_reverse_parking = allow_reverse_parking
        
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    

    def get_ranges_filtered(self):
        scan = rospy.wait_for_message('/scan', LaserScan)

        
        ranges_filtered = np.array(scan.ranges)

        ranges_diff=calculate_differences(ranges_filtered)
        print(ranges_diff)

        
        ranges_filtered[ranges_filtered < 0.14] = 10.0
        
        return ranges_filtered.tolist(),ranges_diff.tolist()
    
    def detect_pole(self): 
        ranges,diff_range= self.get_ranges_filtered()
        
        max_diff_range=np.argmax(diff_range)
        
        nearest_idx2 = np.argmin(ranges)
        
        
        index,_,_=extract_and_find_closest(diff_range)
        
        # largest = np.partition(diff_range, -2)[-2:]  # 只获取两个最大值
        
        # max_value = largest[-1]  # 最大值
        # second_max_value = largest[-2]  # 次大值

        # # 找到它们的索引
        # nearest_idx = np.argmax(diff_range)
        # second_nearest_idx = np.argmax(diff_range) if max_value == second_max_value else np.argmax(diff_range[:nearest_idx] + [0] + diff_range[nearest_idx + 1:])
        
        # nearest_idx=int((nearest_idx+second_nearest_idx)/2)
        # print(index)
        nearest_idx=int((index[0]+index[1])/2)
        
        # print("*********************************************************")
        # print(nearest_idx,ranges[nearest_idx])
        # print("*********************************************************")
        # print(nearest_idx)
        
        
        alpha = (nearest_idx - 360 if nearest_idx >= 180 else nearest_idx) / 180.0 * np.pi
        return alpha, ranges[nearest_idx],ranges[nearest_idx2]
    
    def stop(self):
        self.pub_cmd_vel.publish(Twist())
    
    def run(self):
        v = 0
        omega = 0
        
        integral_v = 0
        integral_omega = 0
        
        delta_v = 0
        delta_omega = 0
        # print("OK")
        while not rospy.is_shutdown():
            try:
                alpha, rho,minggg = self.detect_pole()
                # print()
                print(minggg)
                if minggg < 0.15:
                    self.stop()
                    break   
                if self.allow_reverse_parking and abs(alpha) > np.pi / 2:
                    alpha = alpha + np.pi if alpha < 0 else alpha - np.pi
                    rho = -rho
                
                target_v = np.clip(self.k_rho * rho * (1 if abs(alpha) < np.pi / 2 else -1), -0.22, 0.22)
                target_omega = np.clip(self.k_alpha * alpha, -2.84, 2.84)
                
                error_v = target_v - v
                error_omega = target_omega - omega
                
                integral_v += error_v
                integral_omega += error_omega
                
                delta_v = self.k_p * error_v - self.k_d * delta_v + self.k_i * integral_v
                delta_omega = self.k_p * error_omega - self.k_d * delta_omega + self.k_i * integral_omega
                
                v = np.clip(v + delta_v, -0.22, 0.22)
                omega = np.clip(omega + delta_omega, -2.84, 2.84)
                
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = omega

                
                self.pub_cmd_vel.publish(twist)
                
                
                # print(">>>>>>")
            except KeyboardInterrupt:
                self.stop()
                break


def main():
    rospy.init_node('pole_Go', anonymous = True)
    

    pole_go = PoleGo(0.6, 0.8, 0.25, 0.02, 0.01, False) # Stable, not so fast

    pole_go.run()

    rospy.spin()

if __name__ == '__main__':
    main()



import re
import ast
import numpy as np
import matplotlib  
from matplotlib import pyplot as plt



class LidarData():
    #retrieves lidar data from Lidar topic. Stores newest three lists of range values in self.range_data. 
    # Also stores a single smoothed list of range values.
    def __init__(self, file_name):
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.time_increment = 0
        self.angles = list()
        self.range_data = list() #list of scans, a scan is a list of ranges, from angle_min to angle_max at angle_increment
        self.file_name = file_name

    def read_data_from_file(self):
        with open(self.file_name, "r") as data_file:
            for line in data_file:
                if re.match(r"^angle_min:", line): 
                    value = self.read_line_value(line)
                    self.angle_min = value
                if re.match(r"^angle_max:", line):
                    value = self.read_line_value(line)
                    self.angle_max = value
                if re.match(r"^angle_increment:", line):
                    value = self.read_line_value(line)
                    self.angle_increment = value
                if re.match(r"^time_increment:", line):
                    value = self.read_line_value(line)
                    self.time_increment = value
                if re.match(r"^ranges:", line):
                    self.range_data.append(self.read_line_list(line))

        self.calc_angles()          
        self.remove_inf_range_data()


    def read_line_value(self, line):
        line_data = line.split(" ")
        line_value = line_data[1:]#everything not including the name of the parameter
        if len(line_value) == 1:
           return float(line_value[0])

    def read_line_list(self, line):
        #used to read parameter with a list as its value
        line_value_str= ""
        line_value = (line.split(" "))[1:]
        for val in line_value:
            line_value_str += val
        line_value_str = line_value_str.strip()
        result = line_value_str.strip('][')
        result = result.split(",")
        return result

    def calc_angles(self):
        curr_angle = self.angle_min
        while curr_angle <= self.angle_max:
            self.angles.append(curr_angle)
            curr_angle += self.angle_increment

    def remove_inf_range_data(self):
        #removes any case of "inf" from range data.
        for iter in range(len(self.range_data)):
            for range_iter in range(len(self.range_data[iter])):
                if self.range_data[iter][range_iter] == "inf":
                    self.range_data[iter][range_iter] = 0
        



    def plot_data(self, scan_set):
        #prints data from exactly one scan
        out_str = "out_" + str(scan_set)+".png"
        theta = [self.angles]
        r = [self.range_data[scan_set]]
        print(theta)
        plt.subplot(111, projection='polar')
        plt.scatter(theta, r)
        plt.savefig(out_str)

          
my_lidar_data = LidarData("../sample_data/petes_room.txt")
my_lidar_data.read_data_from_file()
for iter in range(len(my_lidar_data.range_data)):
    my_lidar_data.plot_data(iter)
#print(my_lidar_data.range_data)

  

import math
from numpy import inf
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

# These should be made user-settable
vehicle_width_m = .60
amplification = 1.1
DEG_TO_RAD = math.pi / 180

deg_per_index = 0.333299998
rad_per_index = deg_per_index * DEG_TO_RAD
min_angle = -2.35619449615
lidar_distance_m = 25


class ProcessLidarNode(object):
    # def __init__(self):
    #     rospy.init_node('preprocess_node')
    #     lidar_sub = message_filters.Subscriber('lidar_cloud', cloud)
    #     desired_heading_sub = message_filters.Subscriber('heading', nav_heading)
    #     imu_sub = message_filters.Subscriber('imu', Imu)
    #     range_sub = message_filters.Subscriber('range' , Range)
    #     pub = rospy.Publisher('obstacle_avoid_heading', output_heading)
    #     self.ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, imu_sub, desired_heading_sub, range], 10, 0.1)
    #     self.ts.registerCallback(self.process_lidar_callback)

    # def process_lidar_callback(self, cloud, imu, desired_heading, range)
        # Tie everything together
        # filtered_cloud = self.filter_points(cloud)
        # _, _, yaw = euler_from_quaternion([imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z])
        # current_heading = yaw
        # rel_heading = desired_heading - current_heading
        # output_heading = self.update_heading(rel_heading)
        # pub.publish(output_heading)

    # def filter_points(cloud):
        # 3x3 (temporal and spatial) filtering

    def segment_points(self, cloud):
        # Separate point cloud into segments of consecutive readings
        # Stored as list of pairs (start index, stop index)
        obstacle_list = []
        start_point = 0
        stop_point = 0

        for point in range(len(cloud)):
            # if the first point in the scan has a range reading OR the current point has a reading and the previous does not
            if ((point == 0) and cloud[point]) or ((not cloud[point-1]) and cloud[point]):
                # start the segment
                start_point = point
            # if the last point in the scan has a range reading 
            if ((point == len(cloud)-1) and cloud[point]):
                #end the segment
                stop_point = point
                obstacle_list.append((start_point, stop_point))
                break
            # the current point has a reading and the next does not
            elif  (cloud[point] and (not cloud[point+1])):
                #end the segment
                stop_point = point
                obstacle_list.append((start_point, stop_point))
            # add the obstacle start-stop pair to the list
        return obstacle_list

    def convert_to_cartesian(self, cloud, index):
        # returns the specified index of the range readings as a cartesian point
        angle = min_angle + (index*rad_per_index)
        x_loc = cloud[index] * math.cos(angle)
        y_loc = cloud[index] * math.sin(angle)
        return [x_loc, y_loc]

    def split_obstacles(self, cloud, obstacle_list):
        # Split segments into smaller segments if any distance between neighboring
        # points is wide enough for the vehicle to fit.

        split_obstacle_list = []

        for obstacle in obstacle_list:
            start_point = obstacle[0]
            stop_point = obstacle[1]
            if(start_point == stop_point):
                split_obstacle_list.append((start_point, stop_point))
                continue
            for point in range(obstacle[0], obstacle[1]):
                x1, y1 = self.convert_to_cartesian(cloud, point)
                x2, y2 = self.convert_to_cartesian(cloud, point+1)
                distance = math.sqrt( pow((x1-x2),2) + pow((y1-y2),2))
                if distance > (amplification * vehicle_width_m):
                    split_obstacle_list.append((start_point, point))
                    start_point = point + 1
                if point+1 == stop_point:
                    split_obstacle_list.append((start_point, stop_point))
        return split_obstacle_list

    def merge_obstacles(self, cloud, obstacle_list):
        # Merge segments into larger segments if distance between the start(/end)
        # of one segment and the end(/start) of another segment is too narrow for
        # the vehicle to fit.

        merge_obstacle_list = []

        start_point = obstacle_list[0][0]
        stop_point = obstacle_list[0][1]

        for index in range(len(obstacle_list)-1):
            x1, y1 = self.convert_to_cartesian(cloud, obstacle_list[index][1])
            x2, y2 = self.convert_to_cartesian(cloud, obstacle_list[index+1][0])
            distance = math.sqrt(math.pow((x1-x2),2) + math.pow((y1-y2),2))
            if distance < (amplification * vehicle_width_m):
                count = 1
                span = obstacle_list[index][1] - obstacle_list[index+1][0]
                slope = (cloud[obstacle_list[index][1]] - cloud[obstacle_list[index+1][0]])/span
                for point in range(obstacle_list[index][1]+1, obstacle_list[index+1][0]):
                    cloud[point] = cloud[obstacle_list[index][1]] + count*slope
                    count+=1
                stop_point = obstacle_list[index+1][1]
            else:
                merge_obstacle_list.append((start_point, stop_point))
                start_point = obstacle_list[index+1][0]
                stop_point = obstacle_list[index+1][1]
        merge_obstacle_list.append((start_point, stop_point))
        return merge_obstacle_list

    def cluster_obstacles(self, cloud, obstacle_list):

        cluster_obstacle_list = [None] * len(obstacle_list)
        # Classify as circle, line, or rectangle
        for index in range(len(obstacle_list)):
            if (obstacle_list[index][1] - obstacle_list[index][0]) < 5:
                # Make circle
                cluster_obstacle_list[index] = circle(cloud, obstacle_list[index][0], obstacle_list[index][1])
            else: # Line or rectangle
                # calculate d_max (distance from point and line), s (distance between endpoints)
                x_p, y_p = self.convert_to_cartesian(cloud, obstacle_list[index][0]) # start point
                x_q, y_q = self.convert_to_cartesian(cloud, obstacle_list[index][1]) # end point
                s = abs(math.sqrt( math.pow((x_q - x_p), 2) + math.pow((y_q - y_p), 2) )) # distance
                d_max = 0
                for point in range(obstacle_list[index][0], obstacle_list[index][1]):
                    x_t, y_t = self.convert_to_cartesian(cloud, point) # test point
                    d = abs((y_q - y_p)*x_t - (x_q - x_p)*y_t + x_q*y_p - y_q*x_p)/math.sqrt( math.pow(y_q - y_p,2) + math.pow(x_q - x_p,2))
                    d_max = d if d > d_max else d_max
                if(d_max < 0.2 * s):
                    #Line
                    start = ProcessLidarNode().convert_to_cartesian(cloud, obstacle_list[index][0]) 
                    end  = ProcessLidarNode().convert_to_cartesian(cloud, obstacle_list[index][1])
                    cluster_obstacle_list[index] = line(cloud, start, end)
                else: # Rectangle
                    cluster_obstacle_list[index] = rectangle(cloud, obstacle_list[index][0] , obstacle_list[index][1], d_max)
        return cluster_obstacle_list

    def update_heading(self, relative_heading, obstacle_list):
        #remember to add some angle bias for clearance. Make function of distance from obstacle so buffer is constant (vehicle_width_m * amplification/2)
        for obstacle in obstacle_list:
                # Check if intersection occurs
                if obstacle.type == "Circle":
                    obstacle.adjustHeading(relative_heading)
                elif obstacle.type == "Line":
                    obstacle.adjustHeading(relative_heading, range)
                elif  obstacle.type == "Rectangle":
                    obstacle.adjustHeading(relative_heading, range)
        return relative_heading

class obstacle(object):
    def __init__(self, points):
        self.points = points

class circle(obstacle):
    def __init__(self, cloud, startIndex, endIndex):
        self.type = "Circle"
        x_p, y_p = ProcessLidarNode().convert_to_cartesian(cloud, startIndex)
        x_q, y_q = ProcessLidarNode().convert_to_cartesian(cloud, endIndex)
        if(startIndex == endIndex):
            self.center = x_p, y_p
        else:
            self.center = ( ((x_p + x_q)/2), ((y_p + y_q)/2) )
        self.radius =  0
        for point in range(startIndex, endIndex +1):
            x_t, y_t = ProcessLidarNode().convert_to_cartesian(cloud, point)
            distance = math.sqrt(math.pow((self.center[0]-x_t),2) + math.pow((self.center[1]-y_t),2))
            if distance > self.radius:
                self.radius = distance 
    #    self.radius = self.radius * (vehicle_width_m * amplification/2) buffer (not needed if we bias heading at the end)
    def adjustHeading(self, relative_heading):
        degFromPosXAxis = 90 - relative_heading
        if (relative_heading == 0): # we are headed staight forward
            distance = abs(self.center[0])
        elif(relative_heading == abs(180)): # we are headed staight backward
            # we can't see behind us so no update to heading needed
            return relative_heading
        else:
            slope = math.tan(degFromPosXAxis * DEG_TO_RAD) 
            distance = abs(slope * self.center[0] + self.center[1]) / math.sqrt(math.pow(slope,2) + 1)
            incrementLeft = degFromPosXAxis
            incrementRight = degFromPosXAxis
        while distance < self.radius:
            # we are going to intersect the obstacle
            incrementLeft += 1
            incrementRight -=1
            if(incrementLeft == 225 or incrementRight == -45):
                break
            slopeLeft = math.tan(incrementLeft * DEG_TO_RAD)
            slopeRight = math.tan(incrementRight * DEG_TO_RAD)
            distanceLeft = abs(slopeLeft * self.center[0] + self.center[1]) / math.sqrt(math.pow(slope,2) + 1) 
            distanceRight = abs(slopeRight * self.center[0] + self.center[1]) / math.sqrt(math.pow(slope,2) + 1) 
            distance = distanceLeft if (distanceLeft > distanceRight) else distanceRight
            increment  = incrementLeft if (distanceLeft > distanceRight) else incrementRight
        return increment - 90


class line(obstacle):
    def __init__(self, cloud, start, end):
        self.type = "Line"
        self.endpoint1  = start 
        self.endpoint2  = end

    def adjustHeading(self, relative_heading, range):
        robotOrigin = (0,0)
        degFromPosXAxis = 90 - relative_heading
        if(relative_heading == 0): # we are headed staight forward
            destination = (0, range)
        elif(relative_heading == abs(180)):
            # we can't see behind us so no update to heading needed
            return relative_heading
        else:
            destination_x = range* math.cos(degFromPosXAxis)
            destination_y = range * math.sin(degFromPosXAxis)
            destination = (destination_x, destination_y)
            
            denom = (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - destination[1]) - (robotOrigin[0] - destination[0])(self.endpoint2[1] - self.endpoint1[1])
            if(denom == 0):
                return relative_heading
            intersect_a = (self.endpoint1[1] - self.endpoint2[1])(robotOrigin[0] - self.endpoint1[0]) + (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - self.endpoint1[1]) / denom
            intersect_b = (robotOrigin[1] - destination[1])(robotOrigin[0] - self.endpoint1[0]) + (destination[0] - robotOrigin[0])(robotOrigin[1] - self.endpoint1[1]) / denom

            incrementLeft = degFromPosXAxis
            incrementRight = degFromPosXAxis
           
            willIntersect = True if ((intersect_a >= 0) and (intersect_a <= 1) and (intersect_b >= 0) and (intersect_b <= 1)) else False
            while(willIntersect): # we are going to intersect the obstacle
                incrementLeft += 1
                incrementRight -=1
                if(incrementLeft == 225 or incrementRight == -45):
                    break
                destination_x_left = range * math.cos(incrementLeft)
                destination_y_left = range * math.sin(incrementLeft)
                destination_left = (destination_x_left, destination_y_left)
                
                destination_x_right = range * math.cos(incrementRight)
                destination_y_right = range * math.sin(incrementRight)
                destination_right = (destination_x_right, destination_y_right)
                
                denom_left = (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - destination_left[1]) - (robotOrigin[0] - destination_left[0])(self.endpoint2[1] - self.endpoint1[1])
                denom_right = (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - destination_right[1]) - (robotOrigin[0] - destination_right[0])(self.endpoint2[1] - self.endpoint1[1])
                
                if(denom == 0):
                   return relative_heading
                intersect_a_left = (self.endpoint1[1] - self.endpoint2[1])(robotOrigin[0] - self.endpoint1[0]) + (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - self.endpoint1[1]) / denom_left
                intersect_b_left = (robotOrigin[1] - destination_left[1])(robotOrigin[0] - self.endpoint1[0]) + (destination_left[0] - robotOrigin[0])(robotOrigin[1] - self.endpoint1[1]) / denom_left

                intersect_a_right = (self.endpoint1[1] - self.endpoint2[1])(robotOrigin[0] - self.endpoint1[0]) + (self.endpoint2[0] - self.endpoint1[0])(robotOrigin[1] - self.endpoint1[1]) / denom_right
                intersect_b_right = (robotOrigin[1] - destination_right[1])(robotOrigin[0] - self.endpoint1[0]) + (destination_right[0] - robotOrigin[0])(robotOrigin[1] - self.endpoint1[1]) / denom_right

                willIntersect_left = True if ((intersect_a_left >= 0) and (intersect_a_left <= 1) and (intersect_b_left >= 0) and (intersect_b_left <= 1)) else False
                willIntersect_right = True if ((intersect_a_right >= 0) and (intersect_a_right <= 1) and (intersect_b_right >= 0) and (intersect_b_right <= 1)) else False

                if(~willIntersect_left):
                   return incrementLeft - 90
                elif(~willIntersect_right):
                   return incrementRight - 90

class rectangle(obstacle):
    def __init__(self, cloud, startIndex, endIndex, d_max):
        self.type = "Rectangle"
        angleStart = min_angle + (startIndex*deg_per_index)
        angleEnd = min_angle + (endIndex*deg_per_index) 
        self.point1 = ProcessLidarNode().convert_to_cartesian(cloud, startIndex)
        self.point2 = ProcessLidarNode().convert_to_cartesian(cloud, endIndex)
        self.point3 = ProcessLidarNode().convert_to_cartesian(cloud, startIndex)
        self.point4 = ProcessLidarNode().convert_to_cartesian(cloud, endIndex)
        
        self.point1[0] +- (d_max/2) * math.cos(angleStart)
        self.point1[1] += (d_max/2) * math.sin(angleStart)
        self.point3[0] -= (d_max/2) * math.cos(angleStart)
        self.point3[1] -= (d_max/2) * math.sin(angleStart) 

        
        self.point2[0] += (d_max/2) * math.cos(angleEnd)
        self.point2[1] += (d_max/2) * math.sin(angleEnd)
        self.point4[0] -= (d_max/2) * math.cos(angleEnd)
        self.point4[1] -= (d_max/2) * math.sin(angleEnd)

        self.line1 = line(cloud, self.point1, self.point4)
        self.line2 = line(cloud, self.point2, self.point4)
        self.line3 = line(cloud, self.point1, self.point3)
        self.line4 = line(cloud, self.point2, self.point3)

        self.lines = self.line1, self.line2, self.line3, self.line4

    def adjustHeading(self, relative_heading, range):
        for line in self.lines:
            relative_heading = line.adjustHeading(relative_heading, range)
        return relative_heading

# Tests the obstacle classification functions with a test scan
def plotTest():
    myNode = ProcessLidarNode()
    cloud = [2.48799991607666, 2.4700000286102295, 2.450000047683716, 2.421999931335449, 2.4070000648498535, 2.3989999294281006, 2.382999897003174, 2.365000009536743, 2.3420000076293945, 2.328000068664551, 2.303999900817871, 2.2960000038146973, 2.2760000228881836, 2.25600004196167, 2.246000051498413, 2.2300000190734863, 2.2230000495910645, 2.2119998931884766, 2.196000099182129, 2.187000036239624, 2.171999931335449, 2.171999931335449, 2.1659998893737793, 2.1600000858306885, 2.1600000858306885, 2.1459999084472656, 2.1410000324249268, 2.2019999027252197, 1.0130000114440918, 1.031000018119812, 1.0379999876022339, 1.0240000486373901, 1.0260000228881836, 1.0199999809265137, 1.0160000324249268, 1.0080000162124634, 1.0019999742507935, 0.9929999709129333, 0.9940000176429749, 0.9929999709129333, 0.9850000143051147, 0.9860000014305115, 0.9919999837875366, 0.9829999804496765, 0.972000002861023, 0.9649999737739563, 0.9649999737739563, 0.9660000205039978, 0.9639999866485596, 0.9700000286102295, 0.9570000171661377, 0.9570000171661377, 0.9509999752044678, 0.9520000219345093, 0.9430000185966492, 0.9589999914169312, 0.9459999799728394, 0.9419999718666077, 0.9399999976158142, 0.9330000281333923, 0.9399999976158142, 0.9369999766349792, 0.921999990940094, 0.9240000247955322, 0.9290000200271606, 0.9229999780654907, 0.9210000038146973, 0.9150000214576721, 0.9150000214576721, 0.9049999713897705, 0.9129999876022339, 0.9089999794960022, 0.9039999842643738, 0.902999997138977, 0.9010000228881836, 0.9020000100135803, 0.8999999761581421, 0.902999997138977, 0.8939999938011169, 0.9010000228881836, 0.8949999809265137, 0.890999972820282, 0.890999972820282, 0.8920000195503235, 0.890999972820282, 0.8759999871253967, 0.8769999742507935, 0.8870000243186951, 0.8759999871253967, 0.8790000081062317, 0.8769999742507935, 0.8870000243186951, 0.8759999871253967, 0.8669999837875366, 0.8679999709129333, 0.8650000095367432, 0.875, 0.8650000095367432, 0.8730000257492065, 0.8690000176429749, 0.8659999966621399, 0.8650000095367432, 0.8600000143051147, 0.8600000143051147, 0.8560000061988831, 0.8700000047683716, 0.8629999756813049, 0.8619999885559082, 0.8550000190734863, 0.8619999885559082, 0.8539999723434448, 0.8009999990463257, 0.7009999752044678, 0.6800000071525574, 0.6840000152587891, 0.6800000071525574, 0.6710000038146973, 0.6859999895095825, 0.6869999766349792, 0.6759999990463257, 0.675000011920929, 0.6740000247955322, 0.675000011920929, 0.6759999990463257, 0.6769999861717224, 0.6840000152587891, 0.6779999732971191, 0.6779999732971191, 0.6800000071525574, 0.6830000281333923, 0.6819999814033508, 0.6819999814033508, 0.6690000295639038, 0.6919999718666077, 0.6869999766349792, 0.6899999976158142, 0.6869999766349792, 0.6840000152587891, 0.6859999895095825, 0.6859999895095825, 0.6740000247955322, 0.6769999861717224, 0.6809999942779541, 0.6869999766349792, 0.6830000281333923, 0.6940000057220459, 0.6830000281333923, 0.6909999847412109, 0.6859999895095825, 0.6779999732971191, 0.6830000281333923, 0.6819999814033508, 0.6869999766349792, 0.6840000152587891, 0.6869999766349792, 0.6759999990463257, 0.6980000138282776, 0.6840000152587891, 0.6880000233650208, 0.6890000104904175, 0.6919999718666077, 0.699999988079071, 0.6990000009536743, 0.6919999718666077, 0.6869999766349792, 0.6990000009536743, 0.7009999752044678, 0.6980000138282776, 0.7020000219345093, 0.699999988079071, 0.6980000138282776, 0.6990000009536743, 0.6980000138282776, 0.7049999833106995, 0.7089999914169312, 0.7020000219345093, 0.7049999833106995, 0.7170000076293945, 0.7070000171661377, 0.7020000219345093, 0.7139999866485596, 0.7080000042915344, 0.7170000076293945, 0.7139999866485596, 0.7139999866485596, 0.7120000123977661, 0.7279999852180481, 0.7250000238418579, 0.722000002861023, 0.7279999852180481, 0.7229999899864197, 0.7210000157356262, 0.7229999899864197, 0.7279999852180481, 0.734000027179718, 0.7379999756813049, 0.7400000095367432, 0.7409999966621399, 0.7390000224113464, 0.7409999966621399, 0.7360000014305115, 0.7429999709129333, 0.75, 0.7540000081062317, 0.753000020980835, 0.7549999952316284, 0.746999979019165, 0.7639999985694885, 0.7639999985694885, 0.7680000066757202, 0.765999972820282, 0.7609999775886536, 0.7699999809265137, 0.7720000147819519, 0.7710000276565552, 0.7689999938011169, 0.7739999890327454, 0.7770000100135803, 0.7870000004768372, 0.7870000004768372, 0.7879999876022339, 0.7870000004768372, 0.8149999976158142, 0.9279999732971191, 0.9909999966621399, 0.9900000095367432, 0.9980000257492065, 1.003000020980835, 1.0099999904632568, 1.0099999904632568, 1.0110000371932983, 1.0180000066757202, 1.0219999551773071, 1.0230000019073486, 1.0299999713897705, 1.0379999876022339, 1.0019999742507935, 2.6730000972747803, 2.687000036239624, 2.7019999027252197, 2.7219998836517334, 2.744999885559082, 2.7860000133514404, 2.7839999198913574, 2.48799991607666, 2.3480000495910645, 2.688999891281128, 2.8589999675750732, 2.878999948501587, 2.885999917984009, 2.9189999103546143, 2.940000057220459, 2.9579999446868896, 2.9800000190734863, 2.9739999771118164, 2.9539999961853027, 2.890000104904175, 2.9749999046325684, 3.0169999599456787, 3.0439999103546143, 2.937000036239624, 2.8329999446868896, 2.821000099182129, 2.816999912261963, 2.811000108718872, 2.809999942779541, 2.812000036239624, 2.9690001010894775, 3.055999994277954, 6.198999881744385, 4.109000205993652, inf, 4.422999858856201, 4.502999782562256, 3.5169999599456787, 3.5969998836517334, 3.694999933242798, 3.700000047683716, 5.160999774932861, 5.179999828338623, inf, 4.24399995803833, 4.224999904632568, 4.386000156402588, 4.2779998779296875, 4.35099983215332, 4.629000186920166, 4.258999824523926, inf, 4.547999858856201, 4.461999893188477, 4.421999931335449, 5.895999908447266, 5.9039998054504395, inf, 6.585999965667725, 6.380000114440918, 6.464000225067139, 6.732999801635742, 5.008999824523926, 5.0289998054504395, 5.426000118255615, 5.593999862670898, 5.670000076293945, 8.003999710083008, 8.116999626159668, 7.984000205993652, inf, 8.206000328063965, inf, inf, inf, inf, inf, inf, inf, 8.072999954223633, inf, inf, inf, inf, 6.752999782562256, 6.783999919891357, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 9.319999694824219, 9.359000205993652, 9.347000122070312, 9.343999862670898, 9.322999954223633, 9.32800006866455, 9.324999809265137, 9.321000099182129, 9.267999649047852, 9.3100004196167, 9.302000045776367, 9.295000076293945, 9.295000076293945, 9.284000396728516, 9.298999786376953, 9.289999961853027, 9.276000022888184, 9.277000427246094, 9.279999732971191, 9.279999732971191, 9.28600025177002, 9.281999588012695, 9.279999732971191, 9.263999938964844, 9.27299976348877, 9.27299976348877, 9.270000457763672, 9.269000053405762, 9.269000053405762, 9.27299976348877, 9.281000137329102, 9.284000396728516, 9.28499984741211, 9.281000137329102, 9.293999671936035, 9.305000305175781, 9.284000396728516, 9.303999900817871, 9.288000106811523, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 16.79599952697754, inf, inf, 16.44700050354004, inf, inf, 16.05500030517578, 15.979999542236328, inf, inf, 15.699000358581543, 15.564000129699707, 15.496000289916992, 15.385000228881836, inf, 0.9940000176429749, 0.9909999966621399, 0.9929999709129333, 0.9729999899864197, 0.9800000190734863, 0.9779999852180481, 0.9589999914169312, 0.9559999704360962, 0.9520000219345093, 0.9399999976158142, 0.9369999766349792, 0.9449999928474426, 0.9319999814033508, 0.9269999861717224, 0.9309999942779541, 0.921999990940094, 0.9139999747276306, 0.9169999957084656, 0.9049999713897705, 0.8939999938011169, 0.8980000019073486, 0.8859999775886536, 0.8899999856948853, 0.8849999904632568, 0.8690000176429749, 0.7689999938011169, 0.6949999928474426, 0.6690000295639038, 0.6660000085830688, 0.6660000085830688, 0.6700000166893005, 0.6600000262260437, 0.656000018119812, 0.6499999761581421, 0.652999997138977, 0.6460000276565552, 0.6460000276565552, 0.6470000147819519, 0.6439999938011169, 0.6510000228881836, 0.6359999775886536, 0.6420000195503235, 0.6269999742507935, 0.628000020980835, 0.6330000162124634, 0.6230000257492065, 0.6230000257492065, 0.6190000176429749, 0.6230000257492065, 0.621999979019165, 0.621999979019165, 0.6169999837875366, 0.6100000143051147, 0.6129999756813049, 0.6019999980926514, 0.6100000143051147, 0.609000027179718, 0.597000002861023, 0.5910000205039978, 0.5960000157356262, 0.5950000286102295, 0.6000000238418579, 0.6050000190734863, 0.5960000157356262, 0.5870000123977661, 0.5920000076293945, 0.5860000252723694, 0.5899999737739563, 0.5849999785423279, 0.5809999704360962, 0.5849999785423279, 0.5820000171661377, 0.5799999833106995, 0.5759999752044678, 0.5699999928474426, 0.5709999799728394, 0.5759999752044678, 0.574999988079071, 0.574999988079071, 0.5730000138282776, 0.5680000185966492, 0.5630000233650208, 0.5680000185966492, 0.5559999942779541, 0.5640000104904175, 0.5619999766349792, 0.5550000071525574, 0.5559999942779541, 0.5609999895095825, 0.5559999942779541, 0.5680000185966492, 0.5609999895095825, 0.5580000281333923, 0.5529999732971191, 0.5590000152587891, 0.5490000247955322, 0.5490000247955322, 0.5429999828338623, 0.5490000247955322, 0.546999990940094, 0.550000011920929, 0.5519999861717224, 0.5540000200271606, 0.5490000247955322, 0.5460000038146973, 0.5479999780654907, 0.550000011920929, 0.5479999780654907, 0.5360000133514404, 0.5460000038146973, 0.5440000295639038, 0.5410000085830688, 0.5410000085830688, 0.5429999828338623, 0.5339999794960022, 0.5360000133514404, 0.5379999876022339, 0.5440000295639038, 0.5400000214576721, 0.5339999794960022, 0.5400000214576721, 0.5419999957084656, 0.5379999876022339, 0.5379999876022339, 0.5379999876022339, 0.5370000004768372, 0.5370000004768372, 0.5350000262260437, 0.5389999747276306, 0.5410000085830688, 0.5260000228881836, 0.5329999923706055, 0.5379999876022339, 0.5360000133514404, 0.5360000133514404, 0.5299999713897705, 0.5320000052452087, 0.5329999923706055, 0.5400000214576721, 0.5379999876022339, 0.5400000214576721, 0.5400000214576721, 0.5339999794960022, 0.5400000214576721, 0.5329999923706055, 0.5320000052452087, 0.5370000004768372, 0.5389999747276306, 0.531000018119812, 0.5320000052452087, 0.5350000262260437, 0.5320000052452087, 0.5339999794960022, 0.5379999876022339, 0.531000018119812, 0.531000018119812, 0.5370000004768372, 0.5370000004768372, 0.5440000295639038, 0.5370000004768372, 0.5289999842643738, 0.5320000052452087, 0.5400000214576721, 0.5350000262260437, 0.5419999957084656, 0.5360000133514404, 0.5410000085830688, 0.5360000133514404, 0.5360000133514404, 0.5389999747276306, 0.5379999876022339, 0.5400000214576721, 0.5619999766349792, 0.6349999904632568, 0.7670000195503235, 0.8069999814033508, 0.8019999861717224, 0.8159999847412109, 0.8080000281333923, 0.8159999847412109, 0.8119999766349792, 0.8109999895095825, 0.8180000185966492, 0.8180000185966492, 0.8080000281333923, 0.8140000104904175, 0.8169999718666077, 0.8220000267028809, 0.8149999976158142, 0.828000009059906, 0.8159999847412109, 0.8270000219345093, 0.8339999914169312, 0.8230000138282776, 0.8289999961853027, 0.828000009059906, 0.8220000267028809, 0.8379999995231628, 0.8370000123977661, 0.8330000042915344, 0.8270000219345093, 0.8389999866485596, 0.8420000076293945, 0.8399999737739563, 0.8360000252723694, 0.8399999737739563, 0.8420000076293945, 0.8420000076293945, 0.8489999771118164, 0.8519999980926514, 0.8600000143051147, 0.8610000014305115, 0.8700000047683716, 0.8600000143051147, 0.875, 0.8650000095367432, 0.8659999966621399, 0.8679999709129333, 0.8709999918937683, 0.8820000290870667, 0.8809999823570251, 0.8759999871253967, 0.875, 0.8889999985694885, 0.8859999775886536, 0.8899999856948853, 0.8989999890327454, 0.8999999761581421, 0.9049999713897705, 0.9010000228881836, 0.902999997138977, 0.9110000133514404, 0.906000018119812, 0.9129999876022339, 0.9139999747276306, 0.9210000038146973, 0.9229999780654907, 0.9309999942779541, 0.9290000200271606, 0.9340000152587891, 0.9309999942779541, 0.9380000233650208, 0.9449999928474426, 0.9470000267028809, 0.9430000185966492, 0.9580000042915344, 0.9520000219345093, 0.953000009059906, 0.9729999899864197, 0.9649999737739563, 0.9700000286102295, 0.9710000157356262, 0.9819999933242798, 0.9950000047683716, 0.9890000224113464, 0.9959999918937683, 0.9929999709129333, 0.9990000128746033, 1.0099999904632568, 1.0110000371932983, 1.0160000324249268, 1.0260000228881836, 1.0299999713897705, 1.0290000438690186, 1.0390000343322754, 1.0479999780654907, 1.0549999475479126, 1.0499999523162842, 1.062999963760376, 1.059000015258789, 1.0679999589920044, 1.0800000429153442, 1.0859999656677246, 1.0889999866485596, 1.0959999561309814, 1.0889999866485596, 1.1080000400543213, 1.1059999465942383, 1.11899995803833, 1.11899995803833]
    theta = -2.35619449615
    for point in range(len(cloud)):
        if cloud[point] == inf:
            cloud[point] = 0
        x, y = myNode.convert_to_cartesian(cloud, point)
        plt.plot(x, y, 'ro', markersize=1)
        theta += deg_per_index * (DEG_TO_RAD)


    obstacle_list = myNode.segment_points(cloud)
    obstacle_list = myNode.split_obstacles(cloud, obstacle_list)
    obstacle_list = myNode.merge_obstacles(cloud, obstacle_list)
    obstacle_list = myNode.cluster_obstacles(cloud, obstacle_list)

    lines = []
    for obstacle in obstacle_list:
        if obstacle.type == "Circle":
            circle = plt.Circle((obstacle.center[0], obstacle.center[1]), obstacle.radius, fill=False)
            plt.gcf().gca().add_artist(circle)
        if obstacle.type == "Line":
            lines.append((obstacle.endpoint1, obstacle.endpoint2))
        if obstacle.type == "Rectangle":
            for line in obstacle.lines:
                lines.append((line.endpoint1, line.endpoint2))
    lc = mc.LineCollection(lines)
    plt.gcf().gca().add_collection(lc)
    plt.show()

if __name__ == "__main__":
	plotTest()
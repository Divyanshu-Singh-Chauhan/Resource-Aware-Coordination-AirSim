#!/usr/bin/env python
import rospy
from multi_target_tracking.msg import PursuerEvaderDataArray
from std_msgs.msg import Header
import csv
import os
import argparse
import time

class PursuerEvaderDataProcessor:
    def __init__(self, experiment_number, algorithm, replanning_frequency, evader_flying_formation, logging_frequency):
        rospy.init_node("pursuer_evader_data_processor_node")

        self.experiment_number = experiment_number
        self.algorithm = algorithm
        self.replanning_frequency = replanning_frequency
        self.evader_flying_formation = evader_flying_formation
        self.total_min_distances = []
        self.logging_frequency = logging_frequency
        self.first_message_time = None  # Use time module for first message time

        # self.output_csv_filename = f"output_{experiment_number}_{algorithm}_{replanning_frequency}.csv"
        # self.pe_data_sub = rospy.Subscriber('/pursuer_evader_data', PursuerEvaderDataArray, self.pe_data_callback)
        save_dir = "/home/sgari/sandilya_ws/unity_airsim_workspace/experiments_data_mtt/"
        self.output_csv_filename = save_dir + "output_{}_{}_{}_{}.csv".format(experiment_number, algorithm, replanning_frequency, evader_flying_formation)
        self.pe_data_sub = rospy.Subscriber('/pursuer_evader_data', PursuerEvaderDataArray, self.pe_data_callback)

    def pe_data_callback(self, msg):
        if self.first_message_time is None:
            self.first_message_time = time.time()

        # Calculate time difference in seconds
        timestamp = time.time() - self.first_message_time

        evader_ids_vec = msg.pursuer_evader_info_array[0].evaders_range_bearing_ids  # contains ids of all evaders
        evader_ids_mindist = {evader_id: float('inf') for evader_id in evader_ids_vec}

        checked_all_fovs_flag = False
        all_evaders_in_fov = set()

        for evader_id in evader_ids_vec:
            # for j in range(len(msg.pursuer_evader_info_array[0].pursuers_range_bearing_ids)):
                # note pursuer ID will be j+1
            for pursuer_data in msg.pursuer_evader_info_array:
                pe_rb_ids = pursuer_data.evaders_range_bearing_ids
                pe_rb_vals = pursuer_data.evaders_range_bearing_values
                num_pursuers = len(msg.pursuer_evader_info_array)
                evader_range_ind = (evader_id - num_pursuers)-1
                evader_to_pursuer_range = pursuer_data.evaders_range_bearing_values[2*evader_range_ind]

                if (evader_to_pursuer_range < evader_ids_mindist[evader_id]):
                    evader_ids_mindist[evader_id] =  evader_to_pursuer_range

                if (checked_all_fovs_flag ==  False):
                    for evader_id in pursuer_data.evaders_in_FOV:
                        all_evaders_in_fov.add(evader_id)
            
            checked_all_fovs_flag=True

        # sum all values of the evaders dict
        total_min_dist = sum(evader_ids_mindist.values())

        # get total number of unique evaders in collective FOV
        num_unique_evaders_in_fov = len(all_evaders_in_fov)

        # Save the result to the list
        # self.total_min_distances.append((timestamp, total_min_dist))
        self.total_min_distances.append((timestamp, total_min_dist, num_unique_evaders_in_fov))


    # def save_to_csv(self):
    #     with open(self.output_csv_filename, mode='w') as csv_file:
    #         fieldnames = ['Timestamp', 'Total_Min_Distance']
    #         writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    #         writer.writeheader()
    #         for timestamp, total_min_distance in self.total_min_distances:
    #             writer.writerow({'Timestamp': timestamp.to_sec(), 'Total_Min_Distance': total_min_distance})


    def save_to_csv(self):
        with open(self.output_csv_filename, mode='w') as csv_file:
            writer = csv.writer(csv_file)
            for timestamp, total_min_distance, num_evaders_in_fov in self.total_min_distances:
                writer.writerow([timestamp, total_min_distance, num_evaders_in_fov])
            # for timestamp, total_min_distance in self.total_min_distances:
            #     writer.writerow([timestamp, total_min_distance])

        print("Results saved to {}".format(self.output_csv_filename))

    def shutdown_callback(self):
        self.save_to_csv() 

if __name__ == '__main__':
    # NOTE! flying formation must be: L for pure linear motion (no noise), S for stochastic, and H for stochastic herd
    # python2 total_min_distance_logger.py <experiment_number> <algorithm> <replanning_frequency> <evader_formation> <logging_frequency>
    parser = argparse.ArgumentParser(description='Pursuer Evader Data Processor')
    parser.add_argument('experiment_number', type=str, help='Experiment number')
    parser.add_argument('algorithm', type=str, help='Algorithm')
    parser.add_argument('replanning_frequency', type=str, help='Replanning frequency')
    parser.add_argument('evader_formation', type=str, help='Evader flying formation')
    parser.add_argument('logging_frequency', type=str, help='logging frequency')

    args = parser.parse_args()
    processor = PursuerEvaderDataProcessor(args.experiment_number, args.algorithm, args.replanning_frequency, args.evader_formation, args.logging_frequency)

    # Manual input for testing
    # experiment_number = 'test_experiment'
    # algorithm = 'test_algorithm'
    # replanning_frequency = 'test_frequency'
    # evader_formation = 'herd'
    # logging_frequency = '10'
    # processor = PursuerEvaderDataProcessor(experiment_number, algorithm, replanning_frequency, evader_formation, logging_frequency)

    rate = rospy.Rate(10)  # Hz
    # if processor.logging_frequency
    # rate = rospy.Rate(float(processor.logging_frequency))
    rospy.on_shutdown(processor.shutdown_callback)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

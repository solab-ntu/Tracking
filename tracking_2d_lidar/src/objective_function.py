import numpy as np
from run_launch_file import launch
import csv
import yaml
import pickle
import time

# global variable
max_dist = 0.4

def set_params(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold):  # set param by changing the yaml file
    file = '/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/src/setting.yaml'
    
    with open(file) as f:
        yml = yaml.load(f)

    yml['EGBIS']['k'] = float(k)
    yml['EGBIS']['min_size'] = int(min_size)
    yml['ICP']['icp_max_dist'] = float(icp_max_dist)
    yml['CheckStatic']['inflation_size'] = int(inflation_size)
    yml['CheckStatic']['static_threshold'] = float(static_threshold)
    yml['CheckStatic']['speed_threshold'] = float(speed_threshold)

    with open(file, 'w') as f:
        yaml.dump(yml, f)


def objective_func(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold):
    global max_dist

    # set params
    set_params(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold)

    # roslaunch
    launch_filename = '/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/launch/tracking_bagfile.launch'
    launch(launch_filename) 

    # read test (tracking) and label (ground true)
    with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/outputs/history.csv') as testfile:
        with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/outputs/label_ok.csv') as labelfile:

            test_rows = csv.reader(testfile)
            label_rows = csv.reader(labelfile)

            x_test = []
            y_test = []
            x_label = []
            y_label = []

            true_positive = 0
            false_positive = 0
            false_negative = 0

            for i, (test_row, label_row) in enumerate(zip(test_rows, label_rows)):
                if i%2 == 0:  # collect x (in a row)
                    x_test = [float(a) for a in test_row]
                    x_label = [float(a) for a in label_row]
                else:  # collect y (in the next row of x)
                    y_test = [float(a) for a in test_row]
                    y_label = [float(a) for a in label_row]

                    # ------ now at i/2 th frame. main work here: ------

                    # in process_kf, if at a time frame there is no tracks, it will record (1000,1000)
                    fake_history = 0
                    for x, y in zip(x_test, y_test):
                        if x==1000 and y==1000:
                            fake_history += 1

                    # count origin (0,0) clicks
                    origin_click_count = 0
                    for x_l, y_l in zip(x_label, y_label):
                        if ((0.0-x_l)**2 + (0.0-y_l)**2) < 0.15**2: 
                            origin_click_count += 1

                    label_matched = [0] * len(x_label)
                    # use collected x and y to match 
                    for x, y in zip(x_test, y_test):
                        matched = False
                        for i_label, (x_l, y_l) in enumerate(zip(x_label, y_label)):
                            # true positive
                            if ((x-x_l)**2 + (y-y_l)**2) < max_dist**2:
                                if label_matched[i_label] == 0:  # this label hasn't been matched
                                    true_positive += 1
                                    label_matched[i_label] = 1
                                matched = True
                                #break  # comment to allow one test match multiple labels
                        # false positive
                        if matched == False:
                            false_positive += 1
                    
                    # those left in label unmatched are false negative
                    false_negative += (len(x_label) - label_matched.count(1) - origin_click_count)
                    
                    false_positive -= fake_history
                    # ------ now at i/2 th frame. main work finished here ------
                    

            print('TP=%d, FP=%d, FN=%d' % (true_positive, false_positive, false_negative))

            # precision, recall, f1 score
            precision = true_positive/float(true_positive+false_positive)
            recall = true_positive/float(true_positive+false_negative)
            f1 = 2.0*precision*recall/(precision+recall)

            # write to pickle file
            time_ = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
            with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/f1s/'+time_+'.pkl', 'wb') as f:
                pickle.dump([precision, recall, f1], f)

            print('Precision=%f, Recall=%f, F1=%f' % (precision, recall, f1))
            
    return f1
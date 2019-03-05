# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqStartTime = [600, 0, 900]
SeqDuration = [3000, 1200, 3000]
SeqNameList = ['2014-07-14-15-16-36', '2014-11-18-13-20-12', '2014-12-05-11-09-10'];

Result_root = '/mnt/DATA/tmp/RobotCar/vins_Mono_Baseline/'

Number_GF_List = [150, 200, 400, 600, 800]; 

Num_Repeating = 10 # 20 #  5 # 
SleepTime = 5

config_prefix = '/home/yipuzhao/vins_ws/src/VINS-Fusion/config/robotcar/bumblebee-stereo_config'

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

subprocess.call('rosparam set use_sim_time true', shell=True)

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] #+ '_blur_9'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_rosbag  = '/mnt/DATA/Datasets/Oxford_Robotcar/' + SeqName + '/stereo_rect_rescale.bag'
            Config_Yaml = config_prefix + '_lmk' + str(num_gf) + '.yaml'

            cmd_vinsrun   = str('rosrun vins vins_node ' + Config_Yaml)
            cmd_looprun   = str('rosrun loop_fusion loop_fusion_node ' + Config_Yaml)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sn]) + ' -u ' + str(SeqDuration[sn])
            # 
            cmd_timelog = str('cp /mnt/DATA/tmpLog.txt ' + Experiment_dir + '/' + SeqName + '_Log.txt')
            cmd_vinslog = str('cp /mnt/DATA/vio.csv ' + Experiment_dir + '/' + SeqName + '_AllFrameTrajectory_noLC.txt')
            cmd_looplog = str('cp /mnt/DATA/vio_loop.csv ' + Experiment_dir + '/' + SeqName + '_AllFrameTrajectory.txt')

            print bcolors.WARNING + "cmd_vinsrun: \n"   + cmd_vinsrun   + bcolors.ENDC
            print bcolors.WARNING + "cmd_looprun: \n"   + cmd_looprun   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC
            print bcolors.WARNING + "cmd_timelog: \n" + cmd_timelog + bcolors.ENDC
            print bcolors.WARNING + "cmd_vinslog: \n" + cmd_vinslog + bcolors.ENDC
            print bcolors.WARNING + "cmd_looplog: \n" + cmd_looplog + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_vins = subprocess.Popen(cmd_vinsrun, shell=True)
            proc_loop = subprocess.Popen(cmd_looprun, shell=True)
            # proc_slam = subprocess.Popen("exec " + cmd_slam, stdout=subprocess.PIPE, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for vins_estimator init" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill /loop_fusion', shell=True)
            subprocess.call('rosnode kill /vins_estimator', shell=True)
            # subprocess.call('pkill roslaunch', shell=True)
            # subprocess.call('pkill svo_node', shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for vins_estimator to quit" + bcolors.ENDC
            time.sleep(SleepTime)
            print bcolors.OKGREEN + "Copy the time log to result folder" + bcolors.ENDC
            subprocess.call(cmd_timelog, shell=True)
            print bcolors.OKGREEN + "Copy the local optim. track to result folder" + bcolors.ENDC
            subprocess.call(cmd_vinslog, shell=True)
            print bcolors.OKGREEN + "Copy the global optim. track to result folder" + bcolors.ENDC
            subprocess.call(cmd_looplog, shell=True)
            # proc_rec.terminate()
            # outs, errs = proc_rec.communicate()
            # proc_slam.kill()

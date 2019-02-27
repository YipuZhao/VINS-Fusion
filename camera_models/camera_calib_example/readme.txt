# help for checking input parameters.
rosrun camera_models Calibrations --help

# example pinhole model.
rosrun camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model pinhole

# example mei model.
rosrun camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model mei

rosrun camera_models Calibrations -w 8 -h 6 -s 106 -i hololens -p left- --camera-model pinhole --camera-name hololens-left

rosrun camera_models Calibrations -w 8 -h 6 -s 106 -i hololens -p right- --camera-model pinhole --camera-name hololens-right
# 评价每段距离的误差， 每隔100米统计一下
evo_rpe kitti gnss_odometry.txt laser_odometry.txt -r trans_part --delta 100 --plot --plot_mode xyz

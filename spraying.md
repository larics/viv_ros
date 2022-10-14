# ViV spraying commands

<img src="doc/viv_driving_1_transparent.png" width="800">

## First terminal (ViV)
ssh

    ssh viv@VIV.local

    killall -9 rosmaster

    roslaunch viv_base viv_base.launch

    roslaunch hektor_viv_description load_viv_kinova_kortex_driver_clone.launch spraying_frame_distance:=0.35

    rosrun lawnmower_control test_real_robot __ns:="my_gen3" _spraying_frame_distance:=0.35
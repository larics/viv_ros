# ViV useful commands

![viv1](doc/viv_pequeno_img.png)

## First terminal (ViV)
ssh

    ssh viv@VIV.local

to restart viv_base

    killall -9 rosmaster

    roslaunch viv_base viv_base.launch

Now you should be able to drive around with a joystick

If viv_base breaks down check the can connection:

    ip -s -c -h a

See if can0 is UP or DOWN 

If its UP run:

    sudo ip link set can0 down 

If its DOWN, run:

    sudo ip link set can0 up type can bitrate 125000  

## Second terminal (local)
Remote master

    export ROS_MASTER_URI=http://VIV.local:11311


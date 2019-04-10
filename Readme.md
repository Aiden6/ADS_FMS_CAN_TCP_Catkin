##### PRE INSTALL 
```
sudo apt-get install libusb-dev
sudo apt-get install libusb-1.0-0-dev
```
##### copy file to your /usr/lib和、usr/lib64
```
libusbcan.so libusbcan.so.1
```

#将路径加入到root超级用户中
1.sudo su
2.echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
3.source ~/.bashrc
#开一个终端
1.sudo su
2.roscore 
#重开一个终端
1.catkin_make clean
2.catkin_make
3.sudo su
4.source ./devel/setup.bash
5.rosrun ads_fms ads_fms



Ctrl + c

Ctrl + z

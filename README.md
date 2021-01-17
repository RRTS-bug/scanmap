# scanmap ![CI status]
(https://img.shields.io/badge/bulid-python-ros-brightgreen.svg)

scanmap 是通过2D激光数据和里程计信息进行栅格建图的工具包

## Installation - 安装

### Requirements - 必要条件

* Linux
* python 3.0 and up

## Usage -用法

'''
$ git clone https://github.com/RRTS-bug/scanmap.git
$ cd scanmap
$ catkin_make
$ source devel/setup.bash
$ cd src
$ roslaunch scan scanRun.launch
'''
通过launch文件修改参数和话题，建图方法详见docx文件

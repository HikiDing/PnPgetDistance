
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    # 创建Actions.Node对象li_node，标明李四所在位置
    paramter = Node(
        package="paramtest",
        executable="main_node",
        parameters=[
            {'cameraArray': [1262.748030 , 0.000000 , 315.770278,
		                     0.000000 , 1259.194056 , 226.760690,
		                     0.000000 , 0.000000    , 1.000000  ]},
            {'distArray': [-0.421204,-0.237514,-0.003354,-0.001293,0.000000]}
            ],
            output='screen',
            emulate_tty=True
        )
    pubNode = Node(
        package="test",
        executable="p1_node",
        output='screen'
    )
    launch_description = LaunchDescription([paramter,pubNode])
    # 返回让ROS2根据launch描述执行节点
    return launch_description



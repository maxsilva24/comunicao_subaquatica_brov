from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    comunicacao_node = Node( package="comunicacao_brov",
                            executable="no_comunicacao_brov",
                            parameters=[
                                {'coordenadada_origem':[2,3,5]},
                                {'coordenadada_destino':[2,3,5]}
                            ],
                            output='screen',
    )
    #
    ld.add_action(comunicacao_node)
    #
    return ld
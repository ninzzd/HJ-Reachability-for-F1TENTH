from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rt_reachability',
            executable='sdf_node',
            name='cuda_debugger',
            prefix='bash -c "cuda-gdb --args -ex run"',  # For CUDA debugging
            output='screen',
            parameters=[{'use_cuda': True}]
        )
    ])
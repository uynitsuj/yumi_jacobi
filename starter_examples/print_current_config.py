from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point

def run():
    interface = Interface(speed=0.26, file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/YUMI.jacobi-project')
    print(interface.get_FK('left'))
    print(interface.get_FK('right'))
    print("Left current ", interface.driver_left.current_joint_position)
    print("Right current ", interface.driver_right.current_joint_position)
    
    print("Left Min ", interface.yumi.left.min_position)
    print("Left Max ", interface.yumi.left.max_position)
    print("Right Min ", interface.yumi.right.min_position)
    print("Right Max ", interface.yumi.right.max_position)
if __name__ == '__main__':
    run()
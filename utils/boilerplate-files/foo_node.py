import rclpy
from rclpy.node import Node

from foo.foo_core import FooCore

class Foo(Node):
    def __init__(self):
        super().__init__('python_foo')

        # Declare and get the parameters
        self.declare_parameter('version', 1)

        self.__foo = FooCore()

def main(args=None):
    rclpy.init(args=args)

    python_foo = Foo()

    rclpy.spin(python_foo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_foo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # 假设我们正在实现一个加法服务

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        # 请求参数来自于request
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    
    server_node = AddTwoIntsServer()

    while rclpy.ok():
        rclpy.spin_once(server_node)

    # 关闭节点
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
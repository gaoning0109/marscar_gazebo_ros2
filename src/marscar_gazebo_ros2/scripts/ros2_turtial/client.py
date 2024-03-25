import rclpy
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)

    client_node = rclpy.create_node('add_two_ints_client')

    # 创建客户端代理
    client = client_node.create_client(AddTwoInts, 'add_two_ints')

    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            client_node.get_logger().info('Service not available, exiting.')
            break

    # 准备请求消息
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 4

    # 发起服务调用
    future = client.call_async(request)

    # 等待响应
    rclpy.spin_until_future_complete(client_node, future)

    if future.result() is not None:
        response = future.result()
        print(f'The sum of {request.a} and {request.b} is: {response.sum}')
    else:
        raise Exception("Service call failed")

    # 关闭节点
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
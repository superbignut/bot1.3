import socket
import time

def tcp_client():
    # 服务器地址和端口
    server_address = "192.168.198.206"  
    server_port = 5557

    # 创建一个 TCP/IP 套接字
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            # 连接到服务器
            s.connect((server_address, server_port))
            
            # 要发送的消息
            message = "hello world\n"
            
            # 发送消息
            while True:

                s.sendall(message.encode('utf-8'))
                
                # 打印发送成功的消息
                # print(f"Sent: {message}")
                
                time.sleep(1)

                response = s.recv(1024)

                print(f"Received: {response.decode('utf-8')}")

                

        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == "__main__":
    
    tcp_client()
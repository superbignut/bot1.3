import socket

ESP32_IP = "192.168.1.9"  # 替换为ESP32的实际IP地址
ESP32_PORT = 1234          # 与ESP32代码中的端口一致

def main():
    try:
        # 程序启动时建立一次连接
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ESP32_IP, ESP32_PORT))
            print(f"已连接到 ESP32 ({ESP32_IP}:{ESP32_PORT})，输入 'exit' 退出")

            while True:
                message = input("请输入要发送的消息: ").strip()
                if message.lower() == 'exit':
                    print("关闭连接...")
                    break
                if not message:
                    print("错误: 消息不能为空")
                    continue

                try:
                    # 发送消息
                    s.sendall(message.encode('utf-8'))
                    print(f"已发送: {message}")

                except ConnectionResetError:
                    print("错误: ESP32 断开了连接")
                    break
                except Exception as e:
                    print(f"通信错误: {e}")
                    break

    except ConnectionRefusedError:
        print("错误: 无法连接到ESP32，请检查IP和端口是否正确，以及ESP32是否运行")
    except TimeoutError:
        print("错误: 连接超时，请检查网络连接")
    except Exception as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    main()
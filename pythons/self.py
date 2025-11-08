import socket

ESP32_IP = "192.168.1.9"  # 替换为ESP32的实际IP地址
ESP32_PORT = 1234          # 与ESP32代码中的端口一致

def send_message(message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ESP32_IP, ESP32_PORT))
        s.sendall(message.encode('utf-8'))
        data = s.recv(1024)
        print('Received:', data.decode('utf-8'))

if __name__ == "__main__":
    while True:
        message = input("Enter message to send to ESP32 (or 'exit' to quit): ")
        if message.lower() == 'exit':
            break
        send_message(message)
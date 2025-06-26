import socket
import json
import threading

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('172.28.14.196', 5000))  # 実際のIPに変更
print("ラズベリーパイに接続されました")

def receive():
    while True:
        data = client_socket.recv(1024)
        if data:
            try:
                pose_data = json.loads(data.decode())
                print(f"ラズベリーパイからの姿勢データを受信: {pose_data}")
            except:
                print("解析できない姿勢データを受信しました")

def send_ack():
    while True:
        ack_msg = {
            "device": "cugo_ugv",
            "command": "acknowledge_pose",
            "status": "ok"
        }
        json_str = json.dumps(ack_msg)
        client_socket.sendall(json_str.encode() + b'\n')
        print("確認メッセージを送信しました")
        import time
        time.sleep(3)

threading.Thread(target=receive).start()
threading.Thread(target=send_ack).start()
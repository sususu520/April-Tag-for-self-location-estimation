#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import struct
import threading
import time
from datetime import datetime
from pymodbus.client import ModbusTcpClient

# ------------- 配置区 -------------
# 请替换为你的树莓派实际 IP
RASPI_IP = '172.28.14.196'
RASPI_PORT = 1502

# 结果存放文件夹
RESULTS_DIR = "自己位置推定結果"
os.makedirs(RESULTS_DIR, exist_ok=True)

# 生成 JSON 文件名
today = datetime.now().strftime("%Y%m%d")
base_filename = f"{today}_テストデータ"
index = 1
while os.path.exists(os.path.join(RESULTS_DIR, f"{base_filename}_{index}.json")):
    index += 1
json_path = os.path.join(RESULTS_DIR, f"{base_filename}_{index}.json")
print(f"📁 日志文件：{json_path}")

# ------------- 工具函数 -------------
def decode_float(high: int, low: int) -> float:
    """
    将两个 16 位整数组合成一个 32 位浮点数
    """
    combined = (high << 16) | low
    return struct.unpack('>f', combined.to_bytes(4, byteorder='big'))[0]

def log_data(entry: dict):
    """
    往 JSON 文件追加一条记录，自动带 timestamp
    """
    entry["timestamp"] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        if not os.path.exists(json_path):
            with open(json_path, "w", encoding='utf-8') as f:
                json.dump([entry], f, ensure_ascii=False, indent=2)
        else:
            with open(json_path, "r+", encoding='utf-8') as f:
                data = json.load(f)
                data.append(entry)
                f.seek(0)
                json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print("❌ 日志写入失败:", str(e))

# ------------- Modbus 客户端初始化 -------------
client = ModbusTcpClient(RASPI_IP, port=RASPI_PORT)
if not client.connect():
    print(f"❌ 无法连接到 {RASPI_IP}:{RASPI_PORT}")
    exit(1)
print(f"✅ 已连接 Modbus 服务器 {RASPI_IP}:{RASPI_PORT}")

# ------------- 收数据线程 -------------
def receive():
    while True:
        try:
            # 读取保持寄存器 0~5（共 6 个寄存器 = 3 个 float）
            result = client.read_holding_registers(address=0, count=6)
            if result.isError():
                print("❌ 读取失败（Modbus 错误）")
                log_data({"type": "error", "message": "Modbus 读取失败"})
            else:
                regs = result.registers
                x = decode_float(regs[0], regs[1])
                y = decode_float(regs[2], regs[3])
                z = decode_float(regs[4], regs[5])
                print(f"✅ 收到位置：x={x:.3f}, y={y:.3f}, z={z:.3f}")
                log_data({
                    "type": "pose",
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "z": round(z, 3)
                })
        except Exception as e:
            print("❌ 接收异常:", str(e))
            log_data({"type": "error", "message": str(e)})
        time.sleep(1)

# ------------- 发送确认线程 -------------
def send_ack():
    while True:
        try:
            client.write_coils(0, [1])
            print("✅ 发送确认")
        except Exception as e:
            print("❌ 确认发送失败:", str(e))
            log_data({"type": "error", "message": f"ACK 发送失败: {e}"})
        time.sleep(3)

# ------------- 启动线程 -------------
threading.Thread(target=receive, daemon=False).start()
threading.Thread(target=send_ack, daemon=False).start()

# 主线程保持运行
try:
    while True:
        time.sleep(10)
except KeyboardInterrupt:
    print("\n🛑 客户端已退出")
    client.close()
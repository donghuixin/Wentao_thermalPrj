import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import sys

def get_all_frames(file_path):
    """解析 111.txt 提取所有帧"""
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"读取文件失败: {e}")
        return []

    # 提取所有十六进制字节
    hex_bytes = re.findall(r'\b[0-9A-Fa-f]{2}\b', content)
    data = [int(b, 16) for b in hex_bytes]

    frames = []
    header = [0xAA, 0xAA, 0xAA, 0xAA]
    tail = [0xBB, 0xBB, 0xBB, 0xBB]
    
    i = 0
    while i <= len(data) - 1544:
        if data[i:i+4] == header:
            if data[i+1540:i+1544] == tail:
                frame_payload = data[i+4 : i+1540]
                temps = []
                for j in range(0, 1536, 2):
                    val = (frame_payload[j] << 8) | frame_payload[j+1]
                    if val > 32767: val -= 65536
                    temps.append(val / 50.0)
                frames.append(np.array(temps).reshape((24, 32)))
                i += 1544
            else: i += 1
        else: i += 1
    return frames

def play_video():
    file_path = 'c:/Users/17619/Desktop/111.txt'
    frames = get_all_frames(file_path)

    if not frames:
        print("未发现有效帧。")
        return

    # 1. 创建画布
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # 自动设置色彩范围
    all_frames = np.array(frames)
    v_min, v_max = np.min(all_frames), np.max(all_frames)

    # 显示第一帧，去除所有坐标轴和标题
    im = ax.imshow(frames[0], cmap='inferno', interpolation='gaussian', 
                   vmin=v_min, vmax=v_max)
    
    plt.colorbar(im, label='Temperature (°C)')
    ax.axis('off') # 彻底关掉坐标轴，让画面更干净
    # 删掉了 ax.set_title 

    def update(frame_idx):
        # 更新图像
        im.set_array(frames[frame_idx])
        
        # --- 改在控制台实时显示进度 ---
        max_t = np.max(frames[frame_idx])
        # 使用 sys.stdout.write 实现原地刷新控制台数字
        sys.stdout.write(f"\r播放中: 帧 [{frame_idx + 1}/{len(frames)}] | 当前最高温: {max_t:.1f}°C")
        sys.stdout.flush()
        
        return [im]

    # 创建动画
    ani = animation.FuncAnimation(fig, update, frames=len(frames), 
                                  interval=125, blit=True, repeat=True)

    print(f"解析到 {len(frames)} 帧数据。请查看弹出的图像窗口...")
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    play_video()
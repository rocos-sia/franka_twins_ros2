import customtkinter as ctk
import subprocess
import threading
import time
import shutil
import webbrowser
# ========== 配置 ==========
USER = "landau"
HOST = "192.168.0.3"
PASS = "a"

LOCAL_PORT1 = 8443
REMOTE_IP1 = "172.16.0.2"
LOCAL_PORT2 = 9443
REMOTE_IP2 = "172.16.1.2"

LOCAL_NODES = [("camera_package", "yolo_realsense_node")]
REMOTE_NODES = [("franka_twins", "wave")]

# ========== 初始化界面 ==========
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

app = ctk.CTk()
app.title("Franka 控制面板")
app.geometry("650x500")

# ========== 状态框 ==========
log_box = ctk.CTkTextbox(app, width=620, height=260)
log_box.pack(pady=10)
log_box.insert("end", "📋 系统日志输出：\n")

def log(text: str):
    log_box.insert("end", text + "\n")
    log_box.see("end")

# ========== 工具函数 ==========

def run_threaded(func, *args):
    t = threading.Thread(target=func, args=args, daemon=True)
    t.start()

def is_sshpass_installed():
    return shutil.which("sshpass") is not None

def ping_host(ip):
    try:
        subprocess.check_output(["ping", "-c", "1", ip], timeout=2)
        return True
    except Exception:
        return False

def start_single_tunnel(local_port, remote_ip):
    cmd = [
        "sshpass", "-p", PASS, "ssh",
        "-o", "StrictHostKeyChecking=no", "-fN",
        "-L", f"{local_port}:{remote_ip}:443", f"{USER}@{HOST}"
    ]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8, text=True)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip())

def start_local_node(pkg, exe):
    try:
        subprocess.Popen(["ros2", "run", pkg, exe])
        log(f"✅ 本地节点启动成功: {pkg}/{exe}")
    except Exception as e:
        log(f"❌ 本地节点启动失败: {e}")

# def start_remote_node(pkg, exe):
#     ssh_cmd = (
#     "bash -c 'source /opt/ros/humble/setup.bash && nohup ros2 run %s %s > ~/ros2_%s.log 2>&1 &'"
#     % (pkg, exe, exe)
# )
#     cmd = [
#         "sshpass", "-p", PASS, "ssh",
#         "-o", "StrictHostKeyChecking=no",
#         f"{USER}@{HOST}", ssh_cmd
#     ]
#     proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10, text=True)
#     if proc.returncode == 0:
#         log(f"✅ 远程节点启动成功: {pkg}/{exe}")
#     else:
#         log(f"❌ 远程节点启动失败: {proc.stderr.strip()}")
def start_remote_node(pkg, exe):
    ssh_cmd = (
        f"bash -c 'source /opt/ros/humble/setup.bash && nohup ros2 run {pkg} {exe} > ~/ros2_{exe}.log 2>&1 &'"
    )
    cmd = [
        "sshpass", "-p", PASS, "ssh",
        "-o", "StrictHostKeyChecking=no",
        f"{USER}@{HOST}", ssh_cmd
    ]

    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
            text=True
        )
        if proc.returncode == 0:
            log(f"✅ 远程节点启动成功: {pkg}/{exe}")
        else:
            log(f"❌ 启动失败: SSH返回错误：{proc.stderr.strip()}")
    except subprocess.TimeoutExpired as e:
        log(f"❌ SSH 连接超时: {e}")
    except Exception as e:
        log(f"❌ 启动失败: {e}")

# ========== 按钮回调函数 ==========

def handle_start_tunnels():
    log("🚀 尝试启动 SSH 隧道...")
    if not is_sshpass_installed():
        log("❌ 请先安装 sshpass：sudo apt install sshpass")
        return
    if not ping_host(HOST) :
        log("❌ 无法 ping 通 Rocos 运动控制器，请检查网络连接/运动控制器是否在线")
        return
    try:
        start_single_tunnel(LOCAL_PORT1, REMOTE_IP1)
        log(f"✅ SSH 隧道1: localhost:{LOCAL_PORT1} → {REMOTE_IP1}:443")
        start_single_tunnel(LOCAL_PORT2, REMOTE_IP2)
        log(f"✅ SSH 隧道2: localhost:{LOCAL_PORT2} → {REMOTE_IP2}:443")
    except Exception as e:
        log(f"❌ SSH 隧道启动失败: {e}")
        return
    def open_browsers():
        time.sleep(2)
        webbrowser.open(f"https://localhost:{LOCAL_PORT1}/desk/")
        webbrowser.open(f"https://localhost:{LOCAL_PORT2}/desk/")
        messagebox.showinfo("成功", "SSH端口转发已启动，浏览器已打开网页。")
    threading.Thread(target=open_browsers, daemon=True).start()

def handle_start_local_nodes():
    log("🚗 启动本地 ROS2 节点...")
    for pkg, exe in LOCAL_NODES:
        run_threaded(start_local_node, pkg, exe)

def handle_start_remote_nodes():
    log("🛰️ 启动远程 ROS2 节点...")
    for pkg, exe in REMOTE_NODES:
        run_threaded(start_remote_node, pkg, exe)

# ========== 控制按钮区 ==========

btn_frame = ctk.CTkFrame(app)
btn_frame.pack(pady=10)

btn_ssh = ctk.CTkButton(btn_frame, text="🔌 启动 SSH 隧道", width=200, command=lambda: run_threaded(handle_start_tunnels))
btn_ssh.grid(row=0, column=0, padx=10, pady=10)

btn_local = ctk.CTkButton(btn_frame, text="📷 启动本地视觉节点", width=200, command=handle_start_local_nodes)
btn_local.grid(row=1, column=0, padx=10, pady=10)

btn_remote = ctk.CTkButton(btn_frame, text="🤖 启动远程挥手运动节点", width=200, command=handle_start_remote_nodes)
btn_remote.grid(row=2, column=0, padx=10, pady=10)

# ========== 运行主窗口 ==========
app.mainloop()

import customtkinter as ctk
import subprocess
import threading
import time
import shutil
import webbrowser
from tkinter import messagebox

# ========== 配置 ==========
USER = "landau"
HOST = "192.168.0.3"
PASS = "a"
REMOTE_WS_PATH = "/home/landau/Documents/franka_twins_ros2"
LOCAL_WS_PATH = "/home/sun/Documents/GitHub/franka_twins_ros2"
LOCAL_PORT1 = 8443
REMOTE_IP1 = "172.16.0.2"
LOCAL_PORT2 = 9443
REMOTE_IP2 = "172.16.1.2"

LOCAL_NODES = [("camera", "yolo_realsense_node")]
# REMOTE_NODES = [("franka_twins_bringup", "wave_bringup")]
# LAUNCH_FILE = "wave_bringup.launch.py"
LAUNCH_FILES = {
    "wave_bringup": "wave_bringup.launch.py",
    "cola_bringup": "cola_bringup.launch.py",
}

REMOTE_NODES = [
    ("franka_twins_bringup", "wave_bringup"),
    ("franka_twins_bringup", "cola_bringup"),  # 新增
]
# for pkg, exe in REMOTE_NODES:
#     print("Remote nodes:", {LAUNCH_FILES[exe]})
# ========== 初始化界面 ==========
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

app = ctk.CTk()
app.title("Franka 控制面板")
app.geometry("650x520")

# ========== 状态框 ==========
log_box = ctk.CTkTextbox(app, width=620, height=260)
log_box.pack(pady=10)
log_box.insert("end", "📋 系统日志输出：\n")

def log(text: str):
    log_box.insert("end", text + "\n")
    log_box.see("end")

# ========== 工具函数 ==========
import psutil

def kill_process_tree(pid, including_parent=True):
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    for child in children:
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(children, timeout=5)
    if including_parent:
        try:
            parent.kill()
        except psutil.NoSuchProcess:
            pass
        parent.wait(5)
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

# ========== 本地节点管理 ==========
local_procs = []  # 用于保存启动的本地节点进程对象

def start_local_node(pkg, exe):
    cmd = (
        f"cd {LOCAL_WS_PATH} && "
        f"source install/setup.bash && "
        f"ros2 run {pkg} {exe}"
    )
    try:
        proc = subprocess.Popen(["bash", "-c", cmd])
        local_procs.append(proc)
        log(f"✅ 本地节点启动成功: {pkg}/{exe}")
    except Exception as e:
        log(f"❌ 本地节点启动失败: {e}")

# def stop_local_nodes():
#     for proc in local_procs:
#         try:
#             proc.terminate()
#             log("✅ 发送终止信号到本地节点进程")
#         except Exception as e:
#             log(f"❌ 终止本地节点失败: {e}")
#     local_procs.clear()
def stop_local_nodes():
    for proc in local_procs:
        try:
            kill_process_tree(proc.pid)
            log(f"✅ 杀死本地节点进程 PID={proc.pid}")
        except Exception as e:
            log(f"❌ 杀死本地节点失败 PID={proc.pid}，错误：{e}")
    local_procs.clear()
# ========== 远程节点管理 ==========
def start_remote_node(pkg, exe):
    ssh_cmd = (
        f"bash -c \""
        f"cd {REMOTE_WS_PATH} && "
        "source install/setup.bash && "
        f"sh -c 'ros2 launch {pkg} {LAUNCH_FILES[exe]} > ~/ros2_{exe}.log 2>&1 &' "
        f"&& sleep 1 "
        f"&& pgrep -f 'ros2 launch {pkg} {LAUNCH_FILES[exe]}' | head -n1 > ~/ros2_{exe}.pid"
        "\""
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

def stop_remote_nodes(pkg):
    
    log("⏹️ 正在尝试关闭远程节点...")

    # 先杀父进程 ros2 launch
    ssh_cmd_parent = (
        "pkill -f 'ros2 launch franka_twins_bringup wave_bringup.launch.py' || true"
    )

    # 再分别杀掉wave和gripper
    ssh_cmd_wave = (
        "pkill -f wave || true"
    )
    ssh_cmd_gripper = (
        "pkill -f dh_gripper || true"
    )

    cmds = [ssh_cmd_parent, ssh_cmd_wave, ssh_cmd_gripper]
    for cmd in cmds:
        ssh_command = [
            "sshpass", "-p", PASS, "ssh",
            "-o", "StrictHostKeyChecking=no",
            f"{USER}@{HOST}",
            "bash -c '" + cmd + "'"
        ]
        try:
            proc = subprocess.run(
                ssh_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            log(f"✅ 已执行：{cmd}")
        except Exception as e:
            log(f"❌ 执行 {cmd} 出错: {e}")

# ========== 按钮回调函数 ==========
def handle_start_tunnels():
    log("🚀 尝试启动 SSH 隧道...")
    if not is_sshpass_installed():
        log("❌ 请先安装 sshpass：sudo apt install sshpass")
        return
    if not ping_host(HOST):
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

def handle_stop_local_nodes():
    log("⏹️ 关闭所有本地节点...")
    stop_local_nodes()

def handle_start_remote_nodes():
    log("🛰️ 启动远程 ROS2 节点...")
    for pkg, exe in REMOTE_NODES:
        run_threaded(start_remote_node, pkg, exe)

def handle_stop_remote_nodes():
    log("⏹️ 关闭远程 ROS2 节点...")
    for pkg in REMOTE_NODES:
        run_threaded(stop_remote_nodes, pkg)

# ========== 控制按钮区 ==========
btn_frame = ctk.CTkFrame(app)
btn_frame.pack(pady=10)

btn_ssh = ctk.CTkButton(btn_frame, text="🔌 启动 SSH 隧道", width=200, command=lambda: run_threaded(handle_start_tunnels))
btn_ssh.grid(row=0, column=0, padx=10, pady=10)

btn_local_start = ctk.CTkButton(btn_frame, text="📷 启动本地视觉节点", width=200, command=handle_start_local_nodes)
btn_local_start.grid(row=1, column=0, padx=10, pady=10)

btn_local_stop = ctk.CTkButton(btn_frame, text="⏹️ 关闭本地视觉节点", width=200, command=handle_stop_local_nodes)
btn_local_stop.grid(row=1, column=1, padx=10, pady=10)

btn_remote_start = ctk.CTkButton(btn_frame, text="🤖 启动远程挥手运动节点", width=200, command=handle_start_remote_nodes)
btn_remote_start.grid(row=2, column=0, padx=10, pady=10)

btn_remote_stop = ctk.CTkButton(btn_frame, text="⏹️ 关闭远程挥手节点", width=200, command=handle_stop_remote_nodes)
btn_remote_stop.grid(row=2, column=1, padx=10, pady=10)
btn_remote_start_cola = ctk.CTkButton(btn_frame, text="🧃 启动 Cola 运动节点", width=200, command=lambda: run_threaded(start_remote_node, "franka_twins_bringup", "cola_bringup"))
btn_remote_start_cola.grid(row=3, column=0, padx=10, pady=10)

btn_remote_stop_cola = ctk.CTkButton(btn_frame, text="⏹️ 关闭 Cola 节点", width=200, command=lambda: run_threaded(stop_remote_nodes, ("franka_twins_bringup", "cola_bringup")))
btn_remote_stop_cola.grid(row=3, column=1, padx=10, pady=10)

# ========== 运行主窗口 ==========
app.mainloop()

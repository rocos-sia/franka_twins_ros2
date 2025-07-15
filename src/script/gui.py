import subprocess
import webbrowser
import tkinter as tk
from tkinter import messagebox
import threading
import time
import shutil

USER = "landau"
HOST = "192.168.0.3"
PASS = "a"

LOCAL_PORT1 = 8443
REMOTE_IP1 = "172.16.0.2"

LOCAL_PORT2 = 9443
REMOTE_IP2 = "172.16.1.2"

def is_sshpass_installed():
    return shutil.which("sshpass") is not None

def ping_host(ip):
    # 跨平台ping命令，发4个包
    try:
        output = subprocess.check_output(
            ["ping", "-c", "4", ip],
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=5
        )
        # 你也可以分析output内容是否丢包，这里只简单返回成功即True
        return True
    except subprocess.CalledProcessError:
        return False
    except subprocess.TimeoutExpired:
        return False

def start_single_tunnel(local_port, remote_ip):
    try:
        proc = subprocess.Popen([
            "sshpass", "-p", PASS, "ssh",
            "-o", "StrictHostKeyChecking=no",
            "-fN",
            "-L", f"{local_port}:{remote_ip}:443",
            f"{USER}@{HOST}"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(1)
        if proc.poll() is not None:
            stdout, stderr = proc.communicate()
            raise RuntimeError(f"端口转发失败: {stderr.decode().strip()}")
        return proc
    except Exception as e:
        raise e

def start_tunnels(button):
    if not is_sshpass_installed():
        messagebox.showerror("错误", "未检测到 sshpass，请先安装 (sudo apt install sshpass)")
        button.config(state=tk.NORMAL)
        return

    # 先ping检测两个远程IP
    if not ping_host(REMOTE_IP1):
        messagebox.showerror("错误", f"无法ping通 {REMOTE_IP1}，请检查网络连接")
        button.config(state=tk.NORMAL)
        return
    if not ping_host(REMOTE_IP2):
        messagebox.showerror("错误", f"无法ping通 {REMOTE_IP2}，请检查网络连接")
        button.config(state=tk.NORMAL)
        return

    button.config(state=tk.DISABLED)
    try:
        proc1 = start_single_tunnel(LOCAL_PORT1, REMOTE_IP1)
        proc2 = start_single_tunnel(LOCAL_PORT2, REMOTE_IP2)

        def open_browsers():
            time.sleep(2)
            webbrowser.open(f"https://localhost:{LOCAL_PORT1}/desk/")
            webbrowser.open(f"https://localhost:{LOCAL_PORT2}/desk/")
            messagebox.showinfo("成功", "SSH端口转发已启动，浏览器已打开网页。")

        threading.Thread(target=open_browsers, daemon=True).start()

    except Exception as e:
        messagebox.showerror("启动失败", str(e))
        button.config(state=tk.NORMAL)

def main():
    root = tk.Tk()
    root.title("Panda SSH端口转发启动器")
    root.geometry("360x150")
    root.resizable(False, False)

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    x = (screen_w - 360) // 2
    y = (screen_h - 150) // 2
    root.geometry(f"+{x}+{y}")

    btn_style = {
        "font": ("Arial", 14),
        "bg": "#4CAF50",
        "fg": "white",
        "activebackground": "#45a049",
        "padx": 10,
        "pady": 8,
        "relief": tk.FLAT,
        "cursor": "hand2",
    }

    btn = tk.Button(root, text="启动端口转发并打开网页", command=lambda: start_tunnels(btn), **btn_style)
    btn.pack(pady=40)

    root.mainloop()

if __name__ == "__main__":
    main()

import socket
import subprocess

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def wifiname():
    try:
        return subprocess.check_output('iwgetid').decode().split("ESSID:")[-1].replace("\"", "").replace("\n", "")
    except Exception as e:
        return "ERROR"

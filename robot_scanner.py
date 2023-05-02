import nmap
import time
import paramiko
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox as MessageBox
from pexpect import pxssh
from PIL import Image, ImageTk
import os
import rosgraph
import shlex
from psutil import Popen
import threading
import subprocess


hosts =   ['192.168.0.165',
         '192.168.0.181',
         '192.168.0.106',
         '192.168.0.195',
         '192.168.0.110',
         '192.168.0.189'
          ]

ips_scanned = 0
scanner = nmap.PortScanner()
robot_selected = "No robot selected"
ip_available = [False, False, False, False, False, False]
root = tk.Tk()

#user = 'remote_laboratory'
#passw = 'pumaspumas'
user = 'pi'
passw = 'raspberry'

userName = os.environ.get("USERNAME")

def launch_simulator(ip):
    os.environ["ROS_MASTER_URI"]="http://" + ip + ":11311"
    if(rosgraph.is_master_online()):
        process = subprocess.check_call(['/home/' + userName
                                    + '/MobileRobotSimulator/./runSimulator.sh', ip])

        print("Process finished")
        #process.kill()

def ros_finder(ip):
    os.environ["ROS_MASTER_URI"]="http://" + ip + ":11311"
    print('.', end='', flush=True)
    if(rosgraph.is_master_online()):
        print("ROS MASTER FOUND AT->" + ip)
        return True
    
    return False

def connect_host(ip):
    print('Connecting with->' + ip)

    try:
        ssh = paramiko.SSHClient()

        # Load SSH host keys.
        ssh.load_system_host_keys()
        # Add SSH host key automatically if needed.
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Connect to router using username/password authentication.
        ssh.connect(ip, username=user, password=passw, look_for_keys=False, timeout=10)
        transport = ssh.get_transport()
        channel = transport.open_session()

        channel.exec_command('~/Minibot/start.sh')
        #MessageBox.showinfo("Connecting", "Please wait...")
        print("Searching ROS MASTER...")
        while True:
            if(ros_finder(ip)):
                break
        launch_simulator(ip)

        print("Killing rosmaster for Minibot")
        ssh.exec_command('killall rosmaster')
        #channel.exec_command('killall rosmaster')
        channel.close()
        ssh.close()

    except Exception as e:
        print(e)
        MessageBox.showerror("Error message", "Cannot connect with robot")
        

        
def event(i):
    global robot_selected
    robot_selected = "Selected->" + " Minibot " + str(i)
    connect_host(hosts[i-1])

    selected_label = ttk.Label(root, text = robot_selected, font='Helvetica 11 bold')
    selected_label.grid(column=1, row=len(hosts)+1, sticky=tk.W, padx=5, pady=5)

def scan_ip(ip, id):
    global ips_scanned, ip_available
    scanner.scan(ip, '1', '-v')
    ips_scanned += 1
    if(scanner[ip].state() == "up"):
        ip_available[id] = True
    
def scan_all():
    global ips_scanned, robot_selected
    id=0
    robot_selected = "No robot selected     "
    start = time.time()
    print("\nScanning...")
    for host in hosts:
        thread = threading.Thread(target=scan_ip, args=(host, id, ))
        thread.start()
        id += 1
    
    while(ips_scanned < len(hosts)):
        wait = True

    end = time.time()
    scaning_time = end-start
    print("Scanned in->", end = ' ')
    print(scaning_time, end= ' ')
    print("seconds")    
    ips_scanned = 0

    selected_label = ttk.Label(root, text = robot_selected, font='Helvetica 11 bold')
    selected_label.grid(column=1, row=len(hosts)+1, sticky=tk.W, padx=5, pady=5)

def render_gui():
    root.geometry("400x360")
    root.title('Robots scanner')
    root.resizable(0, 0)

    # configure the grid
    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=3)

    err_image = Image.open('/home/remotelab/MobileRobotSimulator/error.png').resize((25,25), Image.ANTIALIAS)
    ok_image =  Image.open('/home/remotelab/MobileRobotSimulator/ok.png').resize((25,25), Image.ANTIALIAS)

    label = ttk.Label(root, text="Status", font='Helvetica 11 bold')
    label.grid(column=0, row=0, sticky=tk.W, padx=(20,0), pady=5)

    i = 1
    for ip in hosts:
        if(ip_available[i-1]):
            status = "Available"
            icon_status = ImageTk.PhotoImage(ok_image)
            button = ttk.Button(root, text="Connect", command=lambda m = i: event(m))
        else:
            status = "Not available"
            icon_status = ImageTk.PhotoImage(err_image)
            button = ttk.Button(root, text="Connect" , state= tk.DISABLED)

        label = ttk.Label(root, image = icon_status)
        label.image = icon_status
        label.grid(row=i)

        minibot_label = ttk.Label(root, text="Minibot " + str(i) + ":   " + status)
        minibot_label.grid(column=1, row=i, sticky=tk.W, padx=5, pady=5)
        button.grid(column=2, row=i, sticky=tk.W, padx=(0,10), pady=5)
        
        i += 1

    button_refresh = ttk.Button(root, text="Refresh", command=scan_all)
    button_refresh.grid(column=0, row=i, sticky=tk.W, padx=5, pady=5)

    selected_label = ttk.Label(root, text = robot_selected, font='Helvetica 11 bold')
    selected_label.grid(column=1, row=i, sticky=tk.W, padx=5, pady=5)

    root.mainloop()


def main():
    scan_all()
    render_gui()
    
if __name__ == '__main__':
    try:
        main()
    except:
        pass
    finally:
        print("Finishing robot_scanner.py...")


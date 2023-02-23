import nmap
import time
import paramiko
import tkinter as tk
from tkinter import ttk
from pexpect import pxssh
from PIL import Image, ImageTk
import os
import rosgraph
import shlex
from psutil import Popen

hosts =   ['192.168.0.165',
         '192.168.0.181',
         '192.168.0.106',
         '192.168.0.195',
         '192.168.0.110',
         '192.168.0.193'
          ]

#ip_available = [False, False, False, True, False, True]
ip_available = []

def launch_simulator(ip):
    os.environ["ROS_MASTER_URI"]="http://" + ip + ":11311"
    if(rosgraph.is_master_online()):
        node_process = Popen(
            shlex.split('roslaunch simulator minibot.launch')
        )

def ros_finder(ip):
    os.environ["ROS_MASTER_URI"]="http://" + ip + ":11311"
    print('.', end='', flush=True)
    if(rosgraph.is_master_online()):
        print("ROS MASTER FOUND AT->" + ip)
        return True
    
    return False

def connect_host(ip):
    print('Connecting with->' + ip)

    ssh = paramiko.SSHClient()

    # Load SSH host keys.
    ssh.load_system_host_keys()
    # Add SSH host key automatically if needed.
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    user = 'remote_laboratory'
    passw = 'pumaspumas'

    # Connect to router using username/password authentication.
    ssh.connect(ip, username=user, password=passw, look_for_keys=False, timeout=10)
    transport = ssh.get_transport()
    channel = transport.open_session()

    channel.exec_command('~/Minibot/start.sh')
    print("Searching ROS MASTER...")
    while True:
        if(ros_finder(ip)):
            break
    launch_simulator(ip)

    channel.close()
    ssh.close()
        
def event(i):
    connect_host(hosts[i-1])

def auto_scan():
    global ip_available
    scanner = nmap.PortScanner()
    for ip in hosts:
        scanner.scan(ip, '1', '-v')
        if(scanner[ip].state() == "up"):
            print(ip + '->' + ' ' + 'Available')
            ip_available.append(True)
        else:
            print(ip + '->' + ' ' + 'Not available')
            ip_available.append(False)
    #print(ip_available)

def render_gui():
    print("Render ")
    #root window
    root = tk.Tk()
    root.geometry("400x280")
    root.title('Robots scanner')
    root.resizable(0, 0)

    # configure the grid
    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=3)

    err_image = Image.open('error.png').resize((25,25), Image.ANTIALIAS)
    ok_image =  Image.open('ok.png').resize((25,25), Image.ANTIALIAS)

    label = ttk.Label(root, text="Status")
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

    selected_robot = "No robot selected"
    selected_label = ttk.Label(root, text= selected_robot)
    selected_label.grid(column=1, row=i, sticky=tk.W, padx=5, pady=5)

    root.mainloop()

def main():
    auto_scan()
    render_gui()


if __name__ == '__main__':
    try:
        main()
    except:
        pass
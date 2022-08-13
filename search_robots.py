import nmap, socket

hosts = {'Minibot 1':'192.168.0.165',
         'Minibot 2':'192.168.0.181',
         'Minibot 3':'192.168.0.106',
         'Minibot 4':'192.168.0.195',
         'Minibot 5':'192.168.0.110'}

#hosts['Streamer PC'] = '192.168.0.193'

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


print("\n\nSearching minibots connections...\n")

scanner = nmap.PortScanner()
for hostName in sorted(hosts.keys()):
    scanner.scan(hosts[hostName], '1', '-v')
    if(scanner[hosts[hostName]].state() == "up"):
        print(bcolors.OKGREEN + bcolors.BOLD + hostName + '->' + ' ' + 'Available' + bcolors.ENDC)
    else:
        print hostName + '->' + ' ' + 'Not available'

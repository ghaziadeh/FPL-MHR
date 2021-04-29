import subprocess
import time
import os


def scan_bins():
    # this functions will return the ID of all bins in the perimeter.
    available = subprocess.check_output(["hcitool", "scan"]).decode("utf-8")
    print(available)
    fullstring = available
    substring = '\t'
    start = 0
    id_list = []
    while True:
        start = fullstring.find(substring)
        print(start)
        if start != -1:
            end =fullstring[start+2:len(fullstring)].find(substring)
            end = end +start+2
            id_list.append(fullstring[start+1:end])
            fullstring = fullstring[end+2:len(fullstring)]
        else:
            print('all bins identified and parsed:')
            print(id_list)
            break
    return id_list


def connect_to_bin(item):
    # use subprocess.checkoutput to verify connection and find bin.
    command = "sudo rfcomm connect 0"+item+" 10 >/dev/null"
    os.system(command)
    print('connected to: '+item)


def bt_strength(item):
    # check signal strength in dbm
    result = str(subprocess.check_output(["hcitool", "rssi", item]).decode("utf-8"))
    f=result.split()
    data = int(f[3])
    print(data)
    return data

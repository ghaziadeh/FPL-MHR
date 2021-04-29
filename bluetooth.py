import subprocess
import time
import os


def scan_bins():
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
    result = str(subprocess.check_output(["hcitool", "rssi", item]).decode("utf-8"))
    f=result.split()
    data = int(f[3])
    print(data)
    return data

#print(scan_bins())
#ID1 = '01:B6:EC:B8:EE:19'
#ID2 = '01:B6:EC:B8:F3:E4'
ab_iphone = '6C:E8:5C:5C:D5:87'
item = ab_iphone
#connect_to_bin(item)
#scan_bins()
#connect_to_bin(ID1)
#connect_to_bin(ID2)
#connect_to_bin(ID2)
# commands:
# for connection: add id from list instead of one written in the middle which is my iphone's "6C:etc:87"
    #os.system("sudo rfcomm connect 0 6C:E8:5C:5C:D5:87 10 >/dev/null")
    #subprocess.call(["sudo rfcomm connect 0 6C:E8:5C:5C:D5:87  10 >/dev/null &"])

# last section checks signal strength for id written below. 
# bdaddr = "6C:E8:5C:5C:D5:87"
# while True:
#     result = subprocess.check_output(["hcitool", "rssi", bdaddr])
#     print(result)
#     time.sleep(0.3)
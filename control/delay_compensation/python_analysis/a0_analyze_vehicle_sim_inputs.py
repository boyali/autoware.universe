import os
import matplotlib.pyplot as plt
import numpy as np

data_path = "/mnt/KinsgtonExternal/ROS2_universe/logs"
# Press the green button in the gutter to run the script.

files = os.listdir(data_path)
file_list = []
file_names = []

for f in files:
    if f.endswith(".txt"):
        print(f)
        file_list.append(data_path + f)
        file_names.append(f)

    file_list.sort()
    file_names.sort()

def load_numpy(k):
    f0 = data_path + "/" + file_names[k]

    return np.loadtxt(f0)

if __name__ == '__main__':

    for (f, k) in zip(file_names, range(len(file_names))):
        print(f" k: {k}, file_name: {f}")



    ## LOAD DATA
    u_vel = load_numpy(0)
    u_steer = load_numpy(1)
    u_triag = load_numpy(3)
    timevec = load_numpy(2)

    plt.figure()
    plt.plot(timevec, u_vel)

    plt.figure()
    plt.plot(timevec, u_steer)
    plt.show()



    print()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

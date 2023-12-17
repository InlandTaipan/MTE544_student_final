import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors(files):

    # Plot only one robotPose.csv file
    if (len(files) == 1):
        filename = files[0]
        headers, values=FileReader(filename).read_file()
        
        time_list=[]
    
        first_stamp=values[0][-1]

        for val in values:
            time_list.append(val[-1] - first_stamp)

        # Isolate x and y poses of robot from data
        x = [lin[-5] for lin in values]
        y = [lin[-4] for lin in values]

        plt.plot(x, y, label = "Trajectory")

    plt.legend()
    plt.title("X vs Y Trajectory")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    plt.show()
    
import argparse

if __name__=="__main__":

    ap = argparse.ArgumentParser(description='Make graph')
    ap.add_argument('--files', nargs='+', required=True, help='list of files')

    args = ap.parse_args()

    print('plotting the files', args.files)

    plot_errors(args.files)
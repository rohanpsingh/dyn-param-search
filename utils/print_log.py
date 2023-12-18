import sys
import mc_log_ui
import matplotlib.pyplot as plt

JOINT_ID = 1

def parse(logfile, outfile):
    print("Parsing: {}".format(logfile))
    log = mc_log_ui.read_log(logfile)

    # get start time of 'Demo' state
    exec_main = log['Executor_Main']
    demo_start = exec_main.index('JointCalibState')

    with open(outfile, 'w') as f:
        for t in range(17000):
            l = log['qIn_' + repr(JOINT_ID)][t]
            f.write(repr(l) + "\n")

if __name__=='__main__':
    in_fn = sys.argv[1]
    out_fn = sys.argv[2]
    parse(in_fn, out_fn)

import argparse
import mc_log_ui
import matplotlib.pyplot as plt

def parse(logfile, outfile, jointid):
    print("Parsing: {}".format(logfile))
    log = mc_log_ui.read_log(logfile)

    # get start time of 'Demo' state
    exec_main = log['Executor_Main']
    demo_start = exec_main.index('JointCalibState')
    demo_end = len(exec_main) - exec_main[::-1].index('JointCalibState') - 1

    with open(outfile, 'w') as f:
        for t in range(demo_start, demo_end):
            l = log['qIn_' + repr(jointid)][t]
            f.write(repr(l) + "\n")

if __name__=='__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--in-file", type=str, required=True)
    parser.add_argument("--out-file", type=str, required=True)
    parser.add_argument("--joint-id", type=int, required=True)
    args = parser.parse_args()

    parse(args.in_file, args.out_file, args.joint_id)

import sys
import numpy as np
import matplotlib.pyplot as plt

def parse(fn):
    with open(fn, 'r') as f:
        lines = [l.strip().split(',') for l in f.readlines()]
    return np.array(lines).astype(float)

def main():
    filepaths = sys.argv[1:]

    for fp in filepaths:
        dat = parse(fp)
        plt.plot(dat, label=fp)
    plt.legend()
    plt.show()
    

if __name__=='__main__':
    main()

"""
Read joint data from Kinect and generate histograms.

Author: Zach Mills
HCR Project 3 Deliverable 1
"""

import pandas as pd
import numpy as np
from glob import glob
from statistics import mean, stdev
import svmutil as svm
import argparse


def read_dir(path, sep):
    """
    Read every file fitting path into a pandas df, using delimeter.

    expect path to be `dataset/test/*.txt`
    or `dataset/train/*.txt`
    """
    names = [str(i) for i in range(1, 6)]
    filenames = glob(path)
    nums = [int(f.split('/')[-1].split('_')[0][1:]) for f in filenames]
    return nums, [pd.read_csv(f, sep=sep, names=names) for f in filenames]


def remove_outliers(data):
    """Remove outliers based on std deviation."""
    avg = mean(data)
    std = stdev(data)
    return [d for d in data if (avg - (2 * std)) < d < (avg + (2 * std))]


def get_dists(locs, center):
    """Get the distances between all values and the center."""
    return [np.linalg.norm(np.array(loc) - np.array(center)) for loc in locs]


def get_angles(locs, center):
    """Get the angles between each pair of locations."""
    angles = []
    previous_loc = locs[-1]
    for loc in locs:
        v_1 = np.squeeze(np.array(loc) - np.array(center))
        v_1_u = v_1 / np.linalg.norm(v_1)
        v_2 = np.squeeze(np.array(loc) - np.array(previous_loc))
        v_2_u = v_2 / np.linalg.norm(v_2)
        try:
            angles.append(np.arccos(np.clip(np.dot(v_1_u, v_2_u), -1.0, 1.0)))
        except ValueError:
            print(locs)
            exit()
        previous_loc = loc

    return angles


def analyze_action(df, joints, center, edges):
    """
    Analyze the frames of a single action.

    Frames should be provided in a pandas dataframe.
    Joints is a list of ints representing important joints in order.
    Center is an int which represents the center joint
    """
    dist_list = []
    angle_list = []
    hist = []
    for i in range(1, df['1'].max() + 1):
        locs = [df.loc[(df['1'] == i) & (df['2'] == j)] for j in joints]
        locs = [loc[['3', '4', '5']] for loc in locs]
        center_loc = locs[joints.index(center)]
        locs.remove(center_loc)
        for loc in locs:
            if loc.isnull().values.any():
                continue
        dists = np.array(get_dists(locs, center_loc))
        angles = np.array(get_angles(locs, center_loc))

        dist_list.append(dists)
        angle_list.append(angles)

    dists = np.array(dist_list)
    angles = np.array(angle_list)

    for i in range(len(dist_list[0])):
        hist.extend(np.histogram(remove_outliers(dists[:, i]), density=True,
                                 bins=edges[0])[0].tolist())
        hist.extend(np.histogram(remove_outliers(angles[:, i]), density=True,
                                 bins=edges[1])[0].tolist())

    return [str(x) for x in hist]


def get_rad(dataframe):
    """Get RAD representation of action dataframe."""
    joints = [1, 4, 12, 20, 16, 8]
    center = 1
    # edges = []
    # edges.append([0, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 2])
    # edges.append([0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 5])
    edges = [5, 10]
    return analyze_action(dataframe, joints, center, edges)


def get_custom(dataframe):
    """Get custom representation of action dataframe."""
    joints = [1, 3, 10, 18, 14, 6]
    center = 1
    # edges = []
    # edges.append([0, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 2])
    # edges.append([0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 5])
    edges = [5, 10]
    return analyze_action(dataframe, joints, center, edges)


def convert_line(l, c, id=0):
    """Convert one histogram line into a LIBSVM line."""
    return [f'{c}'] + [f'{i + 1}:{n}' for i, n in enumerate(l)]


def reformat(l, n):
    """Reformat histogram file into LIBSVM format."""
    return [convert_line(l, c, i) for i, (l, c) in enumerate(zip(l, n))]


def main():
    """Build representation from files."""
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--regen", help="increase output verbosity",
                        action="store_true")
    args = parser.parse_args()

    if args.regen:
        train_class, train_frames = read_dir('dataset/train/*.txt', ' ')
        test_class, test_frames = read_dir('dataset/test/*.txt', ' ')

        rad_d1 = [get_rad(action) for action in train_frames]
        rad_d1_t = [get_rad(action) for action in test_frames]

        cust_d1 = [get_custom(action) for action in train_frames]
        cust_d1_t = [get_custom(action) for action in test_frames]

        with open('rad_d1', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in rad_d1])
        with open('rad_d1.t', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in rad_d1_t])

        with open('cust_d1', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in cust_d1])
        with open('cust_d1.t', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in cust_d1_t])

        rad_d2 = reformat(rad_d1, train_class)
        rad_d2_t = reformat(rad_d1_t, test_class)

        cust_d2 = reformat(cust_d1, train_class)
        cust_d2_t = reformat(cust_d1_t, test_class)

        with open('rad_d2', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in rad_d2])
        with open('rad_d2.t', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in rad_d2_t])

        with open('cust_d2', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in cust_d2])
        with open('cust_d2.t', 'w') as f:
            f.writelines([' '.join(line) + '\n' for line in cust_d2_t])

    # Train the models and test with them
    y, x = svm.svm_read_problem('rad_d2')
    y_t, x_t = svm.svm_read_problem('rad_d2.t')
    rad_model = svm.svm_train(y, x, '-s 0 -t 2 -c 2 -g 0.0005')
    rad_labels, (rad_acc, *_), _ = svm.svm_predict(y_t, x_t, rad_model)

    y, x = svm.svm_read_problem('cust_d2')
    y_t, x_t = svm.svm_read_problem('cust_d2.t')
    cust_model = svm.svm_train(y, x, '-s 0 -t 2 -c 8 -g 0.0005')
    cust_labels, (cust_acc, *_), _ = svm.svm_predict(y_t, x_t, cust_model)

    print(f'RAD accuracy: {rad_acc}')
    print(f'Custom accuracy: {cust_acc}')


if __name__ == "__main__":
    main()

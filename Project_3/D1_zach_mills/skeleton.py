"""
Read joint data from Kinect and generate histograms.

Author: Zach Mills
HCR Project 3 Deliverable 1
"""

import pandas as pd
import numpy as np
from glob import glob


def read_dir(path, sep):
    """
    Read every file fitting path into a pandas df, using delimeter.

    expect path to be `dataset/test/*.txt`
    or `dataset/train/*.txt`
    """
    names = [str(i) for i in range(1, 6)]
    return [pd.read_csv(f, sep=sep, names=names) for f in glob(path)]


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
        angles.append(np.arccos(np.clip(np.dot(v_1_u, v_2_u), -1.0, 1.0)))
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
    for i in range(1, 21):
        locs = [df.loc[(df['1'] == i) & (df['2'] == j)] for j in joints]
        locs = [loc[['3', '4', '5']] for loc in locs]
        center_loc = locs[joints.index(center)]
        locs.remove(center_loc)
        dists = np.array(get_dists(locs, center_loc))
        angles = np.array(get_angles(locs, center_loc))

        dist_list.append(dists)
        angle_list.append(angles)

    dists = np.array(dist_list)
    angles = np.array(angle_list)

    for i in range(len(dist_list[0])):
        hist.extend(np.histogram(dists[:, i], density=True,
                                 bins=edges[0])[0].tolist())
        hist.extend(np.histogram(angles[:, i], density=True,
                                 bins=edges[1])[0].tolist())

    return [str(x) for x in hist]


def get_rad(dataframe):
    """Get RAD representation of action dataframe."""
    joints = [1, 4, 8, 12, 16, 20]
    center = 1
    edges = []
    edges.append([0, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 2])
    edges.append([0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 5])
    return analyze_action(dataframe, joints, center, edges)


def get_custom(dataframe):
    """Get custom representation of action dataframe."""
    joints = [1, 3, 6, 10, 14, 18]
    center = 1
    edges = []
    edges.append([0, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 2])
    edges.append([0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 5])
    return analyze_action(dataframe, joints, center, edges)


def main():
    """Build representation from files."""
    train_frames = read_dir('dataset/train/*.txt', ' ')
    test_frames = read_dir('dataset/test/*.txt', ' ')

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


if __name__ == "__main__":
    main()

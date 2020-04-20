# Project 3, Deliverable 1
## Author: Zach Mills

### Running
To run the code, the *skeleton.py* file must be in the same directory as your data. The python file allows for custom data paths, but in order to make it work without editing the source, the data should be inside a *train* folder and a *test* folder, each of which should be inside a *dataset* folder, which is in the same directory as the python file. Then, simply run `python skeleton.py` to execute the program. It will then create the 4 histogram files in the same directory.

### Implementation
My implementation of the RAD representation uses joints for the Hip Center, Head, Left Hand, Right Hand, Left Foot, and Right Foot as the skeleton representation. First, the distance is computed from each joint to the center at every frame, and a vector from the center to the joint is computed. Then the angle between consecutive joint vectors is computed for every frame. This leads to 10 data points for every frame, which means we need 10 histograms. These are computed on the columns of a data table that has been built by adding all of our data as a new row each frame. They are calculated using numpy's histogram feature, but the bin edges are passed as an argument, ensuring that the bin edges are identical for each action and each frame. These bin edges are defined in the code as follows:

**Distance Bin Edges: `[0, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 2]`**
**Angle Bin Edges: `[0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 5]`**

These lead to a total of 9 bins for the angles, and 10 bins for the distances, meaning there are 19 bins per joint, and 95 bins per frame.

The same bin edges are used for my custom implementation, which is similar to the RAD implementation but rather than using body extremities, I use less extreme joints. Specifically, the Neck, Left Elbow, Right Elbow, Left Knee, and Right Knee, as well as the same Hip Center used as the center joint.

### Data
The data output files can be found in the same directory that the code was run in. The RAD representation histogram files are called

`rad_d1` and `rad_d1.t`

and my custom representation histogram files are called

`cust_d1` and `cust_d1.t`

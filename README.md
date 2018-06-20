# find-superquadric
Run optimization problem to find out the best superquadric that fits a given partial point cloud.

### Note
The superquadric is parametrized in terms of its center, the shape, the principal axes and a rotation angle around the z-axis, in order to account for objects that are supposed to be lying on a table parallel to the x-y plane, while keeping things simple :wink:

The equation of the superquadric is the following:

![equation](./misc/equation.png)

### Dependencies
- [Yarp](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [Ipopt](https://github.com/coin-or/Ipopt)
- [VTK](https://github.com/Kitware/VTK)

### Command-line options
- `--file file-name`: specify the file containing the point cloud given in the following plain format:
  ```
  x0 y0 z0 [r0 g0 b0]
  x1 y1 z1 [r1 g1 b1]
  ...
  ```
- `--remove-outliers "(<radius> <neighbors>)"`: remove those points (outliers) that do not have a specified number of neighbors contained within a given radius.
- `--uniform-sample <int>`: specify the integer step for performing uniform down-sampling as follows:
  - `1` means no down-sampling
  - `> 1` enables down-sampling
- `--random-sample <double>`: specify the percentage in [0,1] for performing random down-sampling.
- `--inside-penalty <double>`: specify how much to penalize points that will lie inside the superquadric's isosurface wrt points lying outside (default = 1.0).
- `--disable-viewer`: specify not to launch the viewer.

### Real-time mode
If no `--file` option is passed through the command line, the module will open up a port called `/find-superquadric/points:rpc` to which the point cloud can be sent as a `yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>` object.
Then, the module will reply with the superquadric parameters:
```
center-x center-y center-z angle size-x size-y size-z epsilon-1 epsilon-2
```
The `angle` around the z-axis is returned in degrees.

### Example
```sh
find-superquadric --file ./data/box
```

### Output
![output](./misc/output.png)

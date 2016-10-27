Julie Shade jshade1
Chris Micek cmicek1

JHU Computer Integrated Surgery
Fall 2016
Programming Assignment 2

File Listing:

pivot_cal.py: Contains method to perform pivot calibration on a given list of point clouds.

Frame.py: Frame class, contains data members for rotation and translation as well as commonly needed frame methods
such as composition and inversion.

PointCloud.py: PointCloud class, a list of points in a given point cloud and commonly needed methods such as
registration between two point clouds and transformation of an entire point cloud by a given frame. Also contains code
for translating input files into lists of PointClouds.

distortion.py: Contains all methods needed to perform distortion correction and pivot calibration on a set of
PointClouds, using the mathematical method described above.

PA2_Prob1.py: Given a distortion calibration data set, computed the “expected” values for C.

PA2_Prob4.py: Given a set of frames in which the EM probe is touching several fiducial points and a corrected pivot
calibration value, correct the distortion in the frames and outputs the locations of the fiducial points in the
tracker base coordinate system.

PA2_Prob5.py: Given an input file with the locations of fiducial points in CT coordinates, calculates and returns
the registration of these points to the points in EM tracker space.

PA2_Prob6.py: Given an input file with arbitrary poses of the probe, calculates the locations of the probe tip
in CT coordinates.

PA2_driver.py: Runs the program and computes the positions of the tip of the probe in CT coordinates with given input
 data specifying the positions of the EM markers on the probe in EM tracker coordinates.

test.py: Contains functions that test basic methods used in other parts of the program to ensure all parts are
working separately and together.

Run configurations:
To test:
python PA2_driver.py test tolerance(optional)

To evaluate input:
python PA2_driver.py “xx-calbody.txt” “xx-calreadings.txt” “xx-empivot.txt” “xx-optpivot.txt”
“xx-ct-fiducials.txt” “xx-em-fiducialss.txt” “xx-EM-nav.txt”

Detailed instructions for running this program are included in our report.
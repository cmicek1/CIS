Julie Shade jshade1
Chris Micek cmicek1

JHU Computer Integrated Surgery
Fall 2016
Programming Assignment 1

File Listing:

Frame.py: Frame class, contains data members for rotation and translation as well as commonly needed frame methods
such as composition and inversion.

PointCloud.py: PointCloud class, a list of points in a given point cloud and commonly needed methods such as
registration between two point clouds, transformation of an entire point cloud by a given frame, and code for
translating input files into lists of PointClouds.

pivot_cal.py: Contains method to perform pivot calibration on a given list of point clouds.

test.py: Contains functions that test basic methods used in other parts of the program to ensure all parts are working.

PA1_Prob4.py: Given a distortion calibration data set, computes the “expected” values for C.

PA1_Prob5.py: Given many poses of an EM probe while it touches the dimple of a pin, finds the position of the top of
the pin in tracker coordinates (performs pivot calibration of EM probe).

PA1_Prob6.py: Performs pivot calibration in the EM tracker frame using optical tracker poses.

PA1_driver.py: Executable- contains main method that parses command line input, as well as a method that runs the
solutions for problems 4-6 and writes them to an output file in the correct format.

PA1_driver.py is the executable for this assignment. Depending on the command line input, it can either run our testing
method or generate an output file given a set of input data files.

Run configurations:
To test:
python PA1_driver.py test tolerance(optional)

To evaluate input:
python PA1_driver.py “xx-calbody.txt” “xx-calreadings.txt” “xx-empivot.txt” “xx-optpivot.txt”

Detailed instructions for running this program are included in our report.
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <conio.h>

using namespace cv;
using namespace std;

Mat cameraMatrix, distCoeffs, rotationVector, rotationMatrix, translationVector, extrinsicMatrix, projectionMatrix, homographyMatrix, inverseHomographyMatrix;


Point point;
vector<Point2d> image_points;
vector<Point3d> world_points;

int main()
{

	FileStorage fs1("intrinsics.yml", FileStorage::READ);

	fs1["camera_matrix"] >> cameraMatrix;
	cout << "Camera Matrix: " << cameraMatrix << endl << endl;

	fs1["distortion_coefficients"] >> distCoeffs;
	cout << "Distortion Coefficients: " << distCoeffs << endl << endl;


	
	image_points.push_back(Point2d(275, 204));
	image_points.push_back(Point2d(331, 204));
	image_points.push_back(Point2d(331, 308));
	image_points.push_back(Point2d(275, 308));

	cout << "Image Points: " << image_points << endl << endl;

	world_points.push_back(Point3d(0.0, 0.0, 0.0));
	world_points.push_back(Point3d(1.775, 0.0, 0.0));
	world_points.push_back(Point3d(1.775, 4.620, 0.0));
	world_points.push_back(Point3d(0.0, 4.620, 0.0));

	cout << "World Points: " << world_points << endl << endl;

	solvePnP(world_points, image_points, cameraMatrix, distCoeffs, rotationVector, translationVector);
	cout << "Rotation Vector: " << endl << rotationVector << endl << endl;
	cout << "Translation Vector: " << endl << translationVector << endl << endl;

	Rodrigues(rotationVector, rotationMatrix);
	cout << "Rotation Matrix: " << endl << rotationMatrix << endl << endl;
	
	hconcat(rotationMatrix, translationVector, extrinsicMatrix);
	cout << "Extrinsic Matrix: " << endl << extrinsicMatrix << endl << endl;

	projectionMatrix = cameraMatrix * extrinsicMatrix;
	cout << "Projection Matrix: " << endl << projectionMatrix << endl << endl;

	double p11 = projectionMatrix.at<double>(0, 0),
		p12 = projectionMatrix.at<double>(0, 1),
		p14 = projectionMatrix.at<double>(0, 3),
		p21 = projectionMatrix.at<double>(1, 0),
		p22 = projectionMatrix.at<double>(1, 1),
		p24 = projectionMatrix.at<double>(1, 3),
		p31 = projectionMatrix.at<double>(2, 0),
		p32 = projectionMatrix.at<double>(2, 1),
		p34 = projectionMatrix.at<double>(2, 3);


	homographyMatrix = (Mat_<double>(3, 3) << p11, p12, p14, p21, p22, p24, p31, p32, p34);
	cout << "Homography Matrix: " << endl << homographyMatrix << endl << endl;

	inverseHomographyMatrix = homographyMatrix.inv();
	cout << "Inverse Homography Matrix: " << endl << inverseHomographyMatrix << endl << endl;

	Mat point2D = (Mat_<double>(3, 1) << image_points[0].x, image_points[0].y, 1);
	cout << "First Image Point" << point2D << endl << endl;

	Mat point3Dw = inverseHomographyMatrix*point2D;
	cout << "Point 3D-W : " << point3Dw << endl << endl;

	double w = point3Dw.at<double>(2, 0);
	cout << "W: " << w << endl << endl;

	Mat matPoint3D;
	divide(w, point3Dw, matPoint3D);

	cout << "Point 3D: " << matPoint3D << endl << endl;

	_getch();
	return 0;
}



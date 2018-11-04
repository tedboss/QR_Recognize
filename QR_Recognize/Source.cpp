#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int getTopAnchor(Point2f A, Point2f B, Point2f C);
float getDistance(Point2f A, Point2f B);

void swapPoints(Point2f& A, Point2f& B);
void swapAnchors(vector<Point2f>& A, vector<Point2f>& B);
void arrangeOutlierPoint(vector<Point2f>& anchor, Point2f center);

Point2f getFarthestPoint(vector<Point2f> anchor, Point2f center);
Point2f getIntersection(Point2f A, Point2f A1, Point2f B, Point2f B1);

Scalar GREEN = Scalar(0, 255, 0);
Scalar BLUE = Scalar(255, 0, 0);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(255, 0, 255);

VideoCapture capture(0);
Mat image, gray, edges;

int main(int argc, char **argv) {
	if (!capture.isOpened()) {
		cerr << "Khong tim thay thiet bi" << endl;
		return -1;
	}

	while (true) {
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy; //Cấp bậc các contours
		vector<vector<Point2f>> anchors; //Chứa 3 neo mã QR (4 đỉnh hình vuông neo)
		vector<Point2f> center_anchors; //Tâm hình vuông neo mã QR
		int count_vertex = 0;

		capture >> image;
		cvtColor(image, gray, CV_RGB2GRAY);
		Canny(gray, edges, 100, 100, 3);

		findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		/*Mã QR gồm 3 neo ở 3 góc trên dưới và bên phải, mỗi neo gồm 1 hình vuông lồng trong 1 viền vuông khác.
		Vì vậy contour ngoài cùng của mỗi neo sẽ có bậc child là 5. Dựa vào đây để định vị 3 neo của mã QR
		*/
		for (int i = 0; i < contours.size(); i++) {
			vector<Point2f> poly_contour;

			//Đưa các contour về dạng đa giác
			approxPolyDP(contours[i], poly_contour, arcLength(contours[i], true) / 100, true);

			if (poly_contour.size() == 4) { //Chỉ xét tứ giác vì neo hình vuông
				int j = i;
				int count = 0;

				while (hierarchy[j][2] != -1) { //Tìm bậc child của contour đang xét
					j = hierarchy[j][2];
					count++;
				}

				if (count >= 5) { //Ghi nhận những contour bậc child bằng 5
					float center_x = (poly_contour[0].x + poly_contour[2].x) / 2;
					float center_y = (poly_contour[0].y + poly_contour[2].y) / 2;
					center_anchors.push_back(Point2f(center_x, center_y));

					anchors.push_back(poly_contour);
					count_vertex++;
				}
			}
		}

		if (count_vertex >= 3) {
			//Tìm neo trên của mã QR và chuyển về đầu vector
			int top = getTopAnchor(center_anchors[0], center_anchors[1], center_anchors[2]);
			swapPoints(center_anchors[top], center_anchors[0]);
			swapAnchors(anchors[top], anchors[0]);

			circle(image, center_anchors[0], 2, GREEN, 5);
			circle(image, center_anchors[1], 2, BLUE, 5);
			circle(image, center_anchors[2], 2, BLUE, 5);

			//Tìm tâm mã QR
			float center_x = (center_anchors[1].x + center_anchors[2].x) / 2;
			float center_y = (center_anchors[1].y + center_anchors[2].y) / 2;
			Point2f center_qr = Point2f(center_x, center_y);

			//Tìm đỉnh ngoài cùng mỗi neo, xếp về đầu vector
			for (int i = 0; i < center_anchors.size(); i++) {
				arrangeOutlierPoint(anchors[i], center_qr);
			}

			//Tìm điểm góc thứ 4 của mã QR (vì chỉ có 3 neo)
			Point2f final_point = getIntersection(
				anchors[1][0],
				getFarthestPoint(anchors[1], center_anchors[0]),
				anchors[2][0],
				getFarthestPoint(anchors[2], center_anchors[0])
			);

			line(image, anchors[0][0], anchors[1][0], YELLOW, 2);
			line(image, anchors[0][0], anchors[2][0], YELLOW, 2);

			line(image, anchors[1][0], final_point, RED, 2);
			line(image, anchors[2][0], final_point, RED, 2);

		}
		namedWindow("Image", 1);
		imshow("Image", image);
		waitKey(1);
	}
	return 0;
}

float getDistance(Point2f A, Point2f B) {
	return(sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2)));
}

//Tìm giao điểm 2 đường thẳng AA1 và BB1
Point2f getIntersection(Point2f A, Point2f A1, Point2f B, Point2f B1) {
	float a1 = (A.y - A1.y) / (A.x - A1.x);
	float b1 = A.y - a1*A.x;

	float a2 = (B.y - B1.y) / (B.x - B1.x);
	float b2 = B.y - a2*B.x;

	float x = (b2 - b1) / (a1 - a2);
	float y = a1*x + b1;

	return Point2f(x, y);
}

//Tìm điểm xa nhất trong mỗi neo (trừ điểm ngoài cùng) để dựng đường thẳng đi qua góc thứ 4 của QR
Point2f getFarthestPoint(vector<Point2f> anchor, Point2f center) {
	float max = getDistance(anchor[1], center);
	int index = 1;
	for (int i = 2; i < anchor.size(); i++) {
		if (getDistance(anchor[i], center) > max) {
			max = getDistance(anchor[i], center);
			index = i;
		}
	}
	return anchor[index];
}

//Đưa điểm ngoài cùng mỗi neo về đầu vector
void arrangeOutlierPoint(vector<Point2f>& anchor, Point2f center) {
	float max = getDistance(anchor[0], center);
	int index = 0;
	for (int i = 1; i < anchor.size(); i++) {
		float temp = getDistance(anchor[i], center);
		if (temp > max) {
			max = temp;;
			index = i;
		}
	}
	swapPoints(anchor[index], anchor[0]);
}

void swapPoints(Point2f& A, Point2f& B) {
	Point2f temp;
	temp = A;
	A = B;
	B = temp;
}

void swapAnchors(vector<Point2f>& A, vector<Point2f>& B) {
	vector<Point2f> temp;
	temp = A;
	A = B;
	B = temp;
}

//Tìm điểm neo giữa trong 3 neo QR (top anchar QR)
int getTopAnchor(Point2f A, Point2f B, Point2f C) {
	float AB = getDistance(A, B);
	float AC = getDistance(A, C);
	float BC = getDistance(C, B);
	float max = AB;
	if (AC > max) max = AC;
	if (BC > max) {
		return 0;
	}
	return max == AC ? 1 : 2;
}
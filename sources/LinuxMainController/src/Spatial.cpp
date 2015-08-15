/*
 * space.cpp
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#include <iomanip>
#include "Spatial.h"

#include "math.h"
#include "Util.h"


ostream& operator<<(ostream& os, const Point& p)
{
	cout << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}

ostream& operator<<(ostream& os, const Rotation& p)
{
	cout << setprecision(0) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}
ostream& operator<<(ostream& os, const Position& p)
{
	cout << setprecision(1) << "(" << p.point.x << "," << p.point.y << "," << p.point.z << "|" << p.rot.x << "," << p.rot.y << "," << p.rot.z << ")";
    return os;
}

ostream& operator<<(ostream& os, const TimedPosition& p)
{
	cout << setprecision(1) << "(" << p.atTime << "ms|" << p.pos.point.x << "," << p.pos.point.y << "," << p.pos.point.z << "|" << p.pos.rot.x << "," << p.pos.rot.y << "," << p.pos.rot.z << ")";
    return os;
}

ostream& operator<<(ostream& os, const TimedPositionList& p)
{
	for (int i = 0;i<p.size();i++) {
		os << setprecision(1) << "   " << i << ":" <<((TimedPositionList&)p).get(i) << endl;
	}
    return os;
}

Point::Point() {
	null();
}

Point::Point(const Point& p) {
	x = p.x;
	y = p.y;
	z = p.z;
}


void Point::null() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
};

Point::Point(float xP,float yP, float zP) {
	x = xP;
	y = yP;
	z = zP;
}

void Point::set(float pX, float pY,float pZ) {
	x = pX;
	y = pY;
	z = pZ;
}


bool Point::isNull() {
	return (x == 0.0) && (y == 0.0) && (z == 0.0);
};



void Point::print() const {
	if (debug)
		cout << fixed << setprecision(2) << "(" << x << "," << y << "," << z << ")";
}

void Point::println()  const{
	print();
	if (debug)
		cout << endl;
}

void Point::print(const string msg) const {
	if (debug)
		cout << msg << "=";
	print();
}

void Point::println(string msg) const {
	print(msg);
	if (debug)
		cout << endl;
}

void Point::translate(const Point& pPoint) {
	x += pPoint.x;
	y += pPoint.y;
	z += pPoint.z;
}

void Point::mirrorAt(const Point& pMirror,float scale) {
	x = pMirror.x + (pMirror.x-x)*scale;
	y = pMirror.y + (pMirror.y-y)*scale;
	z = pMirror.z + (pMirror.z-z)*scale;
}

void Point::mirrorAt(const Point& pMirror) {
	x = pMirror.x + (pMirror.x-x);
	y = pMirror.y + (pMirror.y-y);
	z = pMirror.z + (pMirror.z-z);
}

float Point::distance(const Point& pPoint) const {
	return sqrt((pPoint.x-x)*(pPoint.x-x) + (pPoint.y-y)*(pPoint.y-y)+  (pPoint.z-z)*(pPoint.z-z));
}
float Point::length() const{
	return sqrt(x*x + y*y + z*z);
}

float Point::angleToDegree(const Point& pPoint) const {
	return degrees(acosFast(scalarProduct(pPoint)/(length() * pPoint.length())));
}

float Point::scalarProduct(const Point& pPoint) const {
	return x*pPoint.x + y*pPoint.y + z*pPoint.z;
}

Point Point::orthogonalProjection(const Point& pLine) const {
	// scalarProduct(a,b) / scalarProduct( line, line ) * line
	Point result = pLine;
	result *= scalarProduct(pLine)/ pLine.scalarProduct(pLine);
	return result;
}

Point Point::orthogonalProjection(const Point& pLineA, const Point& pLineB) const {
	Point translate(pLineA);
	Point selfTranslated=(*this);
	selfTranslated-= translate;
	Point line = pLineB;
	line -= translate;
	Point result = selfTranslated.orthogonalProjection(line);
	result += translate;
	return result;
}

Point Point::getPointOfLine(float ratio, const Point& target) {
	ratio = constrain(ratio,0.0,1.0);
	Point result(*this);
	result.x += ratio*(float)(target.x - x);
	result.y += ratio*(float)(target.y - y);
	result.z += ratio*(float)(target.z - z);
	return result;
}


void Position::setPoint (const Point& pPoint) {
	point = pPoint;
}

void Position::setRotation (const Rotation& pRot) {
		rot = pRot;
}

Position Position::getPointOfLine(float ratio, const Position& target) {
	ratio = constrain(ratio,0.0,1.0);
	Position result(*this);
	result.point.x += ratio*(float)(target.point.x - point.x);
	result.point.y += ratio*(float)(target.point.y - point.y);
	result.point.z += ratio*(float)(target.point.z - point.z);
	result.rot.x += ratio*(float)(target.rot.x - rot.x);
	result.rot.y += ratio*(float)(target.rot.y - rot.y);
	result.rot.z += ratio*(float)(target.rot.z - rot.z);

	return result;
}


void Position::print() const {
	point.print("t");
	rot.print("r");
}
void Position::println() const {
	print();
	if (debug) cout << endl;
}
void Position::println(const string msg) const {
	print(msg);
	if (debug) cout << endl;
}
void Position::print(const string msg) const {
	if (debug)
		cout << msg << "={";
	point.print();
	if (debug)
		cout << ",";
	rot.print();
	if (debug)
		cout << "}";
}

float Position::distance(const Position& pOther) const {
	return sqrt(
			(pOther.point.x - point.x)*(pOther.point.x - point.x)+
			(pOther.point.y - point.y)*(pOther.point.y - point.y)+
			(pOther.point.z - point.z)*(pOther.point.z - point.z));
}

void Position::mirrorAt(const Position& pMirror, float scale)  {
	point.mirrorAt(pMirror.point,scale);
	rot.mirrorAt(pMirror.rot,scale);
}

void Position::mirrorAt(const Position& pMirror) {
	point.mirrorAt(pMirror.point);
	rot.mirrorAt(pMirror.rot);
}

void Matrix3x3::setRotationMatrixX(float degree) {
	// set rotation matrix to rotate by x-axis
	float s,c;
	if (degree == 0.0) {
		s = 0.0;
		c = 1.0;
	} else {
		s = sinFast(radians(degree));
		c = cosFast(radians(degree));
	}
	// mat[3][3] = { {1,0,0}, {0,c,s}, {0,-s,c} };
	mat[0][0] = 1.0;
	mat[0][1] = 0.0;
	mat[0][2] = 0.0;

	mat[1][0] = 0.0;
	mat[1][1] = c;
	mat[1][2] = s;

	mat[2][0] = 0.0;
	mat[2][1] = -s;
	mat[2][2] = c;
}

void Matrix3x3::setRotationMatrixY(float degree) {
	float s,c;
	if (degree == 0.0) {
		s = 0.0;
		c = 1.0;
	} else {
		s = sinFast(radians(degree));
		c = cosFast(radians(degree));
	}
	// mat[3][3] = { {1,0,0}, {0,c,s}, {0,-s,c} };

	mat[0][0] = c;
	mat[0][1] = 0.0;
	mat[0][2] = -s;

	mat[1][0] = 0.0;
	mat[1][1] = 1.0;
	mat[1][2] = 0.0;

	mat[2][0] = s;
	mat[2][1] = 0.0;
	mat[2][2] = c;
}
void Matrix3x3::setRotationMatrixZ(float  degree) {
	float s,c;
	if (degree == 0.0) {
		s = 0.0;
		c = 1.0;
	} else {
		s = sinFast(radians(degree));
		c = cosFast(radians(degree));
	}

	mat[0][0] = c;
	mat[0][1] = s;
	mat[0][2] = 0.0;

	mat[1][0] = -s;
	mat[1][1] = c;
	mat[1][2] = 0.0;

	mat[2][0] = 0.0;
	mat[2][1] = 0.0;
	mat[2][2] = 1.0;
}

void Matrix3x3::multiply(const Matrix3x3& a, const Matrix3x3& b) {
	null();
	for(int  i = 0; i<3; i++) {
		float rYi0 = b.mat[i][0];
		float rYi1 = b.mat[i][1];
		float rYi2 = b.mat[i][2];

		for(int j = 0; j<3; j++) {
			float rXkj = a.mat[0][j];
			if ((rXkj != 0.0) && (rYi0 != 0.0))
				mat[i][j] += rYi0*rXkj;

			rXkj = a.mat[1][j];
			if ((rXkj != 0.0) && (rYi1 != 0.0))
				mat[i][j] += rYi1*rXkj;

			rXkj = a.mat[2][j];
			if ((rXkj != 0.0) && (rYi2 != 0.0))
				mat[i][j] += rYi2*rXkj;

		}
	}
}


void Matrix3x3::computeRotationMatrix(const Rotation& rotation) {
	// set rotation matrix to rotate by x-axis
	Matrix3x3 rotateByX;
	rotateByX.setRotationMatrixX( rotation.x);

	// set rotation matrix to rotate by y-axis
	Matrix3x3 rotateByY;
	rotateByY.setRotationMatrixY(rotation.y);

	// set rotation matrix to rotate by z-axis
	Matrix3x3 rotateByZ;
	rotateByZ.setRotationMatrixZ(rotation.z );

	// compute rotation matrix rotating by x and y by multiplying both rotation matrix
	Matrix3x3 rotateByYX;
	rotateByYX.multiply(rotateByY,rotateByX);

	// compute rotation matrix rotating by x,y,z by multiplying both rotation matrix
	multiply(rotateByZ,rotateByYX);
}


void Matrix3x3::rotatePoint(Point &point) {
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;

	for(int k = 0; k<3; k++) {
		float  coord = point.get(k);
		if (coord != 0.0) {
			float matxk = mat[0][k];
			if (matxk != 0.0)
				x  += matxk*coord;
			matxk = mat[1][k];
			if (matxk != 0.0)
				y  += matxk*coord;
			matxk = mat[2][k];
			if (matxk != 0.0)
				z  += matxk*coord;
		}
	}
	point.x = x;
	point.y = y;
	point.z = z;

}

void Matrix3x3::null() {
	for (int i = 0;i<3;i++)
		for (int j = 0;j<3;j++)
			mat[i][j] = 0;
}

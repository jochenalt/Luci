/*
 * space.h
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#ifndef SPACE_H_
#define SPACE_H_


#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#include <iostream>
#include <string>
#include "setup.h"
#include "Util.h"
#include "LEDController.h"

class Point {
	 friend ostream& operator<<(ostream&, const Point&);
	public:
		Point();
		Point(const Point& p);
		Point(float xP,float yP, float zP);
		void translate(const Point& pPoint);
		void mirrorAt(const Point& pPoint, float scale);
		void mirrorAt(const Point& pPoint);
		void set(float pX, float pY,float pZ);
		void null();
		bool isNull();

		float get(int idx) {
			switch (idx) {
				case X_INDEX:
					return x;break;
				case Y_INDEX:
					return y;break;
				case Z_INDEX:
					return z;break;
				default:
				break;
			}
			cerr << "get(" << idx << ") invalid." << endl;
			return x;
		}


		void operator= (const Point& p) {
			x = p.x;
			y = p.y;
			z = p.z;
		}

		void operator+= (const Point& p) {
			x += p.x;
			y += p.y;
			z += p.z;
		}

		void operator-= (const Point& p) {
			x -= p.x;
			y -= p.y;
			z -= p.z;
		}

		void operator*= (const float f) {
			x *= f;
			y *= f;
			z *= f;
		}

		void operator/= (const float f) {
			float xrf= 1.0/f;
			x *= xrf;
			y *= xrf;
			z *= xrf;
		}

		Point operator- (const Point& p) const{
			Point result= (*this);
			result -= p;
			return result;
		}

		Point operator+ (const Point& p) const{
			Point result= (*this);
			result += p;
			return result;
		}

		Point operator/ (const float f) const{
			Point result= (*this);
			result*= (1./f);
			return result;
		}

		Point operator* (const float f) const{
			Point result= (*this);
			result*=f;
			return result;
		}


		bool operator==(const Point& pos) {
			return (x == pos.x) && (y == pos.y) && (z == pos.z);
		};

		bool operator!=(const Point& pos) {
			return !((*this) == pos);
		};

		float distance(const Point& p) const;
		float length() const;
		float angleToDegree(const Point& pPoint) const;
		float scalarProduct(const Point& pPoint) const;
		Point orthogonalProjection(const Point& pLine) const;
		Point orthogonalProjection(const Point& pLineA, const Point &pLineB) const;
		Point getPointOfLine(float ratio, const Point& target);

		void print() const;
		void print(const std::string msg) const;
		void println() const;
		void println(const std::string msg) const;
		// members of a point, needs to be public for access (plus [] operator)
		float x;
		float y;
		float z;
};

class Rotation : public Point {
	 friend ostream& operator<<(ostream&, const Rotation&);
	public:
		Rotation () : Point (0,0,0){};
		Rotation(float xP,float yP, float zP): Point(xP,yP,zP) {
			x = xP;
			y = yP;
			z = zP;
		}

		Rotation(const Rotation& p) : Point(p) {
			x= p.x;
			y= p.y;
			z= p.z;
		};
		void operator=(const Rotation& rot) {
			x= rot.x;
			y= rot.y;
			z= rot.z;
		};
		void operator+=(const Rotation& rot) {
			x += rot.x;
			y += rot.y;
			z += rot.z;
		};

		void operator*=(float f) {
			x *= f;
			y *= f;
			z *= f;
		};
		void operator/=(float f) {
			x /= f;
			y /= f;
			z /= f;
		};
		Rotation operator*(const float f) const {
			Rotation result(*this);
			result.x *= f;
			result.y *= f;
			result.z *= f;
			return result;
		};
		Rotation operator/(const float f) const {
			Rotation result(*this);
			result *= (1./f);
			return result;
		};

		Rotation operator+(const Rotation& rot) const {
			Rotation result(*this);
			result += rot;
			return result;
		};

		Rotation operator-(const Rotation& rot) const {
			Rotation result(*this);
			result -= rot;
			return result;
		};
		bool operator==(const Rotation& pos) {
			return (x == pos.x) && (y == pos.y) && (z == pos.z);
		};

		bool operator!=(const Rotation& pos) {
			return !((*this) == pos);
		};

};

// Tensor represents a position and
class Position {
	 friend ostream& operator<<(ostream&, const Position&);
	public:
		Position() { null(); };

		Position(const Position& pos) {
			point = pos.point;
			rot = pos.rot;
		 };
		Position(const Point& pTrans, const Rotation& pRot) {
			point = pTrans;
			rot = pRot;
		 };

		bool isNull() {
			return point.isNull() && rot.isNull();
		}
		void null() {
			point.null();
			rot.null();
		}

		Position getPointOfLine(float ratio, const Position&  target);

		void operator=(const Position& pos) {
			point = pos.point;
			rot = pos.rot;
		};

		bool operator==(const Position& pos) {
			return (point == pos.point) && (rot == pos.rot);
		};

		bool operator!=(const Position& pos) {
			return !((*this) == pos);
		};

		void operator+=(const Position& pos) {
			point += pos.point;
			rot += pos.rot;
		};
		void operator-=(const Position& pos) {
			point -= pos.point;
			rot -= pos.rot;
		};

		void operator*=(const float x) {
			point *= x;
			rot *= x;
		};
		void operator/=(const float x) {
			point /= x;
			rot /= x;
		};

		Position operator*(float x) const {
			Position result(*this);
			result *= x;
			return result;
		};

		Position operator/(float x) const {
			Position result(*this);
			result /= x;
			return result;
		};
		Position operator+(const Position& pos) const {
			Position result(*this);
			result += pos;
			return result;
		};

		Position operator-(const Position& pos) const {
			Position result(*this);
			result -= pos;
			return result;
		};


		void mirrorAt(const Position& pMirror, float scale);
		void mirrorAt(const Position& pMirror);

		float distance(const Position& pOther) const;

		void print() const;
		void println() const;
		void println(const string msg) const;
		void print(const string msg) const;

		void setPoint (const Point& pPoint);
		void setRotation (const Rotation& pRot);
		Point& getPoint ();
		Rotation& getRotation ();

		Point point;
		Rotation rot;
};


class Matrix3x3 {
	public:
		Matrix3x3() {
			null();
		}

		void setRotationMatrixX(float degree);
		void setRotationMatrixY(float degree);
		void setRotationMatrixZ(float degree);
		void multiply(const Matrix3x3& a,const Matrix3x3& b);

		void null();
		void computeRotationMatrix(const Rotation& rotation);

		void rotatePoint(Point &point);
		private:
			float  mat[3][3];
};



#define MAX_TENSOR_CAPACITY 256 // sufficient to control 25s ahead if we have 10 bezierpoints per seconds,

enum InterpolationType {CUBIC_BEZIER_INTERPOLATION, LINEAR_INTERPOLATION };

class TimedPosition {
	 friend ostream& operator<<(ostream&, const TimedPosition&);

	public:
		TimedPosition(const TimedPosition& tpos) {
			pos = tpos.pos;
			atTime = tpos.atTime;
			interpType = tpos.interpType;
			doStareAtFace = tpos.doStareAtFace;
			doLookAtFace = tpos.doLookAtFace;
			lookAtFace = tpos.lookAtFace;
			lamp = tpos.lamp;
		}

		TimedPosition(unsigned long pAtTime, const Position& pPos) {
			pos = pPos;
			atTime = pAtTime;
			interpType = CUBIC_BEZIER_INTERPOLATION;
			doStareAtFace = false;
			doLookAtFace = false;
			lookAtFace.null();
			lamp.clear();
		}
		TimedPosition(unsigned long pPointInTime, const Point& trans, const Rotation& rot) {
			pos.point = trans;
			pos.rot = rot;
			atTime = pPointInTime;
			interpType = CUBIC_BEZIER_INTERPOLATION;
			doStareAtFace = false;
			doLookAtFace = false;
			lookAtFace.null();
			lamp.clear();
		}

		TimedPosition() {null();};

		bool laterThan(unsigned long time) { return atTime> time; };
		bool earlierThan(unsigned long time) { return atTime<= time; };

		bool isStareAtFace() { return doStareAtFace; };
		void setStareAtFace(bool pLookAtPosition) { doStareAtFace = pLookAtPosition; };
		bool isLookAtFace() { return doLookAtFace; };
		Point getLookAtFacePoint() { return lookAtFace; };
		void setLookAtFace(const Point& pLookAtFace) { doLookAtFace = true;; lookAtFace = pLookAtFace;};
		void setLamp(const LEDPattern& pattern) { lamp = pattern; };
		LEDPattern& getLamp() { return lamp; };


		InterpolationType getInterpolationType() { return interpType; };
		void setInterpolationType(InterpolationType pIntPType) { interpType = pIntPType; };
		void operator=(const TimedPosition &t) {
			pos = t.pos;
			atTime = t.atTime;
			interpType = t.interpType;
			doStareAtFace = t.doStareAtFace;
			doLookAtFace = t.doLookAtFace;
			lookAtFace = t.lookAtFace;
			lamp = t.lamp;
		}


		void null() {
			pos.null();
			atTime = 0;
			interpType = CUBIC_BEZIER_INTERPOLATION;
			doStareAtFace = false;
			doLookAtFace = false;
			lookAtFace.null();
			lamp = LEDPattern();
		}
		bool isNull() {
			return atTime==0;
		}

		Position getPointOfLine(unsigned long time, const TimedPosition& pTarget) {
			float t = intervalRatio(atTime,time, pTarget.atTime);
			Position result(this->pos);
			return pos.getPointOfLine(t, pTarget.pos);
		}

		void println(const string msg) const {
			print(msg);
			cout << endl;
		}
		void print(const string msg) const {
			cout << msg << "(t=" << atTime<< " ";
			pos.print("tensor");
			cout << ")";
		}

		void print() const {
			print("");
		}

		void println() const {
			println("");
		}
		unsigned long timeBetween(const TimedPosition& b) {
			if (b.atTime > atTime)
				return b.atTime - atTime;
			else
				return atTime-b.atTime;

		}

		void setFadeInOut(int msIn, int msOut, int dutyOff, int dutyOn, bool pRepeat) {
			getLamp().setFadeInOut(atTime,msIn, msOut,LEDPattern::OFF,LEDPattern::DARK,pRepeat);
		}

		void setFadeInOut(int msIn, int msOut, int dutyOff, int dutyOn) {
			getLamp().setFadeInOut(atTime,msIn, msOut,LEDPattern::OFF,LEDPattern::DARK);
		}

		void setFadeOut(int time, int duty) {
			getLamp().setFadeOut(atTime,time, duty);
		}
		void setFadeIn(int time, int duty) {
			getLamp().setFadeIn(atTime,time, duty);
		}
		void setConstantDuty(int time, int duty) {
			getLamp().setConstantDuty(atTime,time, duty);
		}

	Position pos;
	unsigned long atTime; // [ms];
	InterpolationType interpType;  // indicates linear interpolation (short movements) vs. bezier curves (longer move).
	bool doStareAtFace;				// stare at the face during movement
	bool doLookAtFace;				// look at face when the movement starts
	Point lookAtFace;
	LEDPattern lamp;
};

class TimedPositionList {
	 friend ostream& operator<<(ostream&, const TimedPositionList&);

public:
	TimedPositionList () {
		tensorListSize = 0;
	};

	void set(int idx, const TimedPosition & tensor) {
		tensorList[idx] = tensor;
	}

	void add(const TimedPosition& tensor) {
		if (tensorListSize < MAX_TENSOR_CAPACITY) {
			set(tensorListSize,tensor);
			tensorListSize++;
		} else {
			cerr << "TimedTensorList::add list is full" << endl;
		}
	}

	unsigned long timeBetween(int idx1, int idx2) {
		if (idx1<tensorListSize && idx2<tensorListSize)
			return get(idx1).timeBetween(get(idx2));
		else
			return 0;
	}
	int getLastIndex() {
		if (tensorListSize > 0)
			return tensorListSize-1;
		return 0;
	}

	void removeLaterThan(unsigned long approachingTime) {
		unsigned long time = approachingTime;
		while ((tensorListSize>0) && (tensorList[tensorListSize-1].atTime >= time))
			tensorListSize--;
	}

	void clear() {
		tensorListSize = 0;
	}
	int size() const {
		return tensorListSize;
	}
	void remove(int idx) {
		if (idx < tensorListSize) {
			for (int i = idx;i<tensorListSize;i++) {
				tensorList[i] = tensorList[i+1];
			}
			tensorListSize--;
		}
	}
	TimedPosition& get(int idx)  {
		if (idx >= tensorListSize) {
			cerr << "tensor list[" << idx << "] out of bounds. Size=" << tensorListSize << endl;
		}
		return tensorList[idx];
	}
	void print(string msg, bool newline) {
		if (debug) {
			if (!msg.empty())
				cout << msg << endl;;

			for (int i = 0;i<size();i++) {
				if (debug)
					cout << "   " << i << "(" << get(i).atTime << "ms)=";
				get(i).pos.println("");
			}

			if (newline)
				cout << endl;
		}
	}
	void println(string msg) {
		print(msg, true);
	}
	void print(string msg) {
		print(msg, false);
	}

	void println() {
		print("", true);
	}
	void print() {
		print("", false);
	}

private:
	TimedPosition tensorList[MAX_TENSOR_CAPACITY];
	int tensorListSize;
};


#endif /* SPACE_H_ */

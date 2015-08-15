/*
 * BezierCurve.cpp
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#include <iostream>
#include <string>

#include "BezierCurve.h"


#define BEZIER_CURVE_SUPPORT_POINT_SCALE (1./3.)

bool debugBezier = false;
bool useDynamicBezierSupportPoint = true;
bool useLinearOrientation = true;


float BezierCurve::computeBezier(InterpolationType ipType, float a, float supportA,  float b, float supportB, float t) {
	if ((t>1.0) || (t<0.0)) {
		cerr << "BUG t!=[0..1]:" << t << endl;
	}

	if (ipType == LINEAR_INTERPOLATION)
		return (1-t)*a + t*b;
	else
		return (1-t)*(1-t)*(1-t)*a + 3*t*(1-t)*(1-t)*supportA + 3*t*t*(1-t)*supportB + t*t*t*b;
}

Position BezierCurve::computeBezier(InterpolationType ipType, const Position& a, const Position& supportA,  const Position& b, const Position& supportB, float t) {
	Position result;
	result.point.x = computeBezier(ipType,a.point.x, supportA.point.x, b.point.x, supportB.point.x,t);
	result.point.y = computeBezier(ipType,a.point.y, supportA.point.y, b.point.y, supportB.point.y,t);
	result.point.z = computeBezier(ipType,a.point.z, supportA.point.z, b.point.z, supportB.point.z,t);
	if (useLinearOrientation) {
		result.rot.x = computeBezier(LINEAR_INTERPOLATION,a.rot.x, supportA.rot.x, b.rot.x, supportB.rot.x,t);
		result.rot.y = computeBezier(LINEAR_INTERPOLATION,a.rot.y, supportA.rot.y, b.rot.y, supportB.rot.y,t);
		result.rot.z = computeBezier(LINEAR_INTERPOLATION,a.rot.z, supportA.rot.z, b.rot.z, supportB.rot.z,t);
	} else {
		result.rot.x = computeBezier(ipType,a.rot.x, supportA.rot.x, b.rot.x, supportB.rot.x,t);
		result.rot.y = computeBezier(ipType,a.rot.y, supportA.rot.y, b.rot.y, supportB.rot.y,t);
		result.rot.z = computeBezier(ipType,a.rot.z, supportA.rot.z, b.rot.z, supportB.rot.z,t);
	}
	return result;
}


Position BezierCurve::getPointOfLine(unsigned long time) {
	float t = intervalRatio(a.atTime,time, b.atTime);
	TimedPosition result = getCurrent(t);
	return result.pos;
}

TimedPosition BezierCurve::getCurrent(float t) {
	InterpolationType interpolType = a.getInterpolationType();
	TimedPosition result;
	result.pos = computeBezier(interpolType,a.pos,supportA,b.pos, supportB, t);
	result.atTime = a.atTime + t*(b.atTime-a.atTime);
	return result;
}


void BezierCurve::set(TimedPosition& pPrev, TimedPosition& pA, TimedPosition& pB, TimedPosition& pNext) {
	Position supportPointB(pB.pos);
	if (!pNext.isNull()) {
		supportPointB =  getSupportPoint(pA,pB,pNext);
		// cout << "BezierCurve::set.getSupportPointB(" << pA << "," << pB <<"," << pNext << ")=" << supportPointB << endl;

	}

	Position supportPointA(pA.pos);
	if (!pNext.isNull()) {
		supportPointA =  getSupportPoint(pB,pA,pPrev);
		// cout << "BezierCurve::set.getSupportPointA(" << pB << "," << pA <<"," << pPrev << ")=" << supportPointA << endl;
	}
	a = pA;
	b = pB;
	supportA = supportPointA;
	supportB = supportPointB;

	active = true;
}

void BezierCurve::amend(float t, TimedPosition& pNewB, TimedPosition& pNext) {

	// compute current and next curve point. This is used later on to compute
	// the current bezier support point which has the same derivation, assuming
	// that we start from the current position
	static float dT = 0.01;										// arbitrary value, needed to compute the current support points
	TimedPosition current = getCurrent(t);						// compute curve point for t(which is now)
	TimedPosition currentPoint_plus_dT = getCurrent(t + dT);	// and for t+dT (which is an arbitrary point in time)

	// since we already passed a small piece of t, the next interval is now smaller (actually 1-t)
	// Later on the need dT that has the same distance but relatively to the (shorter) remaining piece
	static float dTNew = dT/(1.0-t);	// compute new dt assuming that we start from current position

	// now we set the new bezier curve. The current position becomes the new starting point
	// and the end point and end support point remains the same. But we need a new support point,
	// which we get out of the current tangent of the current and next, enhanced to the new time frame dTNew
	//
	// the following equation is derived out of the condition
	// 		Bezier (current, current-support,end,end-support, dT) = currentPoint-for-dT
	//
	// out of that, we separate the Bezier terms
	// 		Bezier(0,1,0,0,dT)*supportA = currentPoint-for-dT - Bezier(current,0,end,end-support)
	//
	// and end up in the equation
	// 		supportA = (currentPoint-for-dTNew - Bezier(current,0,end,end-support, dTNew) / Bezier(0,1,0,0,dTNew)
	float supportABezierTermRezi = 1.0/(computeBezier(CUBIC_BEZIER_INTERPOLATION, 0, 1, 0,0,dTNew)); // take the bezier term of support a only

	Position newSupportA =
			(currentPoint_plus_dT.pos-computeBezier(CUBIC_BEZIER_INTERPOLATION, current.pos, Position(), b.pos, supportB, dTNew))*supportABezierTermRezi;

	// set the new curve
	Position newSupportPointB;
	if (pNext.isNull())
		newSupportPointB = pNewB.pos;
	else
		newSupportPointB =  getSupportPoint(current,pNewB,pNext);

	a = current; // this sets current time as new point in time as well
	a.setInterpolationType(pNewB.getInterpolationType());
	b = pNewB;
	b.setInterpolationType(pNewB.getInterpolationType());

	supportB = newSupportPointB;
	supportA = newSupportA;

	// alternative with small jump in curve
	// set(a,currentTensor, pNewB, pNext);
}

// compute b's support point
Position  BezierCurve::getSupportPoint(const TimedPosition& a, const TimedPosition& b, const TimedPosition& c) {
	// support point for bezier curve is computed by
	// BC' = mirror BC at B with length(BC') = length(AB)

	// mirror C at B = mirrorC
	Position mirroredC(c.pos);
	mirroredC.mirrorAt(b.pos);

	// compute length of BmirrorC and AB and translate point along BmirrorC such that its length equals len(AB)
	float lenBmC = mirroredC.distance(b.pos);
	float lenAB = a.pos.distance(b.pos);
	Position mirroredNormedC (b.pos);
	Position t  = b.pos - mirroredC;

	if (lenBmC > 1 /* [mm] */) {
		t *=lenAB/lenBmC;
		mirroredNormedC  -= t;
	}// otherwise mirroredNormedC equals b (at least it is very close)
	// compute the middle point of A and mirrored C
	Position midOfA_mC = (a.pos + mirroredNormedC) * 0.5;

	// if the speed of the current and the next piece is the same, take 1/3 as support point distance
	// if the speed is doubled, take the 0.66 as support point
	// if the speed is halfed, take end point as support point

	// ratio of speed
	float speedAB = 0, speedBC = 0;
	if (b.atTime != a.atTime)
		speedAB = lenAB/(b.atTime-a.atTime);
	float lenBC = a.pos.distance(b.pos);
	if (c.atTime != b.atTime)
		speedBC = lenBC/(c.atTime-b.atTime);

	float ratioBCcomparedToAB = 1.0;
	if (useDynamicBezierSupportPoint) {
		if (absFloat(speedAB) > 1.0) {
			ratioBCcomparedToAB = speedBC/speedAB;
			ratioBCcomparedToAB = constrain(ratioBCcomparedToAB,0.2,2.0);
		}
	}

	// now move the point towards B such that its length is fine
	t = b.pos - midOfA_mC;
	float lent = midOfA_mC.distance(b.pos);
	if (lent > 1)
		t *= BEZIER_CURVE_SUPPORT_POINT_SCALE*ratioBCcomparedToAB*lenAB/lent;
	else {
		t.null();
	}
	Position supportB = b.pos - t;

	// all this is done for the position only,
	// the rotation becomes the point itself (dont have a good model yet for beziercurves of orientations)
	supportB.rot = b.pos.rot;


	return supportB;
}


float BezierCurve::distance(float dT1, float dT2) {
	TimedPosition last = getCurrent(dT1);
	TimedPosition prev = getCurrent(dT2);
	return last.pos.point.distance(prev.pos.point);
}

void BezierCurve::print() const {
	if (debugBezier)
		cout << "{";
	a.print("A");
	supportA.print("A'");
	supportB.print("B'");
	b.print("B");
	if (debugBezier)
		cout << "}" << endl;
}


/*
 * BezierCurve.h
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#ifndef BEZIERCURVE_H_
#define BEZIERCURVE_H_

#include "BezierCurve.h"
#include <iostream>

#include "Spatial.h"
#include "Util.h"

class BezierCurve : public Printable {
	public:
		BezierCurve() {
			active = false;
		};
		bool isActive() {
			return active;
		}

		void reset() {
			active = false;
			a.null();
			supportA.null();
			b.null();
			supportB.null();
		}


		void set(TimedPosition& pPrev, TimedPosition& pA, TimedPosition& pB, TimedPosition& pNext);
		void amend(float t, TimedPosition& pB, TimedPosition& pNext);

		TimedPosition getCurrent(float t);
		Position getPointOfLine(unsigned long time);
		Position getSupportPoint(const TimedPosition& a, const TimedPosition& b, const TimedPosition& c);

		float distance(float dT1, float dT2);
		void patchB(const TimedPosition& pB, const Position& pSupportB) {
			supportB = pSupportB;
			b = pB;
		}
		void print() const;
		TimedPosition& getStart() { return a; };
		TimedPosition& getEnd() { return b; };

	private:
		float computeBezier(InterpolationType ipType,float a,float supportA,  float b, float supportB, float t);
		Position computeBezier(InterpolationType ipType, const Position& a, const Position& supportA,  const Position& b, const Position& supportB, float t);

		TimedPosition a;
		Position supportA;
		TimedPosition b;
		Position supportB;

		bool active;
};

#endif /* BEZIERCURVE_H_ */

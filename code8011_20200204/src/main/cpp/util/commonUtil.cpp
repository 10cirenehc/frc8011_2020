	#include "util/commonUtil.h"
	/** @return 10% deadband */
	double DB(double axisVal) {
		if (axisVal < -0.10)
			return (axisVal+0.1)*1.11;
		if (axisVal > +0.10)
			return (axisVal-0.1)*1.11;
		return 0;
	}
	/** @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
	}
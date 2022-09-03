// SDS011 dust sensor PM2.5 and PM10
// ---------------------------------
//
// By R. Zschiegner (rz@madavi.de)
// April 2016
//
// Documentation:
//		- The iNovaFitness SDS011 datasheet
//

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class SDS011 {
	public:
		SDS011(void);
		void begin(Uart *uart);
		int read(uint16_t *p25, uint16_t *p10);
		void sleep();
		void wakeup();
	private:
		Uart *sds_data;		
};

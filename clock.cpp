#include "clock.h"

#include "application.h"

clock* clock::_clock = NULL;

#include <windows.h>
#include <mmsystem.h>

static double qpcFrequency;

unsigned long systemClock() { 
	__asm { 
		rdtsc; 
	} 
}

int64_t clock::getclock() { return int64_t(systemClock()); }

// updates the global frame information. should be called once per frame.
void clock::update() {

	// advance the frame number.
	if ( !_application->testflags(application_paused) ) { m_frame++; }

	// update the timing information.
	int64_t currenttimestamp = 0;
	QueryPerformanceCounter((LARGE_INTEGER*)&currenttimestamp);

	m_last_frame_seconds      = float(currenttimestamp - m_last_frame_timestamp) * m_secondspertick;
	m_last_frame_milliseconds = m_last_frame_seconds*1000.0f;
	m_last_frame_timestamp    = currenttimestamp;



	// update the tick information.
	uint64_t thisClock       = getclock();
	m_last_frame_clockticks  = thisClock - m_last_frame_clockstamp;
	m_last_frame_clockstamp  = thisClock;

	// update the RWA frame rate if we are able to.
	if (m_frame > 1) {
		if (m_average_frame_duration <= 0) {
			m_average_frame_duration = (double)m_last_frame_milliseconds;
		}
		else {
			// RWA over 100 frames.
			m_average_frame_duration *= 0.99;
			m_average_frame_duration += 0.01 * (double)m_last_frame_milliseconds;

			// invert to get fps
			m_fps = (float)(1000.0/m_average_frame_duration);
		}
	}
}

void clock::init() {

	int64_t tickspersecond = 0;
	QueryPerformanceFrequency((LARGE_INTEGER*)&tickspersecond);
	m_secondspertick = 1.0f / (float)tickspersecond;
	QueryPerformanceCounter((LARGE_INTEGER*)&m_last_frame_timestamp);

	m_frame = 0;

	m_last_frame_milliseconds = 0;
	m_last_frame_seconds =0.0f;

	m_last_frame_clockstamp = getclock();
	m_last_frame_clockticks = 0;

	m_average_frame_duration = 0;
	m_fps = 0;
}

#pragma once

#include "application_header.h"

struct clock {

	/** seconds per clock tick */
	float m_secondspertick;

	/** the current render frame. this simply increments */
	int64_t m_frame;

	/** the timestamp when the last frame ended */
	int64_t m_last_frame_timestamp;

	/** the duration of the last frame in milliseconds.*/
	float m_last_frame_milliseconds;

	/** the duration of the last frame in seconds */
	float m_last_frame_seconds;

	/** the clockstamp of the end of the last frame */
	int64_t m_last_frame_clockstamp;

	/** the duration of the last frame in clock ticks */
	int64_t m_last_frame_clockticks;

	/**
	* this is a recency weighted average of the frame time, calculated
	* from frame durations
	*/
	double m_average_frame_duration;

	/**
	* the reciprocal of the average frame duration giving the mean
	* fps over a recency weighted average.
	*/
	float m_fps;

	/**
	* updates the timing system, should be called once per frame
	*/
	void update();

	/** initialises the frame information system */
	void init();

	/** gets the clock ticks since process start. */
	static int64_t getclock();

	static clock* _clock;
};
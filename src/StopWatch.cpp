/*
********************************************************************************
MIT License

Copyright(c) 2018 Christopher Brandt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************
*/
#include <iostream>

#include "StopWatch.h"

StopWatch::StopWatch(unsigned int predictedMeasurements, unsigned int maxMeasurements)
	: m_maxMeasurements(maxMeasurements),
	m_curTStart(),
	m_running(false),
	m_measurements(0)
{
	m_measurements.reserve(predictedMeasurements);
}

void StopWatch::startStopWatch()
{
	if (m_measurements.size() < m_maxMeasurements) {
		m_running = true;
		m_curTStart = high_resolution_clock::now();
	}
}

void StopWatch::stopStopWatch()
{
	if (m_measurements.size() < m_maxMeasurements && m_running) {
		auto tEnd = high_resolution_clock::now();
		TimePair t;
		t.tStart = m_curTStart;
		t.tEnd = tEnd;
		m_measurements.push_back(t);
	}
}

int StopWatch::evaluateAverage()
{
	int totalT = 0;
	for (TimePair& t : m_measurements) {
		totalT += duration_cast<microseconds>(t.tEnd - t.tStart).count();
	}
	if (m_measurements.size() > 0) {
		return totalT / m_measurements.size();
	}
	else {
		return 0;
	}
}

int StopWatch::lastMeasurement()
{
	if (m_measurements.size() > 0) {
		TimePair& t = m_measurements.back();
		return duration_cast<microseconds>(t.tEnd - t.tStart).count();
	}
	return -1;
}

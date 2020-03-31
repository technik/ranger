#pragma once

#include <cassert>

#include "units.h"

struct Motor
{
	Motor() = default;
	Motor(FloatSeconds burnDuration, double thrust)
		: m_burnDuration(burnDuration)
		, m_thrust(thrust)
	{}

	bool readyToIgnite() const { return m_phase == Phase::ready; }

	void update(FloatSeconds dt)
	{
		if (m_phase == Phase::burning)
		{
			if (m_burningTime >= m_burnDuration)
				m_phase = Phase::exhausted;
			m_burningTime += dt;
		}
	}

	void ignite()
	{
		assert(m_phase == Phase::ready);
		m_phase = Phase::burning;
	}

	double currentThrust() const
	{
		return m_phase == Phase::burning ? m_thrust : 0.0;
	}

private:
	enum class Phase
	{
		ready,
		burning,
		exhausted
	} m_phase = Phase::ready;

	const FloatSeconds m_burnDuration;
	const double m_thrust;
	FloatSeconds m_burningTime{ 0 };
};
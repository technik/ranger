#include "motor.h"
#include "math.h"
#include <nlohmann/json.hpp>

ConstantMotor::ConstantMotor(FloatSeconds burnDuration, double thrust)
	: m_burnDuration(burnDuration)
	, m_thrust(thrust)
{}

void ConstantMotor::update(FloatSeconds dt)
{
	if (m_phase == Phase::burning)
	{
		if (m_burningTime >= m_burnDuration)
			m_phase = Phase::exhausted;
		m_burningTime += dt;
	}
}

double ConstantMotor::currentThrust() const
{
	return m_phase == Phase::burning ? m_thrust : 0.0;
}


ProfileSRB::ProfileSRB(const char* fileName)
{
	// Load Json file
}

void ProfileSRB::update(FloatSeconds dt)
{
	if (m_phase == Phase::burning)
	{
		if (m_burningTime >= m_thrustProfile.duration())
			m_phase = Phase::exhausted;
		m_burningTime += dt;
	}
}

double ProfileSRB::currentThrust() const
{
	if (m_phase == Phase::burning)
	{
		if (m_burningTime > m_thrustProfile.duration())
		{
			return 0;
		}
		return m_thrustProfile.getThrust(m_burningTime);
	}
	else return 0;
}

void ProfileSRB::ThrustProfile::parse(const nlohmann::json& profileData)
{
	// All thrust profiles assume an implicit 0 start point
	ThrustPoint start;
	start.thrust = 0.0;
	start.time = FloatSeconds(0);

	m_segments.reserve(profileData.size());
	for (auto& dataPoint : profileData)
	{
		ThrustSegment segment;
		segment.start = start;
		segment.end.time = FloatSeconds(dataPoint["t"].get<float>());
		segment.end.thrust = dataPoint["th"].get<float>();

		// Next segments will start where this one ends
		start = segment.end;
	}
}

double ProfileSRB::ThrustProfile::getThrust(FloatSeconds t) const
{
	for (auto& segment : m_segments)
	{
		if (segment.end.time < t)
			continue;

		auto dt = t - segment.start.time;
		auto factor = dt / segment.duration();
		return math::lerp(segment.start.thrust, segment.end.thrust, factor);
	}

	return 0.0; // Exhausted engine
}
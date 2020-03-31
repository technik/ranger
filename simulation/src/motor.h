#pragma once

#include <cassert>
#include <vector>

#include "units.h"
#include <nlohmann/json.hpp>

struct Motor
{
	bool readyToIgnite() const { return m_phase == Phase::ready; }

	virtual void update(FloatSeconds dt) = 0;

	void ignite()
	{
		assert(m_phase == Phase::ready);
		m_phase = Phase::burning;
	}

	virtual double currentThrust() const = 0;

protected:
	enum class Phase
	{
		ready,
		burning,
		exhausted
	} m_phase = Phase::ready;
};

struct ConstantMotor : Motor
{
	ConstantMotor() = default;
	ConstantMotor(FloatSeconds burnDuration, double thrust);

	void update(FloatSeconds dt) override;

	double currentThrust() const override;

private:
	const FloatSeconds m_burnDuration;
	const double m_thrust;
	FloatSeconds m_burningTime{ 0 };
};

struct ProfileSRB : Motor
{
	ProfileSRB() = default;
	ProfileSRB(const char* fileName);

	void update(FloatSeconds dt) override;

	double currentThrust() const override;

private:
	FloatSeconds m_burningTime{ 0 };

	struct ThrustProfile
	{
		struct ThrustPoint
		{
			FloatSeconds time;
			double thrust;
		};

		struct ThrustSegment
		{
			ThrustPoint start, end;
			FloatSeconds duration() const { return end.time - start.time; }
		};

		void parse(const nlohmann::json& profile);
		
		double getThrust(FloatSeconds t) const;

		FloatSeconds duration() const { return m_segments.back().end.time; }

		std::vector<ThrustSegment> m_segments;
	} m_thrustProfile;

};
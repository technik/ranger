#include <cassert>
#include <fstream>
#include <vector>

#include "motor.h"
#include "math.h"


struct Rocket
{
	Rocket(Motor m) : motor(m) {}

	void simulate(FloatSeconds dt)
	{
		if (motor.readyToIgnite())
			motor.ignite();

		motor.update(dt);

		// Update state
		auto thrust = motor.currentThrust();
		ddz = thrust + gravityAccel;
		z += dz * dt.count();
		dz += dt.count() * ddz;
		hasTouchedDown = (z < 0 && dz < 0);
		z = math::max(0.0, z);
	}

	static constexpr double gravityAccel = -9.81;

	Motor motor;
	double z = 0;
	double dz = 0;
	double ddz = 0;
	double mass = 1;
	bool hasTouchedDown = false;
};

struct DataProbe
{
	virtual void sample(const Rocket&) = 0;
	virtual void logRow(std::ostream&) = 0;
};

struct EngineProbe : DataProbe
{
	void sample(const Rocket& model) override
	{
		m_thrustRecord.push_back(model.motor.currentThrust());
	}

	void logRow(std::ostream& out) override
	{
		assert(m_cursor < m_thrustRecord.size());
		out << m_thrustRecord[m_cursor++];
	}
	
	size_t m_cursor = 0;
	std::vector<double> m_thrustRecord;
};

struct SolidBodyProbe : DataProbe
{
	void sample(const Rocket& model) override
	{
		m_zRecord.push_back(model.z);
		m_dzRecord.push_back(model.dz);
	}

	void logRow(std::ostream& out) override
	{
		assert(m_cursor < m_zRecord.size());
		out << m_zRecord[m_cursor] << ", " << m_dzRecord[m_cursor];
		m_cursor++;
	}

	size_t m_cursor = 0;
	std::vector<double> m_zRecord;
	std::vector<double> m_dzRecord;

};

struct Telemetry
{
	Telemetry()
	{
		m_channels.push_back(new EngineProbe());
		m_channels.push_back(new SolidBodyProbe());
	}

	void record(Rocket& state, float t, double accel)
	{
		for (auto& channel : m_channels)
			channel->sample(state);

		m_simTime.push_back(t);
		m_accel.push_back(accel);
	}

	void log(std::ostream& out)
	{
		out << "t, thrust, h, dh, ddh\n";
		for (size_t i = 0; i < m_simTime.size(); ++i)
		{
			out << m_simTime[i] << ", ";

			for (auto& channel : m_channels)
			{
				channel->logRow(out);
				out << ", ";
			}
			out << m_accel[i] << "\n";
		}
	}

private:
	std::vector<DataProbe*> m_channels;

	std::vector<float> m_simTime;
	std::vector<double> m_accel;
};

int main(int, const char**)
{
	const FloatSeconds burnTime(10);
	const double motorThrust = 20;
	Rocket model(Motor(burnTime, motorThrust));

	Telemetry telemetry;

	// Record initial state
	telemetry.record(model, 0, 0);

	// Run simulation
	auto duration = FloatSeconds(60.f);
	auto dt = FloatSeconds(0.1f);
	size_t nSteps = size_t(duration / dt);
	size_t step = 0;

	for (; step < nSteps; ++step)
	{
		auto t = step * dt.count();

		// Update state
		model.simulate(dt);

		// Record state
		telemetry.record(model, t, model.ddz);

		if (model.hasTouchedDown)
			break;
	}

	// Serialize results
	std::ofstream logFile("sim.csv");
	telemetry.log(logFile);

	return 0;
}
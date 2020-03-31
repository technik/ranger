#pragma once

namespace math
{
	template<class T>
	T min(T a, T b)
	{
		return a < b ? a : b;
	}

	template<class T>
	T max(T a, T b)
	{
		return a < b ? b : a;
	}

	template<class T1, class T2>
	auto lerp(T1 a, T1 b, T2 f)
	{
		return a * (1 - f) + b * f;
	}
}
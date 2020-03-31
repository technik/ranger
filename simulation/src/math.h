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
}
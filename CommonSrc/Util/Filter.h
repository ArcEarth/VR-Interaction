#pragma once

#include <cmath>

namespace Platform
{
	namespace Input
	{
		//////////////////////////////////////////////////////////////////////
		//
		// Filter 
		// 
		//////////////////////////////////////////////////////////////////////
		// Template class for filtering
		//  
		template <class T> class Filter
		{
		public:
			Filter(double* pUpdateFrequency);
			Filter() { };
			virtual T Apply(T NewValue) = 0;
			const T& Delta() const { return m_Delta; }
			const T& Value() const { return m_PrevValue; }
			virtual ~Filter(void);

			void Clear() { m_FirstTime = true; };
			void SetUpdateFrequency(double* updateFrequency) { m_pUpdateFrequency = updateFrequency; };


		protected:

			double* m_pUpdateFrequency;

			T m_PrevValue;
			T m_Delta;
			bool m_FirstTime;

		};

		//
		//////////////////////////////////////////////////////////////////////
		// //		mUpdateFrequency (Hz) 
		template <class T> Filter<T>::Filter(double* updateFrequency)
		{
			m_pUpdateFrequency = updateFrequency;
			m_FirstTime = true;
		}

		//
		//////////////////////////////////////////////////////////////////////
		// 
		template <class T> Filter<T>::~Filter(void)
		{
		}

		//////////////////////////////////////////////////////////////////////
		//
		// Low Pass Filter 
		// 
		//////////////////////////////////////////////////////////////////////
		// set the following before using: 
		//		mCutoffFrequency
		//

		template <class T>
		class LowPassFilter
			: public Filter<T>
		{
		public:
			LowPassFilter(double* pUpdateFrequency, double cutOffFrequency = 0) : Filter<T>(pUpdateFrequency), m_CutoffFrequency(cutOffFrequency){};
			LowPassFilter() : Filter<T>() { };
			virtual T Apply(T NewValue);
			void SetCutoffFrequency(double f) { m_CutoffFrequency = f; };
			double GetCutoffFrequency(void) { return m_CutoffFrequency; };

		protected:
			double m_CutoffFrequency;

		};

		//
		//////////////////////////////////////////////////////////////////////
		// 
		template <class T>
		T LowPassFilter<T>::Apply(T newValue)
		{
			/*

			Let's say Pnf the filtered position, Pn the non filtered position and Pn-1f the previous filtered position,
			Te the sampling period (in second) and tau a time constant calculated from the cut-off frequency fc.

			tau = 1 / (2 * pi * fc)
			Pnf = ( Pn + tau/Te * Pn-1f ) * 1/(1+ tau/Te)

			Attention: tau >= 10 * Te
			*/

			if (m_FirstTime)
			{
				m_PrevValue = newValue;
				m_FirstTime = false;
			}

			double updateFrequency = *m_pUpdateFrequency;

			T Te(1.0 / updateFrequency);		// the sampling period (in seconds)
			T Tau(1.0 / (2 * 3.14159265*m_CutoffFrequency));	// a time constant calculated from the cut-off frequency

			T filteredValue = (newValue + (Tau / Te) * m_PrevValue) * (T(1.0) / (T(1.0) + Tau / Te));

			m_Delta = filteredValue - m_PrevValue;
			m_PrevValue = filteredValue;

			return filteredValue;
			//}

		}


		//////////////////////////////////////////////////////////////////////
		//
		// Low Pass Dynamic (or Adjustable) Filter 
		// 
		//////////////////////////////////////////////////////////////////////
		// adjusts the cutoff filter depending on the velocity
		// set the following before using: 
		//		mUpdateFrequency (Hz) 
		//		mCutoffFrequency the lower cuttoff frequency
		//		mCutoffFrequencyHigh
		//		mVelocityLow (mm/s) the speed at which mCutoffFrequencyLow is reached 
		//		mVelocityHigh (mm/s) the speed at which mCutoffFrequencyHigh is reached
		template <class T, class _TNorm = conversion_to_double<T>>
		class LowPassDynamicFilter : public LowPassFilter<T>
		{
		public:
			LowPassDynamicFilter(double* updateFrequency) : LowPassFilter<T>(updateFrequency), mVelocityFilter(updateFrequency) {  };
			LowPassDynamicFilter() {  };
			virtual T Apply(T NewValue);
			void SetUpdateFrequency(double* updateFrequency) { m_pUpdateFrequency = updateFrequency; mVelocityFilter.SetUpdateFrequency(updateFrequency); };

			void SetCutoffFrequencyLow(double f) { m_CutoffFrequency = f; SetCutoffFrequencyVelocity(); };
			void SetCutoffFrequencyHigh(double f) { mCutoffFrequencyHigh = f; SetCutoffFrequencyVelocity(); };
			void SetVelocityLow(double f) { mVelocityLow = f; };
			void SetVelocityHigh(double f) { mVelocityHigh = f; };

		protected:

			// cutoff freq for velocity
			// equal to mCutoffFrequency  + 0.75 * (mCutoffFrequencyHigh - mCutoffFrequency)
			void SetCutoffFrequencyVelocity()
			{
				mVelocityFilter.SetCutoffFrequency((double) (m_CutoffFrequency + 0.75 * (mCutoffFrequencyHigh - m_CutoffFrequency)));
			}
			LowPassFilter<T> mVelocityFilter;
			T mLastPositionForVelocity;

			double mCutoffFrequencyHigh, mVelocityLow, mVelocityHigh;


		};

		//
		//////////////////////////////////////////////////////////////////////
		// 
		//template <class T>
		//T LowPassDynamicFilter<T>::Apply(T x)
		//{
		//	return x;
		//}

		template <class _T>
		struct conversion_to_double
			: public std::unary_function<_T, double>
		{
			double operator()(const _T& lhs)
			{
				return (double) lhs;
			}
		};

		template <class T, class _TNorm>
		T LowPassDynamicFilter<T, _TNorm>::Apply(T x)
		{

			// special case if first time being used
			if (m_FirstTime)
			{
				m_PrevValue = x;
				mLastPositionForVelocity = x;
				m_FirstTime = false;
			}


			double updateFrequency = *m_pUpdateFrequency;

			// first get an estimate of velocity (with filter)
			T mPositionForVelocity = mVelocityFilter.Apply(x);
			T vel = (mPositionForVelocity - mLastPositionForVelocity) * updateFrequency;
			mLastPositionForVelocity = mPositionForVelocity;


			// interpolate between frequencies depending on velocity
			double t = (_TNorm()(vel) - mVelocityLow) / (mVelocityHigh - mVelocityLow);

			t = max(t, 0.0);
			t = min(t, 1.0);

			T cutoff((mCutoffFrequencyHigh * t) + (m_CutoffFrequency * (1 - t)));

			T Te(1.0 / updateFrequency);		// the sampling period (in seconds)
			T Tau(T(1.0) / (2 * 3.14159265*cutoff));	// a time constant calculated from the cut-off frequency

			T filteredValue = (x + (Tau / Te) * m_PrevValue) * (T(1.0) / (T(1.0) + Tau / Te));

			m_Delta = filteredValue - m_PrevValue;
			m_PrevValue = filteredValue;
			return filteredValue;

		}

		//
		//////////////////////////////////////////////////////////////////////
		// 
	}
}
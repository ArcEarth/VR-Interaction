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
		template <class TValue,class TScaler = double>
		class Filter
		{
		public:
			//////////////////////////////////////////////////////////////////////
			// UpdateFrequency (Hz) 
			Filter(TScaler* pUpdateFrequency)
			{
				m_pUpdateFrequency = pUpdateFrequency;
				m_FirstTime = true;
			}

			Filter() {}

			virtual TValue Apply(TValue NewValue) = 0;

			const TValue& Delta() const { return m_Delta; }
			const TValue& Value() const { return m_PrevValue; }

			virtual ~Filter(void) { }

			void Clear() { m_FirstTime = true; };
			void SetUpdateFrequency(TScaler* updateFrequency) { m_pUpdateFrequency = updateFrequency; };


		protected:

			TScaler* m_pUpdateFrequency;

			TValue m_PrevValue;
			TValue m_Delta;
			bool m_FirstTime;

		};

		//////////////////////////////////////////////////////////////////////
		//
		// Low Pass Filter 
		// 
		//////////////////////////////////////////////////////////////////////
		// set the following before using: 
		//		mCutoffFrequency
		//

		template <class TValue,class TScaler = double>
		class LowPassFilter
			: public Filter<TValue, TScaler>
		{
		public:
			LowPassFilter(TScaler* pUpdateFrequency, TScaler cutOffFrequency = 0)
				: Filter<TValue, TScaler>(pUpdateFrequency), m_CutoffFrequency(cutOffFrequency){};

			LowPassFilter() : Filter<TValue>() { };

			virtual TValue Apply(TValue NewValue) override
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
					m_PrevValue = NewValue;
					m_FirstTime = false;
				}

				TScaler updateFrequency = *m_pUpdateFrequency;
				TScaler Te(TScaler(1.0) / updateFrequency);		// the sampling period (in seconds)
				TScaler Tau(TScaler(1.0) / (TScaler(2 * 3.14159265) * m_CutoffFrequency));	// a time constant calculated from the cut-off frequency

				auto t = TScaler(1) / (TScaler(1) + (Tau / Te));

				m_Delta = t * (NewValue - m_PrevValue);

				m_PrevValue += m_Delta;

				return m_PrevValue;
			}

			void SetCutoffFrequency(TScaler f) { m_CutoffFrequency = f; };

			TScaler GetCutoffFrequency(void) { return m_CutoffFrequency; };

		protected:
			TScaler m_CutoffFrequency;

		};

		template <class _TVector,class _TScaler>
		struct conversion_to_scalar
			: public std::unary_function<_TVector, _TScaler>
		{
			_TScaler operator()(const _TVector& lhs)
			{
				return (_TScaler) lhs;
			}
		};

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
		template <class TValue, class TScaler = double, class _TNorm = conversion_to_scalar<TValue, TScaler>>
		class LowPassDynamicFilter : public LowPassFilter<TValue, TScaler>
		{
		protected:
			LowPassFilter<TValue> m_VelocityFilter;
			TValue m_LastPositionForVelocity;
			TScaler m_CutoffFrequencyHigh, m_VelocityLow, m_VelocityHigh;

		public:
			LowPassDynamicFilter(TScaler* updateFrequency) : LowPassFilter<TValue, TScaler>(updateFrequency), m_VelocityFilter(updateFrequency) {  };
			LowPassDynamicFilter() {  };
			virtual TValue Apply(TValue NewValue)
			{

				// special case if first time being used
				if (m_FirstTime)
				{
					m_PrevValue = NewValue;
					m_LastPositionForVelocity = NewValue;
					m_FirstTime = false;
				}


				TScaler updateFrequency = *m_pUpdateFrequency;

				// first get an estimate of velocity (with filter)
				TValue mPositionForVelocity = m_VelocityFilter.Apply(NewValue);
				TValue vel = (mPositionForVelocity - m_LastPositionForVelocity) * updateFrequency;
				m_LastPositionForVelocity = mPositionForVelocity;


				// interpolate between frequencies depending on velocity
				TScaler t = (_TNorm()(vel) - m_VelocityLow) / (m_VelocityHigh - m_VelocityLow);
				t = min(max(t, 0.0), 1.0);
				TScaler cutoff((m_CutoffFrequencyHigh * t) + (m_CutoffFrequency * (1 - t)));
				TScaler Te(1.0 / updateFrequency);		// the sampling period (in seconds)
				TScaler Tau(TScaler(1.0) / (TScaler(2 * 3.14159265) * cutoff));	// a time constant calculated from the cut-off frequency

				t = TScaler(1) / (TScaler(1) + (Tau / Te));

				m_Delta = t * (NewValue - m_PrevValue);

				m_PrevValue += m_Delta;

				return m_PrevValue;


			}
			void SetUpdateFrequency(TScaler* updateFrequency) { m_pUpdateFrequency = updateFrequency; m_VelocityFilter.SetUpdateFrequency(updateFrequency); };

			void SetCutoffFrequencyLow(TScaler f) { m_CutoffFrequency = f; SetCutoffFrequencyVelocity(); };
			void SetCutoffFrequencyHigh(TScaler f) { m_CutoffFrequencyHigh = f; SetCutoffFrequencyVelocity(); };
			void SetVelocityLow(TScaler f) { m_VelocityLow = f; };
			void SetVelocityHigh(TScaler f) { m_VelocityHigh = f; };

		protected:

			// cutoff freq for velocity
			// equal to mCutoffFrequency  + 0.75 * (mCutoffFrequencyHigh - mCutoffFrequency)
			void SetCutoffFrequencyVelocity()
			{
				m_VelocityFilter.SetCutoffFrequency((TScaler) (m_CutoffFrequency + 0.75 * (m_CutoffFrequencyHigh - m_CutoffFrequency)));
			}

		};

		//
		//////////////////////////////////////////////////////////////////////
		// 
	}
}
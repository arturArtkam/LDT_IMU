#ifndef _HMC_HPP_
#define _HMC_HPP_

#include <avr/io.h>
#include <stdlib.h>
#include <tools.h>

#define MAX_CNT 28

#if !defined(HMC)
# error "Global define HMC=port,pin,hx,-hy,hz"
#endif

template <uint8_t SRport, uint8_t SRbit, int8_t chx, int8_t chy, int8_t chz>
class hmc_t
{
	#define SRHMC (*(PORT_t*) (0x0400 + 0x20 * SRport))
	public:
	uint8_t  srCount, cntPlus, cntMinus; 
	int64_t sumPlus[3], sumMinus[3];
	int64_t sum[3];
	uint16_t cnt;
	
	INLN hmc_t(void)
	{
		SRHMC.DIRSET = 1 << SRbit;
		Set();
	}
	void SummData(int32_t*data)
	{
		if (srCount > 1)
		{
			if (IsSet()) 
			{
				sumPlus[0] += data[abs(chx)];
				sumPlus[1] += data[abs(chy)];
				sumPlus[2] += data[abs(chz)];
				cntPlus++;
			}
			else
			{
				sumMinus[0] -= data[abs(chx)];
				sumMinus[1] -= data[abs(chy)];
				sumMinus[2] -= data[abs(chz)];
				cntMinus++;
			}
		}
		else if ((srCount == 0)&&(cntMinus>0)&&(cntPlus>0))
		{
			if(cntMinus==cntPlus)
			{
				sum[0] += sumPlus[0] + sumMinus[0];
				sum[1] += sumPlus[1] + sumMinus[1];
				sum[2] += sumPlus[2] + sumMinus[2];
				cnt += (cntMinus)*2;				
			}
			sumPlus[0] = 0;
			sumPlus[1] = 0;
			sumPlus[2] = 0;
			sumMinus[0] = 0;
			sumMinus[1] = 0;
			sumMinus[2] = 0;
			cntPlus = cntMinus =0;
		}
	}
	INLN void ResetSumm(float* xyz)
	{
		if (cnt == 0) return;	
		if (chx > 0) xyz[0] = sum[0]/cnt;
		else xyz[0] = -sum[0]/cnt;
		sum[0] = 0;
		if (chy > 0) xyz[1] = sum[1]/cnt;
		else xyz[1] = -sum[1]/cnt;
		sum[1] = 0;
		if (chz > 0) xyz[2] = sum[2]/cnt;
		else xyz[2] = -sum[2]/cnt;
		sum[2] = 0;
		cnt = 0;
	}
	INLN void ToggleSetReset(void)
	{
		if (srCount++ > MAX_CNT)
		{
			Toggle();
			srCount = 0;
		}
	}
	INLN void Toggle(void)
	{
		SRHMC.OUTTGL = 1 << SRbit;
	}
	INLN void Set(void)
	{
		SRHMC.OUTSET = (1 << SRbit);
	}
	INLN void ResetSet(void)
	{
		SRHMC.OUTCLR = (1 << SRbit);
	}
	INLN bool IsSet(void)
	{
		return	SRHMC.OUT & (1 << SRbit);
	}
};

extern hmc_t<HMC> hmc;

#endif
#ifndef _ADS131_UDR_HPP_
#define _ADS131_UDR_HPP_

#include <tools.h>
#include "HMC.hpp"

class ADS131_udr
{
	public:
	INLN void UserDataReady(void)
	{
		hmc.ToggleSetReset();		
	}	
};
extern ADS131_udr ads131_udr;
#endif
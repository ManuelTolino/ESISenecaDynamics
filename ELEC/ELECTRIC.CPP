//File:electric.cpp
//Prefix: EL_

//Module: Seneca Dynamics

/*  functions:
    EL_fault : set a fault
    EL_nofault : remove a fault
	GeneratorAvailable : find maximum current available
	UpdateGenerator : add a load to a generator
	UpdateBattery: charge / discharge the battery
	EL_power: check whether sufficient power is available
	EL_load: add a load
	EL_noload: remove a load
	EL_setammeter: set the ammeter source
	EL_getammeter: return the ammeter source
	EL_getcurrent: return the ammeter reading
	EL_getcharge: return the battery charge reading
	EL_indicator: return a alternator status indicator
	EL_start: initialise this module
	UpdateLoadBalancer: split current request between generators
	EL_update: per-frame update
*/

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/Master/seneca/ELEC/RCS/electric.cpp 1.5 2000/06/16 10:15:58 ColinJ Exp $
//$Log: electric.cpp $
//Revision 1.5  2000/06/16 10:15:58  ColinJ
//Changed overload ouput calculations.
//
//Revision 1.4  2000/05/31 15:53:49  colinj
//More changes for L & R Alt overload.
//
//Revision 1.3  2000/05/30 13:12:50  colinj
//Changed handling of L & R Alt overload - now done in
//EL_getcurrent.
//gen_fail_load[] and FullCurrent[] no longer set up.
//left_pop and right_pop no longer set.
//Allow for ALT_L/R_ZERO faults.
//
//Revision 1.2  2000/05/14 10:52:18  colinj
//Changed load values.
//
//Revision 1.1  2000/04/17 09:22:43  colinj
//Initial revision
//

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include "electric.h"
//#include "timing.h"

//----------------
// Defines
//----------------

#define TRUE 1
#define FALSE 0

//----------------
// Externals
//----------------

extern float delta_time;

//-----------------
// Local Variables
//-----------------
/* At idle no current is produced, at full RPM the maximum current is produced */
static const double IdleRPM =  950.0;
static const double FullRPM = 2000.0;

/* engine RPM */ 
static double LeftRPM = 0; 
static double RightRPM = 0;

static const double FullCharge = 35.0 * 3600; /* 35 Ah - stored in As */
static double FullCurrent[2] = {70.0,70.0}; /* per alternator, A */

static double Charge; /* As */
static double Current = 0.0; /* Total current into battery */
static double GeneratorCurrent[2] = {0.0,0.0}; /* Current out of generators */
static char GeneratorSwitch[2] = {OFF, OFF};
static double MaxCurrent[2]; /* Maximum current available from generators */

static long Load = 0; /* Bit map of current loads */
static double TotalLoad = 0.0; /* Sum of loads, As */
static int Indicator[2] = { OFF, OFF };

static const double StarterAddLoad = 25.0; /* Maximum additional load , A, per starter dependent on RPM */

static const double BatteryCharge = 30.0; /* Maximum rate of charging. (XXX guess - not in IM).*/
static const double MaxBatteryDischarge = 100.0; /* Maximum rate of discharging. (XXX guess - not in IM). */
static double BatteryDischarge = 0.0; /* Battery switch - current max discharge rate = " is on, 0 if off */ 
static int DischargeRate = 0; /* Discharge rate flag: 0 = OK, 1 = discharging too fast */

static double PrevAmmeter = 0.0;
static int Ammeter = BATTERY; /* battery, L alternator or R alternator? */
static const double MaxAmmeter = 100.0;
float gen_fail_load[2] = {0.0,0.0};
long ElectricFaults = 0;
static EL_out out;

static const double LoadTable[] =
{
	6.0, /* ambient: '5-7' */
	20.0, /* prop de-ice */
	10.0, /* pitot heat */
	10.0, /* gear pump */
	10.0, /* flap motor */
	5.0, /* vent fan */
	5.0, /* heater */
	5.0, /* defrost fan */
	5.0, /* radios on TX */
	9.0, /* landing lights */
	3.0, /* recognition */
	2.5, /* navigation */
	3.5, /* strobes */
   25.0, /* left starter : minimum current (more is added depending on RPM) */
   25.0, /* right starter */
   9.0, /* landing lights */
};



double BatteryAvailable(void)
{
   if (Charge == 0)
      return 0;

   return BatteryDischarge;
}

/* Is there sufficient power?
   Current drawn must not exceed total capacity of battery and alternators switched on and working/charged. */


void EL_fault(int f)
{
	ElectricFaults |= f;
}

void EL_nofault(int f)
{
	ElectricFaults &= ~f;
}

/* return current available from a generator */
static double GeneratorAvailable(double rpm, int generator)
{
	double current;



if ((rpm < IdleRPM) || (!(GeneratorSwitch[generator])))
	{
		current = 0; /* Get generator power */
      Indicator[generator] = ON;

	}
	else
	{
		if (rpm > FullRPM)
		{
			current = (FullCurrent[generator] + gen_fail_load[generator]);
		}
		else 
		{
			current = ((rpm - IdleRPM) / (FullRPM - IdleRPM)) * (FullCurrent[generator] +  gen_fail_load[generator]); 
		}
	

	   if ((generator == LEFT) && ((ElectricFaults & BREAKER_ALT_L) || (ElectricFaults & ALT_L_ZERO)) 
         || ((generator == RIGHT) && ((ElectricFaults & BREAKER_ALT_R) || (ElectricFaults & ALT_R_ZERO))))
      {
		/* Alternator failure, overvoltage cut off or breaker */
		   current = 0;
		   Indicator[generator] = ON;


		//if (!(ElectricFaults & BREAKER_ALT_L + BREAKER_ALT_R)) 
      //if(GeneratorCurrent[generator] < 3.0) ;
      }
	   if((generator == LEFT) && (ElectricFaults & ALT_L_OVERLOAD) && !(ElectricFaults & BREAKER_ALT_L))
      {
		   current = 60;
      }

	   if((generator == RIGHT) && (ElectricFaults & ALT_R_OVERLOAD) && !(ElectricFaults & BREAKER_ALT_R) )
      {
		   current = 60;
      }

   }
   

   //if(GeneratorCurrent[generator] < 3.0) Indicator[generator] = ON;
	return current;
}

static void UpdateGenerator(int generator, double load)
{

	
GeneratorCurrent[generator] += load;

}

/* Discharge the battery through the given load. */
/* (-ve = discharge, +ve = charge.) */
static void UpdateBattery(double load, double dt) {

   if(ElectricFaults & BATTERY_FLAT)
   {
      Charge = 0;
      return;
   }

	Charge += (load * dt);
	if (Charge < 0)
	{
		Charge = 0;
	}
	if (Charge > FullCharge) 
	{
		Charge = FullCharge;
	}

}

/* Check if the battery is charged and not overloaded - if so there is sufficient power. */

int EL_power(void) {
   
	return ((GeneratorAvailable(LeftRPM, LEFT) + GeneratorAvailable(RightRPM, RIGHT) + BatteryAvailable()) > Current);
}
/*
int EL_power(void) {
   
	return 1; //(Charge > 0) && (!DischargeRate) && (Current > 0);
}
*/
/* base 2 log of an integer */
int log2(int i)
{
   return (i==1) ? 0 : log2(i>>1)+1;
}

/* add load */
void EL_load(int load)
{
	if ((load >= 1<<LOADS) || (load < 0))
	{
		
		exit(2);
	}
	if (!(Load & load))
	{
 		TotalLoad += LoadTable[log2(load)];
		Load |= load;
	}
}

/* remove load */
void EL_noload(int load)
{
	if ((load >= 1<<LOADS) || (load < 0))
	{

		exit(2);
	}
	if (Load & load)
	{
      /*fprintf(stderr,"removing load %d (%d)\n", load, log2(load));
      Sleep(5000);*/
		TotalLoad -= LoadTable[log2(load)];
		Load &= ~load;
	}
}


void EL_setammeter(int source)
{
	if ((source < 0) || (source > SOURCES))
	{
		
		exit(2);
	}
	Ammeter = source;
}

int EL_getammeter(void)
{
	return Ammeter;
}

/* return ammeter reading */
double EL_getcurrent(void)
{
	//if (ElectricFaults & AMMETER_ZERO) return 0.0;
	//if (ElectricFaults & AMMETER_MAX) return MaxAmmeter;
	//if (ElectricFaults & AMMETER_STUCK) return PrevAmmeter;

	switch(Ammeter)
	{
	case BATTERY:
		{
			PrevAmmeter = Current;
			break;
		}
	case GENERATOR_L:
		{
            PrevAmmeter = GeneratorCurrent[LEFT];
			break;
		}
	case GENERATOR_R:
		{
            PrevAmmeter = GeneratorCurrent[RIGHT];
			break;
		}
	default:
		{
			
			exit(2);
		}
	}

	return PrevAmmeter;
}

/* Charge in Ah */
double EL_getcharge(void)
{
	return Charge/3600;
}

int EL_getindicator(int i)
{
	return Indicator[i];
}

void EL_start(void) {
	Charge = FullCharge;
}

/* Given current draw, calls generators to get current. Returns >=0 current still needed. */
static double UpdateLoadBalancer(double current)
{
double lcurrent, rcurrent;
double maxcurrent;
double unavailable;

maxcurrent = MaxCurrent[LEFT] + MaxCurrent[RIGHT];

if (maxcurrent == 0)
{
	lcurrent = rcurrent = 0;
	unavailable = current;
}
else
{

	unavailable = current - maxcurrent;

	if (current > maxcurrent)
	{
		current = maxcurrent;
	}

	if((ElectricFaults & ALT_R_OVERLOAD) && (ElectricFaults & ALT_L_OVERLOAD))
	{
		lcurrent = MaxCurrent[LEFT];
		rcurrent = MaxCurrent[RIGHT];
	}
   else
	if(ElectricFaults & ALT_L_OVERLOAD)
	{
		lcurrent = MaxCurrent[LEFT];
	   current -= MaxCurrent[LEFT];
      rcurrent = current;
      if(rcurrent < 0.0) rcurrent = 0.0;
	}
	else
	if(ElectricFaults & ALT_R_OVERLOAD)
	{
		rcurrent = MaxCurrent[RIGHT];
   	current -= MaxCurrent[RIGHT];
	   lcurrent = current;
      if(lcurrent < 0.0) lcurrent = 0.0;
	}
   else
	{
	   lcurrent = (MaxCurrent[LEFT]/maxcurrent)*current;
	   rcurrent = (MaxCurrent[RIGHT]/maxcurrent)*current;
	}
}

UpdateGenerator(LEFT, lcurrent);
UpdateGenerator(RIGHT, rcurrent);

if (unavailable < 0)
{
	unavailable = 0;
}

return unavailable;

}


/* return load, including 'special' loads (currently only the starter) */
static double GetLoad(void)
{
   double load = TotalLoad; /* total std loads */


/* Starter load = 25A at idle or above, linearly rising to 50A with engine stopped. */

   if ((Load & LSTARTER) && (LeftRPM < IdleRPM))
   {
      load += (StarterAddLoad * (IdleRPM - LeftRPM) / IdleRPM);
   }
   
   if ((Load & RSTARTER) && (RightRPM < IdleRPM))
   {
      load += (StarterAddLoad * (IdleRPM - RightRPM) / IdleRPM);
   }

   return load;
}


/* Battery charging current - normally BatteryCharge but less if battery is nearly full */
static double ChargeRate(void)
{
   if (Charge < (FullCharge * 0.9))
   {
      return BatteryCharge;
   }

   


   return (1.0 - ((Charge - (FullCharge * 0.9)) / (FullCharge * 0.1))) * BatteryCharge;
}



/* Per frame update of the electrical system */
void EL_update(double leftrpm, double rightrpm)
{
	int i;
	float dt;
	static double oldcharge = 0;
	double current;
	double uncharged;

   LeftRPM = leftrpm;
   RightRPM= rightrpm;
	
//	dt = TI_dt(); /* seconds */

   dt = delta_time;

	for(i=0;i<2;i++)
	{
		Indicator[i] = OFF;
		GeneratorCurrent[i] = 0;
	}

		MaxCurrent[LEFT] = GeneratorAvailable(leftrpm, LEFT);
      
		MaxCurrent[RIGHT] = GeneratorAvailable(rightrpm, RIGHT);

	current = UpdateLoadBalancer(GetLoad()); /* Get load current from generators */

   
	
	if (current > 0)
	{
		/* Battery is discharging */
		if (current > BatteryDischarge)
		{
			DischargeRate = 1;
			current = BatteryDischarge;
		}
		else
		{
			DischargeRate = 0;
		}
		UpdateBattery(-current, dt);
	}
	else /* current = 0 */
	{
         DischargeRate = 0;
		/* Battery is charging */
		if (Charge < FullCharge)
      {
         if (BatteryDischarge > 0) /* battery is connected */
            {
               uncharged = UpdateLoadBalancer(ChargeRate());
               current = ChargeRate() - uncharged;
               UpdateBattery(current, dt);
            }
		}
	}
	Current = (Charge - oldcharge)/dt;
	oldcharge = Charge;

	return;
}


/* Update everything */
EL_out EL_frame(EL_in *in)
{
   int i;
   static first = 0;
   

 /* initialise once */

   if (first == 0)
   {
      first = 1;
      EL_start();
   }

   
   /* Set things that change every frame (mainly engine) */

   /* switches */

   GeneratorSwitch[LEFT] = (char)(in->switches & EL_SW_LGEN);
   
   GeneratorSwitch[RIGHT] = (char) (in->switches & EL_SW_RGEN);
   if (in->switches & EL_SW_BATT)
   {
      BatteryDischarge = MaxBatteryDischarge;
   }
   else
   {
      BatteryDischarge = 0;
   }

   EL_update(in->leftrpm, in->rightrpm); 

   /* Set things that don't change each frame */

   /* loads */
   for(i=0;i<LOADS;i++)
   {
      if (in->loads & 1<<i)
      {
         EL_load(1<<i);
      }
      else
      {
         EL_noload(1<<i);
      }
   }

   /* ammeter source */
   EL_setammeter(in->ammeter);

   /* faults */

   for(i=0;i<FAULTS;i++)
   {
      if (in->faults & 1<<i)
      {
         EL_fault(1<<i);
      }
      else
      {
         EL_nofault(1<<i);
      }
   }
   

   /* Return values */

   out.current = EL_getcurrent();
   out.charge = EL_getcharge();
   out.lind = EL_getindicator(LEFT);
   out.rind = EL_getindicator(RIGHT);
   out.power = EL_power();
//if(GeneratorCurrent[LEFT] <= 0.0) out.lind = ON;   
//if(GeneratorCurrent[RIGHT] <= 0.0) out.rind = ON;   

   return out;
}
//----------------------------------------------------------------------

float calc_cofg(void)
{
//position in inches of weight areas in respect to center of gravity
#define AIRCRAFT_DATUM  86.7f
#define PILOTS_DATUM   118.1f
#define PAX1_DATUM     119.1f
#define PAX2_DATUM     157.6f
#define FUEL_DATUM      93.6f
#define BAGAGE_DATUM   178.7f
#define AIRCRAFT_WEIGHT


   float aircraft_moment,total_moment;
   float fuel_moment,bagage_moment = 0.0;
   float pilots_moment,pax1_moment,pax2_moment;
   float total_weight,cofg;


// calculate individual moments.
   aircraft_moment = (float)(AIRCRAFT_DATUM * DryWeight);
   pilots_moment   = (float)(PILOTS_DATUM * PilotWeight);
   pax1_moment     = (float)(PAX1_DATUM * Pax1Weight);
   pax2_moment     = (float)(PAX2_DATUM * Pax2Weight);
   fuel_moment     = (float)(FUEL_DATUM * ((outputs.user[Y_A_L_FUEL] * 6.0) + (outputs.user[Y_A_R_FUEL] * 6.0)));
   bagage_moment   = (float)(BAGAGE_DATUM * BaggageWeight);
//calculate total weight and total moment
   if(DryWeight < 0.0) DryWeight = 3222.0;
   total_weight = (float)(DryWeight + Pax1Weight + Pax2Weight + PilotWeight + BaggageWeight +((outputs.user[Y_A_L_FUEL] * 6.0) + (outputs.user[Y_A_R_FUEL] * 6.0)));
   total_moment = aircraft_moment + pilots_moment + pax1_moment + pax2_moment + fuel_moment + bagage_moment;
//center of gravity is total moments over total weight;

   cofg = (float)(total_moment/total_weight);

   return cofg;
}

//----------------------------------------------------------------------

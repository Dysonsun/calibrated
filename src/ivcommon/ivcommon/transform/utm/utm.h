
#ifndef UTM_H
#define UTM_H
namespace transform {
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
enum GridZone
  {
    UTM_ZONE_AUTO = 0,
    UTM_ZONE_1=1,  UTM_ZONE_2=2,    UTM_ZONE_3=3,    UTM_ZONE_4=4,    UTM_ZONE_5=5,
    UTM_ZONE_6=6,    UTM_ZONE_7=7,    UTM_ZONE_8=8,    UTM_ZONE_9=9,    UTM_ZONE_10=10,
    UTM_ZONE_11=11,   UTM_ZONE_12=12,   UTM_ZONE_13=13,   UTM_ZONE_14=14,   UTM_ZONE_15=15,
    UTM_ZONE_16=16,   UTM_ZONE_17=17,   UTM_ZONE_18=18,   UTM_ZONE_19=19,   UTM_ZONE_20=20,
    UTM_ZONE_21=21,   UTM_ZONE_22=22,   UTM_ZONE_23=23,   UTM_ZONE_24=24,   UTM_ZONE_25=25,
    UTM_ZONE_26=26,   UTM_ZONE_27=27,   UTM_ZONE_28=28,   UTM_ZONE_29=29,   UTM_ZONE_30=30,
    UTM_ZONE_31=31,   UTM_ZONE_32=32,   UTM_ZONE_33=33,   UTM_ZONE_34=34,   UTM_ZONE_35=35,
    UTM_ZONE_36=36,   UTM_ZONE_37=37,   UTM_ZONE_38=38,   UTM_ZONE_39=39,   UTM_ZONE_40=40,
    UTM_ZONE_41=41,   UTM_ZONE_42=42,   UTM_ZONE_43=43,   UTM_ZONE_44=44,   UTM_ZONE_45=45,
    UTM_ZONE_46=46,   UTM_ZONE_47=47,   UTM_ZONE_48=48,   UTM_ZONE_49=49,   UTM_ZONE_50=50,
    UTM_ZONE_51=51,   UTM_ZONE_52=52,   UTM_ZONE_53=53,   UTM_ZONE_54=54,   UTM_ZONE_55=55,
    UTM_ZONE_56=56,   UTM_ZONE_57=57,   UTM_ZONE_58=58,   UTM_ZONE_59=59,   UTM_ZONE_60=60,
    UPS_NORTH, UPS_SOUTH, 
    GRID_AUTO
  };

enum Hemisphere
  {
    HEMI_AUTO = 0,  HEMI_NORTH, HEMI_SOUTH
  };
  struct point{
    double x;
    double y;
  };
struct state_struct{
 point position;
};

state_struct Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0);
#if !defined(__cplusplus)
typedef enum GridZone GridZone;
typedef enum Hemisphere Hemisphere;
#endif

/* FORWARD AND BACK TM/PS PROJECTIONS FOR A SPHERE */

void geographic_to_tm_sphere(double R, double k0, 
			     double lon_mer, double FN, double FE,
			     double lat_rad, double lon_rad,
			     double* N, double* E);

void tm_to_geographic_sphere(double R, double k0, 
			     double lon_mer, double FN, double FE,
			     double N, double E,
			     double* lat_rad, double* lon_rad);

void geographic_to_ps_sphere(double R, double k0, 
			     Hemisphere hemi, double FN, double FE,
			     double lat_rad, double lon_rad,
			     double* N, double* E);

void ps_to_geographic_sphere(double R, double k0, 
			     Hemisphere hemi, double FN, double FE,
			     double N, double E,
			     double* lat_rad, double* lon_rad);

/* FORWARD AND BACK TM/PS PROJECTIONS FOR AN ELLIPSOID */

#ifndef TM_TO_GEOGRAPHIC_TOLERANCE_M
#define TM_TO_GEOGRAPHIC_TOLERANCE_M 0.001
#endif

void geographic_to_tm(double a, double e2, double k0, 
		      double lon_mer, double FN, double FE,
		      double lat_rad, double lon_rad,
		      double* N, double* E);

void tm_to_geographic(double a, double e2, double k0, 
		      double lon_mer, double FN, double FE,
		      double N, double E,
		      double* lat_rad, double* lon_rad);

void geographic_to_ps(double a, double e2, double k0, 
		      Hemisphere hemi, double FN, double FE,
		      double lat_rad, double lon_rad,
		      double* N, double* E);

void ps_to_geographic(double a, double e2, double k0, 
		      Hemisphere hemi, double FN, double FE,
		      double N, double E,
		      double* lat_rad, double* lon_rad);

/* FORWARD AND BACK PROJECTIONS FOR AN ELLIPSOID ONTO THE UTM/UPS GRID */

int geographic_to_grid(double a, double e2,
		       double lat_rad, double lon_rad, 
		       GridZone* zone, Hemisphere* hemi, double* N, double* E);

int grid_to_geographic(double a, double e2,		       
		       GridZone zone, Hemisphere hemi, double N, double E,
		       double* lat_rad, double* lon_rad);
} //namespace transform
#endif /* UTM_H */

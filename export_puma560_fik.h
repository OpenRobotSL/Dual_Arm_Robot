#ifndef EXPORT_PUMA560_FIK_H
#define EXPORT_PUMA560_FIK_H

#define DLLTESTSHARED_EXPORT __declspec(dllexport)
#ifdef DLLTESTSHARED_EXPORT
#else
#define DLLTESTSHARED_EXPORT __declspec(dllimport)
#endif

//DLLTESTSHARED_EXPORT float multiplyfun_export(float a,float b);

DLLTESTSHARED_EXPORT int  fk_hrg_puma560_DefTcs(const double *dh,const double *pfAxesPosRad,double *xyz_rpy);

DLLTESTSHARED_EXPORT int ik_hrg_puma560(const double *dh,const double *xyz_rpy,const double *axis_pos_old_rad,double *ikResultRad);


#endif // EXPROT_PUMA560_FIK_H
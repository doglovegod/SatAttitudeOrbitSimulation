/*
Attitude dynamics simulation for satellite which is suitable for which equiped with 4 momentum wheels and 3 magtorques
the attitude module is coded as a DLL
this file just demo. how to create and use the attitude dynamics module.
for more details, please contact:708979286@qq.com
*/
#include <iostream>
#include <fstream>
#include <iomanip> 

#include <windows.h>

#include "OrbitKit.h"
#include "DateTime.h"

using namespace std;

typedef bool(*lpInitAttDynSystem)();
typedef void(*lpAttDyn)(double*,double,double*,double*,int*);

int main(int argc, char* argv[])
{

	CDateTime m_dateTime;
	COrbitKit m_orbitKit;

	SATELLITE m_sat;
	m_sat.modeltype = NEWCREATEDSAT;
	m_sat.oe.jd_epoch = m_dateTime.GetStartTime(true);
	m_sat.oe.semi_ma = 6845.7144407503;
	m_sat.oe.ecc = 0.0008936;
	m_sat.oe.inc = 97.2473;
	m_sat.oe.omega = 165.2955;
	m_sat.oe.arg_perigee = 118.3424;
	m_sat.oe.mean_M0 = 329.8468;
	
	double jd = m_dateTime.GetStartTime(true);
	double start = jd;
	double end = jd + 10./86400;//only simulate 10s duration
	double step = 0.25/86400;//0.25s
	double t=0;

	ofstream out("results.txt");
	out<<"wx wy wz q0 q1 q2 q3"<<endl;

	HINSTANCE hDll;

	lpInitAttDynSystem InitAttDynSystem;
	lpAttDyn AttDyn;

	hDll = LoadLibrary("AttitudeDynamics.dll");

	if (hDll != NULL)
	{

		//step1£ºinitialize the attitude system£¬note that "attsimparams.xml" file should be editted firstly
		InitAttDynSystem = (lpInitAttDynSystem)GetProcAddress(hDll, "InitAttDynSystem");

		if (InitAttDynSystem != NULL)
		{
			bool result = InitAttDynSystem();

//			cout << result << endl;
		}

		AttDyn = (lpAttDyn)GetProcAddress(hDll, "AttDyn");
		if (AttDyn != NULL)
		{

			for (jd = start;jd < end;jd += step)
			{

				//step 2£ºcommunicate with the Onboard computer to get output of the actuators
				double attdrv[10];
				int flag[]={0,1};


				//step3£ºcalculate the orbit
				m_orbitKit.SatellitePosition(jd, &m_sat);

				double orbit[14]={m_sat.sp.ma ,m_sat.sp.ecc ,m_sat.sp.inc,m_sat.sp.raan ,m_sat.sp.w ,m_sat.sp.u ,m_sat.sp.x,m_sat.sp.y,m_sat.sp.z,m_sat.sp.vx,m_sat.sp.vy,m_sat.sp.vz,m_sat.sp.geolat,m_sat.sp.geolon};


				//step 4£ºcall the attitude functions to calculate the current attitude
				double attout[32]={0};
				
				t=(jd-start)*86400.;

				AttDyn(attout,t,orbit,attdrv,flag);//Call the DLL

				//Step 5£ºprocess the results and send to the Onboard computer

				//here for demo.,we just save them to file

				for(int i=0;i<7;i++)
				{
					cout<<attout[i]<<" ";//attitude rotate speed and quaternion
					out << setprecision(12)<<attout[i]<<" ";
				}
				cout<<endl;
				out<<endl;

			}
		}


		FreeLibrary(hDll);
	}

	system("pause");
	
	out.close();

	return 0;
}
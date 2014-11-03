#include "cstdio"
#include "cstdlib"
#include "cmath"

#define	TYPE_BIG	0
#define	TYPE_LITTLE	1

#define TIMESLICE	100

#define	DEVELOPING

struct VIRT_CORE{
	int domain;
	int num;
	int code;
	double requ;	
};

struct PHYS_CORE{
	int num;
	int type;
	int load;
	double freq;	
	double efficient;
	int* workload;
};


struct DecreSet{
//	int LB;
	int delta;
	bool* MachTight;
	bool* JobTight;
	bool* MachPicked;
	bool* JobPicked;
};
#include "cstdio"
#include "cstdlib"

#define	TYPE_BIG	0
#define	TYPE_LITTLE	1

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
	double freq;
	double load;
	double efficient;
	double* workload;
};

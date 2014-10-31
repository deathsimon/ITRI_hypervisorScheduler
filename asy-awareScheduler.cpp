#include "asy-awareScheduler.h"

VIRT_CORE** vcoreContainer;
PHYS_CORE** pcoreContainer;

static int numVcore;
static int numPcore;


/*
phase 1()
	Generate the percentage of time each virtual core should run on each physical core.	
*/
double phase1(){

	// sort vcpu by requirement in ascescending order
	for(int i=0;i<numVcore-1;i++){
		for(int j=i+1;j<numVcore;j++){
			if(vcoreContainer[i]->requ > vcoreContainer[j]->requ){
				VIRT_CORE* temp;
				temp = vcoreContainer[i];
				vcoreContainer[i] = vcoreContainer[j];
				vcoreContainer[j] = temp;
			}
		}
	}

	// compute scale factor
	// scale = 1: the current physical core setting can process all the virtual core requirements
	// scale < 1: scale down the virtual core requirements
	double req_sum = 0.0;
	double freq_sum = 0.0;
	double scale;

	for(int i=0;i<numVcore;i++){
		req_sum += vcoreContainer[i]->requ;
	}
	for(int i=0;i<numPcore;i++){
		freq_sum += pcoreContainer[i]->freq;
	}
	(freq_sum >= req_sum)?(scale = 1):(scale = freq_sum/req_sum);

	// Greedy Assignment
	// Assign vcpu to the most "efficient" pcpu with load less than 100%

	for(int i=0;i<numVcore;i++){
		// find the most efficeint pcpu with load less than 100%
		PHYS_CORE* target_pcore = NULL;

		for(int j=0;j<numPcore;j++){
			if(pcoreContainer[j]->load != 1.0){
				if((target_pcore != NULL) && (target_pcore->efficient >= pcoreContainer[j]->efficient)){
				}
				else{
					target_pcore = pcoreContainer[j];
				}
			}
		}
		if(target_pcore == NULL){
			// Something wrong...
			fprintf(stderr,"Cannot find any physical core.\n");
		}

		// assign vcpu to pcpu
		if(target_pcore->freq*(1-target_pcore->load) >= vcoreContainer[i]->requ*scale){
			double percentage = vcoreContainer[i]->requ*scale/target_pcore->freq;
			target_pcore->workload[vcoreContainer[i]->code] = percentage;
			target_pcore->load += percentage;
			vcoreContainer[i]->requ = 0;
		}
		else{
			target_pcore->workload[vcoreContainer[i]->code] = 1-target_pcore->load;
			vcoreContainer[i]->requ -= (target_pcore->freq*(1-target_pcore->load)/scale);
			target_pcore->load = 1.0;
			i--;
		}		
	};

	return scale;
}


void gen_schedule_plan(){	
	
#ifdef DEVELOPING
	numVcore = 8;
	numPcore = 6;
#else
	// get from hypervisor
	// numVcore =;
	// numPcore =;
#endif
	
	vcoreContainer = (VIRT_CORE**)malloc(sizeof(VIRT_CORE*)*numVcore);
	pcoreContainer = (PHYS_CORE**)malloc(sizeof(PHYS_CORE*)*numPcore);

#ifdef DEVELOPING
	VIRT_CORE* temp_c;
	for(int i=0;i<numVcore;i++){
		temp_c = (VIRT_CORE*)malloc(sizeof(VIRT_CORE));
		temp_c->domain = i/4;
		temp_c->num = i%4;
		temp_c->requ = 100000*i;
		temp_c->code = i;
		vcoreContainer[i] = temp_c;
	}

	PHYS_CORE* temp_p;
	for(int i=0;i<numPcore;i++){
		temp_p = (PHYS_CORE*)malloc(sizeof(PHYS_CORE));
		(i<2)?(temp_p->freq = 1200000):(temp_p->freq = 600000);
		(i<2)?(temp_p->type = TYPE_BIG):(temp_p->type = TYPE_LITTLE);
		temp_p->efficient = temp_p->type;
		temp_p->load = 0;
		temp_p->num = i;
		temp_p->workload = (double*)malloc(sizeof(double)*numVcore);
		for(int j=0;j<numVcore;j++){
			temp_p->workload[j] = 0;
		}
		pcoreContainer[i] = temp_p;
	}
#else
	// fetch vcpu and pcpu info
#endif

	// phase 1
	phase1();

	// phase 2

	// phase 3


}


int main(){

	gen_schedule_plan();
	
	return 0;
}
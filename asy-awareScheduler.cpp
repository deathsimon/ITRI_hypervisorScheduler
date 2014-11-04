#include "asy-awareScheduler.h"

VIRT_CORE** vcoreContainer;
PHYS_CORE** pcoreContainer;

static int numVcore;
static int numPcore;

static DecreSet decSet;
static ExecutionSlice eSlice;

/*
assginJob()
	find a match in phase 2
*/
bool assginJob(int* matching, int MachNum){//, Machine* machines){
	bool findAns = false;

	if(MachNum == numPcore){
		findAns = true;
		for(int i = 0; i < numPcore; i++){
			// check if every tight pcore is picked
			if(decSet.MachTight[i] && !decSet.MachPicked[i]){
				findAns = false;
				break;
			}
		}
		if(findAns){
			for(int i = 0; i < numVcore; i++){
				// check if every tight vcore is picked
				if(decSet.JobTight[i] && !decSet.JobPicked[i]){
					findAns = false;
					break;
				}
			}
		}
	}
	else{
		matching[MachNum] = -1;
		if(assginJob(matching,MachNum+1) == true){
			findAns = true;
		}
		else{
			for(int i = 0;i < numVcore; i++){
				if((decSet.JobPicked[i] == false) && (pcoreContainer[MachNum]->workload[i] != 0)){
					matching[MachNum] = i;
					decSet.MachPicked[MachNum] = true;
					decSet.JobPicked[i] = true;
					if(assginJob(matching,MachNum+1) == true){
						findAns = true;
						break;
					}
					else{
						decSet.JobPicked[i] = false;
						decSet.MachPicked[MachNum] = false;
					}
				}
			}
		}
	}

	return findAns;
}





/*
compareSlice()
	compare if two execution slices is the same
*/
bool compareSlice(int* target, int* obj, int size){
	bool match = true;
	for(int i=0;i<size;i++){
		if(target[i] != obj[i]){
			match = false;
			break;
		}	
	}
	return match;
}

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
			if(pcoreContainer[j]->load != TIMESLICE){
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
		double res_remain = target_pcore->freq*(1-((double)target_pcore->load/(double)TIMESLICE));
		if(res_remain >= vcoreContainer[i]->requ*scale){
			double percentage = vcoreContainer[i]->requ*scale/target_pcore->freq;
			target_pcore->workload[vcoreContainer[i]->code] = (int)ceil(percentage*TIMESLICE);
			target_pcore->load += (int)ceil(percentage*TIMESLICE);
			vcoreContainer[i]->requ = 0;
		}
		else{
			target_pcore->workload[vcoreContainer[i]->code] = TIMESLICE-target_pcore->load;
			vcoreContainer[i]->requ -= res_remain/scale;
			target_pcore->load = TIMESLICE;
			i--;
		}		
	};

	return scale;
}

/*
phase 2()
	
*/
int phase2(){
	
	int LowerBound = 0;
	int* vcoreLoad = (int*)malloc(sizeof(int)*numVcore);	
	int* matching = (int*)malloc(sizeof(int)*numPcore);
	ExecutionSlice* newSlice;
	int distinctES = 0;

	// init
	for(int i = 0;i < numVcore;i++){
		vcoreLoad[i] = 0;
	}		
	for(int i = 0;i < numPcore;i++){
		for(int j = 0;j < numVcore;j++){
			//pcoreContainer[i]->load += pcoreContainer[i]->workload[j];
			vcoreLoad[j] += pcoreContainer[i]->workload[j];
		}
	}
	decSet.JobTight = (bool*)malloc(sizeof(bool)*numVcore);
	decSet.MachTight = (bool*)malloc(sizeof(bool)*numPcore);
	decSet.JobPicked = (bool*)malloc(sizeof(bool)*numVcore);
	decSet.MachPicked = (bool*)malloc(sizeof(bool)*numPcore);

	while(1){	
		// find LB	
		LowerBound = 0;
		for(int i = 0;i < numPcore;i++){
			if(pcoreContainer[i]->load > LowerBound){
				LowerBound = pcoreContainer[i]->load;
			}
		}
		for(int i = 0;i < numVcore;i++){
			if(vcoreLoad[i] > LowerBound){
				LowerBound = vcoreLoad[i];
			}
		}
		if(LowerBound == 0.0){
			// no vcore, we are done in phase 2
			break;
		}

		// init decreSet
		decSet.delta = LowerBound;
		for(int i = 0; i < numVcore;i++){
			decSet.JobTight[i] = false;
			decSet.JobPicked[i] = false;
		}
		for(int i = 0; i < numPcore;i++){
			decSet.MachTight[i] = false;
			decSet.MachPicked[i] = false;
		}
		// find a decreSet
		for(int i = 0;i < numPcore;i++){
			if(pcoreContainer[i]->load == LowerBound){
				decSet.MachTight[i] = true;
			}
			else{
				decSet.MachTight[i] = false;
				if(decSet.delta > (LowerBound - pcoreContainer[i]->load)){
					decSet.delta = LowerBound - pcoreContainer[i]->load;
				}
			}
		}
		for(int i = 0;i < numVcore;i++){
			if(vcoreLoad[i] == LowerBound){
				decSet.JobTight[i] = true;
			}
			else{
				decSet.JobTight[i] = false;
				if(decSet.delta > (LowerBound - vcoreLoad[i])){
					decSet.delta = LowerBound - vcoreLoad[i];
				}
			}
		}

		// pick a matching		
		if(!assginJob(matching,0)){
			// something wrong
			fprintf(stderr,"!!!\n");
		}
		
		// compute delta
		for(int i = 0;i< numPcore; i++){
			if((matching[i] != -1) && (pcoreContainer[i]->workload[matching[i]] < decSet.delta)){
				decSet.delta = pcoreContainer[i]->workload[matching[i]];
			}
		}

		// reduct workload
		for(int i = 0;i < numPcore; i++){
			if(matching[i] != -1){
				pcoreContainer[i]->workload[matching[i]] -= decSet.delta;
				pcoreContainer[i]->load -= decSet.delta;
				vcoreLoad[matching[i]] -= decSet.delta;
			}			
		}

		// create an execution slice
		newSlice = (ExecutionSlice*)malloc(sizeof(ExecutionSlice));
		newSlice->timeslice = decSet.delta;
		newSlice->mapping = (int*)malloc(sizeof(int)*numPcore);
		memcpy(newSlice->mapping, matching, sizeof(int)*numPcore);

		// add to the list of execution slice
		ExecutionSlice* currSlice = &eSlice;
		while(currSlice->next != NULL){
			// compare
			if(compareSlice(currSlice->next->mapping, newSlice->mapping, numPcore)){
				// if the same
				currSlice->next->timeslice += newSlice->timeslice;
				free(newSlice->mapping);
				free(newSlice);
				break;
			}
			else{
				currSlice = currSlice->next;
			}
		};
		if(currSlice->next == NULL){			
			newSlice->prev = currSlice;
			newSlice->next = NULL;
			currSlice->next = newSlice;
			distinctES++;
		}
	};

	/*
	ExecutionSlice* currSlice = &eSlice;
	while(currSlice->next != NULL){
		for(int i=0;i<numPcore; i++){
			if(currSlice->next->mapping[i] != -1){
				fprintf(stdout,"%d ", currSlice->next->mapping[i]);
			}
			else{
				fprintf(stdout,"x ", currSlice->next->mapping[i]);
			}		
		}
		fprintf(stdout,"| %d\n", currSlice->next->timeslice);
		currSlice = currSlice->next;
	};
	*/

	// clean up
	free(decSet.MachPicked);
	free(decSet.JobPicked);
	free(decSet.MachTight);
	free(decSet.JobTight);

	free(matching);
	free(vcoreLoad);

	return distinctES;
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
		temp_c->requ = 100000*(i+1);
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
		temp_p->workload = (int*)malloc(sizeof(int)*numVcore);
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
	eSlice.next = NULL;
	phase2();

	// phase 3


}


int main(){

	gen_schedule_plan();
	
	return 0;
}
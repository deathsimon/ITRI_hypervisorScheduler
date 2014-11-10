#include "asy-awareScheduler.h"

VIRT_CORE** vcoreContainer;
PHYS_CORE** pcoreContainer;

static int numVcore;
static int numPcore;

static DecreSet decSet;
static ExecutionSlice eSlice;
static ExecutionSlice exePlan;

/*
outputSlices()
	output the contents of all slices
*/
void outputSlices(ExecutionSlice* currSlice){	
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
}

/*
assginJob()
	find a match in phase 2
*/
bool assginJob(int* matching, int MachNum){
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
	compare if two execution slices are the same
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
computeInc()
	compute the increases of the number of switching if inserting the current slice
*/
int computeInc(ExecutionSlice* currSlice, int* roadMap){
	int inc = 0;
	int vCore = 0;
	for(int i=0;i<numPcore;i++){
		vCore = currSlice->mapping[i];
		if((vCore != -1) && (roadMap[vCore] != -1) && (roadMap[vCore] != i)){
			inc++;
		}
	}
	return inc;
}
/*
interaction()

*/
int interaction(ExecutionSlice* slice){
	int times = 0;
	int vCore = 0;
	int* tempMap = (int*)malloc(sizeof(int)*numVcore);
	ExecutionSlice* currSlice = eSlice.next;

	for(int i=0;i<numVcore;i++){
		tempMap[i] = -1;
	}
	for(int i=0;i<numPcore;i++){
		vCore = slice->mapping[i];
		if(vCore != -1){
			tempMap[vCore] = i;
		}
	}
	while(currSlice != NULL){
		times += computeInc(currSlice, tempMap);
		currSlice = currSlice->next;
	};

	free(tempMap);

	return times;
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
			//target_pcore->workload[vcoreContainer[i]->code] = (int)ceil(percentage*TIMESLICE);
			target_pcore->workload[vcoreContainer[i]->code] = (int)(percentage*TIMESLICE);
			//target_pcore->load += (int)ceil(percentage*TIMESLICE);
			target_pcore->load += (int)(percentage*TIMESLICE);
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
	
	//outputSlices(&eSlice);	

	// clean up
	free(decSet.MachPicked);
	free(decSet.JobPicked);
	free(decSet.MachTight);
	free(decSet.JobTight);

	free(matching);
	free(vcoreLoad);

	return distinctES;
}

/*
phase 3

*/
int phase3(){
	
	int numSwitching = 0;
	int minIncrease = 0;
	int* roadMap = (int*)malloc(sizeof(int)*numVcore);

	ExecutionSlice* currSlice;
	ExecutionSlice* candSlice;

	ExecutionSlice* planTail = &exePlan;

	for(int i = 0;i<numVcore;i++){
		roadMap[i] = -1;
	}
	
	while(eSlice.next != NULL){
		// find the execution slice with the least marginal number of swtiching
		candSlice = eSlice.next;
		minIncrease = computeInc(candSlice, roadMap);
		currSlice = candSlice->next;
		
		while(currSlice != NULL){
			// compute increase 
			int numIncrease = computeInc(currSlice, roadMap); 
			if(numIncrease < minIncrease){
				candSlice = currSlice;
			}
			else if(numIncrease == minIncrease){
				if(interaction(candSlice) > interaction(currSlice)){
					candSlice = currSlice;
				}
			}
			currSlice = currSlice->next;
		};

		// update current status
		int vCore = 0;
		minIncrease += numSwitching;
		for(int i = 0; i<numPcore; i++){
			vCore = candSlice->mapping[i];
			if(vCore != -1){
				if((roadMap[vCore] != -1) && (roadMap[vCore] != i)){
					numSwitching++;
				}
				roadMap[vCore] = i;
			}
		}
		if(minIncrease != numSwitching){
			// something wrong
			fprintf(stderr, "Switching time anomaly\n");
		}

		// remove from queue
		candSlice->prev->next = candSlice->next;
		if(candSlice->next != NULL){
			candSlice->next->prev = candSlice->prev;
		}
		// insert candSlice into plan
		planTail->next = candSlice;
		candSlice->prev = planTail;
		candSlice->next = NULL;
		planTail = planTail->next;		
	};

	//outputSlices(&exePlan);

	free(roadMap);

	return numSwitching;
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


#ifdef DEVELOPING
	clock_t t;
	t = clock();
	// phase 1
	phase1();
	t = clock() - t;
	fprintf(stderr, "phase 1: %d ticks (%.3lf seconds).\n", t, ((double)t)/CLOCKS_PER_SEC);
	
	t = clock();
	// phase 2
	eSlice.next = NULL;
	phase2();
	t = clock() - t;
	fprintf(stderr, "phase 2: %d ticks (%.3lf seconds).\n", t, ((double)t)/CLOCKS_PER_SEC);

	t = clock();
	// phase 3
	phase3();
	t = clock() - t;
	fprintf(stderr, "phase 3: %d ticks (%.3lf seconds).\n", t, ((double)t)/CLOCKS_PER_SEC);
#else
	// phase 1
	phase1();

	// phase 2
	eSlice.next = NULL;
	phase2();

	// phase 3
	phase3();
#endif

	// clean up
	ExecutionSlice* target;
	while(exePlan.next != NULL){
		target = exePlan.next;
		exePlan.next = exePlan.next->next;
		free(target);
	};

	for(int i=0;i<numPcore;i++){
		free(pcoreContainer[i]);
	}
	free(pcoreContainer);

	for(int i=0;i<numVcore;i++){
		free(vcoreContainer[i]);
	}
	free(vcoreContainer);
}

int main(){

	gen_schedule_plan();
	
	return 0;
}
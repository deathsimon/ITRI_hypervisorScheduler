#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <asm/div64.h>
#include <xen/errno.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>

/*
 * Basic constants
 */
#define CORE_AMOUNT					6		/* 0~3: A53, 4~5: A57	*/
#define AMOUNT_EFFI					4

#define	ASYM_TIMESLICE_MS			25		/* timeslice in ms	*/
#define ASYM_INTERVAL_TS			40		/* # of timeslices in an interval 	*/

#define	TYPE_PERFORMANCE			0
#define	TYPE_EFFICIENCY				1

#define	FREQ_PERFORMANCE			1200	/* MHz	*/
#define	FREQ_EFFICIENCY				600

#define	DOM0_ID						0

#define	ASYM_DEBUG

#ifdef	ASYM_DEBUG
#define DEBUGMSG(msg)	printk(msg)
#else
#define DEBUGMSG(msg)
#endif

/* From xen/common/vcpu_freq.c by Isaac	*/
#define NUM_DOMAIN      16
#define MAX_VCPU        16
extern uint64_t freq_amp[NUM_DOMAIN][MAX_VCPU];

/*
 * Useful macros
 */
#define ASYM_PRIV(_ops)   \
    ((struct asym_private *)((_ops)->sched_data))
#define ASYM_PCPU(_c)     \
    ((struct asym_pcpu *)per_cpu(schedule_data, _c).sched_priv)
#define ASYM_VCPU(_vcpu)  ((struct asym_vcpu *) (_vcpu)->sched_priv)
#define ASYM_DOM(_dom)    ((struct asym_dom *) (_dom)->sched_priv)
#define RUNQ(_cpu)          (&(ASYM_PCPU(_cpu)->runq))
/* Is the first element of _cpu's runq its idle vcpu? */
#define IS_RUNQ_IDLE(_cpu)  (list_empty(RUNQ(_cpu)) || \
                             is_idle_vcpu(__runq_elem(RUNQ(_cpu)->next)->vcpu))

/*
 * Physical CPU
 */
//DEFINE_PER_CPU(type, name);
DEFINE_PER_CPU(unsigned int, curr_slice);
struct asym_pcpu {
	struct list_head runq;
	struct timer ticker;
	/* for scheduling algorithm*/
	struct asym_pcpuinfo{
		unsigned int type;	
		unsigned int provRes;			/* in MHz	*/
		unsigned int load;
		unsigned int* workloads;
		//unsigned int curr_slice;	/* record current time slice in an interval	*/
	} info;
};
/*
 * Virtual CPU
 */
struct asym_vcpu {
    struct list_head runq_elem;	/* on the runq list	*/
    struct list_head sdom_elem;	/* on the domain VCPU list	*/
	/* Up-pointers	*/
    struct asym_dom *sdom;
    struct vcpu *vcpu;
	unsigned flags;
	unsigned int on_cpu;
	/* for scheduling algorithm*/
	struct asym_vcpuinfo{
		unsigned int vcpuNum;
		unsigned int requRes;			/* in MHz	*/
		struct list_head plan;		/* excution period on a pcpu	*/
		struct list_head act_elem;	/* on the active VCPU list*/
	} info;	
};
/*
 * Domain
 */
struct asym_dom {
    struct list_head vcpu;		/* link its VCPUs	*/
    struct list_head sdom_elem;	/* link list on asym_priv	*/
    struct domain *dom;			/* pointer to upper domain	*/
};
/*
 * System-wide private data
 */
struct asym_private {
    /* lock for the whole pluggable scheduler, nests inside cpupool_lock */
    spinlock_t lock;
    
	struct list_head sdom;    
	/* global timer that triggers the scheduling algorithm */
    struct timer timer_gen_plan;
    /* physical core	*/
	unsigned int master;
	struct asym_pcpu* cpuArray[CORE_AMOUNT];
    	
    //cpumask_var_t cpus;
};
/*
 * Decrementing set in Open Shop Scheduling
 */
struct asym_decSet{
	unsigned int numPCPU;
	unsigned int numVCPU;
	int delta;
	bool_t* MachTight;
	bool_t* JobTight;
	bool_t* MachPicked;
	bool_t* JobPicked;
	struct asym_pcpu** cpu;
};
/*
 * Execution slice
 * the pcpu-vcpu realtionship of each pcpu during a preiod of time
 */
struct asym_exeSlice{
	int timeslice;
	int* mapping;
	struct list_head slice_elem;	/* on the slice list*/
};
/*
 * Execution plan element
 * vcpu keeps a list of element, each with the target pcpu and execution length
 */
struct asym_vcpuPlan{
	unsigned int pcpu;
	unsigned int start_slice;
	unsigned int end_slice;
	struct list_head plan_elem;	/* on the plan list*/
};
static void asym_tick(void *_cpu);
static void asym_gen_plan(void *dummy);
static bool_t assignV2P(int* matching, int level, struct asym_decSet *decSet);
static inline struct asym_vcpuPlan* __plan_elem(struct asym_vcpu* svc);
/*
 * Inline functions from credit-base scheduler
 */
static inline int
__vcpu_on_runq(struct asym_vcpu *svc)
{
	/* list_empty(const struct list_head *head) */
    return !list_empty(&svc->runq_elem);
}

static inline struct asym_vcpu *
__runq_elem(struct list_head *elem)
{
	/* list_entry(ptr,type,member) */
    return list_entry(elem, struct asym_vcpu, runq_elem);
}

static inline void
__runq_insert(unsigned int cpu, struct asym_vcpu *svc)
{
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;
	unsigned int vcpu_start, curr_start;
	struct asym_vcpuPlan* planElem = NULL;

    BUG_ON( __vcpu_on_runq(svc) );
    BUG_ON( cpu != svc->vcpu->processor );

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] insert vcore %i into the run queue on core %i \n",
		svc->vcpu->vcpu_id, cpu);
#endif
	svc->on_cpu = cpu;
	planElem = __plan_elem(svc);
	if(planElem != NULL){
		vcpu_start = planElem->start_slice;
		/* list_for_each(pos, head)  */
		list_for_each( iter, runq )
		{
			struct asym_vcpu * iter_svc = __runq_elem(iter);
			planElem = __plan_elem(iter_svc);
			if(planElem != NULL){
				curr_start = planElem->start_slice;
				/* insert according to the start_time of the vcpu	*/
				if ( vcpu_start < curr_start )
					break;
			}
		}	
	}
	else{
		iter = RUNQ(cpu);
	}
	list_add_tail(&svc->runq_elem, iter);
	BUG_ON( !__vcpu_on_runq(svc) );
}

static inline void
__runq_remove(struct asym_vcpu *svc)
{
    BUG_ON( !__vcpu_on_runq(svc) );

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] remove vcore %i from the run queue\n",
		svc->vcpu->vcpu_id);
#endif
    list_del_init(&svc->runq_elem);
	svc->on_cpu = UINT_MAX;
}
static inline void
__runq_migrate(unsigned int cpu, struct asym_vcpu *svc)
{
	struct asym_vcpuPlan* elem;
	unsigned int target;

	__runq_remove(svc);
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] migrate vcore %i to ", 
		svc->vcpu->vcpu_id);
#endif	
	elem = __plan_elem(svc);
	(elem != NULL)?(target = elem->pcpu):(target = cpu);
#ifdef	ASYM_DEBUG
	printk("core %i\n", target);
#endif
	__runq_insert(target, svc);
}
static inline void
__runq_check(unsigned int cpu)
{
	const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter, *iter_next;
	struct asym_vcpu * iter_svc;

	list_for_each_safe( iter, iter_next, runq )
	{
		iter_svc = __runq_elem(iter);
		if(iter_svc->on_cpu != cpu){
			__runq_migrate(cpu, iter_svc);
		}
	}
}
static inline void
__runq_tickle(unsigned int cpu, struct asym_vcpu *new)
{
    /* After inserting a vcpu to the runqueue of a pcpu, 
	 * raise softirq to tell the cpu to re-schedule.
	 */
	DEBUGMSG("[SCHED_ASYM] cpu_raise_softirq() in __runq_tickle()\n");
	cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);   	
}
/* get the first plan element of a virtual core
 * return NULL if there is no plan element
 */
static inline struct asym_vcpuPlan* 
__plan_elem(struct asym_vcpu* svc)
{
	struct asym_vcpuPlan* vplan;
	if(!list_empty(&svc->info.plan)){
		vplan = list_entry(svc->info.plan.next, struct asym_vcpuPlan, plan_elem);
	}
	else{
		vplan = NULL;
	}
	return vplan;
}
static inline struct asym_vcpuPlan* 
__pop_elem(struct asym_vcpu* svc)
{
	struct asym_vcpuPlan* vplan = NULL;

	BUG_ON(list_empty(&svc->info.plan));

	list_del_init(svc->info.plan.next);
	if(!list_empty(&svc->info.plan)){
		vplan = list_entry(svc->info.plan.next, struct asym_vcpuPlan, plan_elem);
	}

	return vplan;
}
/*	get the type of a pcpu	*/
static inline int
__fetch_core_type(unsigned int cpu){
	unsigned int type = UINT_MAX;
	/* for JUNO board	*/
	(cpu >= AMOUNT_EFFI)?(type = TYPE_PERFORMANCE):(type = TYPE_EFFICIENCY);
	/* for general cases, 
	 * Victor: To get the physical CPU type info, you can access it in Xen by get part_number field from MIDR register with current_cpu_data.midr.part_number
	 */	
	return type;
}
static inline int
__fetch_vcore_frequ(struct vcpu *vcpu){
	unsigned int freq;
	
	BUG_ON( is_idle_vcpu(vcpu) );
	freq = freq_amp[vcpu->domain->domain_id][vcpu->vcpu_id];
	DEBUGMSG("__fetch_vcore_freq()\n");
	return freq;
}
static inline int
__fetch_pcore_frequ(unsigned int cpu){
	unsigned int freq = UINT_MAX;
	/* for JUNO board	*/
	(cpu >= AMOUNT_EFFI)?(freq = FREQ_PERFORMANCE):(freq = FREQ_EFFICIENCY);
	/* for general cases, 
	 * TODO
	 */	
	return freq;
}

static void
asym_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
	struct asym_private *prv = ASYM_PRIV(ops);
    struct asym_pcpu *spc = pcpu;
	struct asym_vcpu *svc;
	struct list_head *iter, *iter_next;
    unsigned long flags;

	if ( spc == NULL )
        return;

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] free pdata of cpu %i.\n", cpu);
#endif

	spin_lock_irqsave(&prv->lock, flags);
	
	/* remove the pcpu from the pcpu array */
	prv->cpuArray[cpu] = NULL;
	/* clean runq */	
	list_for_each_safe(iter, iter_next, &spc->runq){
		svc = list_entry(iter, struct asym_vcpu, runq_elem);
		list_del_init(&svc->runq_elem);
	};
	/* remove timer */
	kill_timer(&spc->ticker);
	if (prv->master == cpu){
		kill_timer(&prv->timer_gen_plan);
	}

	spin_unlock_irqrestore(&prv->lock, flags);

    xfree(spc);
}

static void *
asym_alloc_pdata(const struct scheduler *ops, int cpu)
{
	struct asym_pcpu *spc;
    struct asym_private *prv = ASYM_PRIV(ops);
    unsigned long flags;	

    /* Allocate per-PCPU info */
    spc = xzalloc(struct asym_pcpu);
    if ( spc == NULL )
        return NULL;

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] Allocating pdata of cpu %i.\n", cpu);
#endif

	spin_lock_irqsave(&prv->lock, flags);

    /* Initialize/update system-wide config */
	
	/* set timer	*/
	if(cpu == prv->master){
		init_timer(&prv->timer_gen_plan, asym_gen_plan, prv, cpu);
		set_timer(&prv->timer_gen_plan,
			NOW() + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS));
	}
	init_timer(&spc->ticker, asym_tick, (void *)(unsigned long)cpu, cpu);
    set_timer(&spc->ticker, NOW() + MILLISECS(ASYM_TIMESLICE_MS) );		
	
	/* set per-CPU timeslice	*/
	get_cpu_var(curr_slice) = 0;
	put_cpu_var(curr_slice);

	/* set info	*/
	spc->info.type = __fetch_core_type(cpu);
	spc->info.provRes = 0;
	spc->info.load = 0;
	spc->info.workloads = NULL;

	/* Initialize runqueue	*/
    INIT_LIST_HEAD(&spc->runq);

	prv->cpuArray[cpu] = spc;
    
	if ( per_cpu(schedule_data, cpu).sched_priv == NULL )
        per_cpu(schedule_data, cpu).sched_priv = spc;

    /* Start off idling... */
    BUG_ON(!is_idle_vcpu(curr_on_cpu(cpu)));    

    spin_unlock_irqrestore(&prv->lock, flags);

	DEBUGMSG("[SCHED_ASYM] allocate pdata successful.\n");

    return spc;
}

static int
asym_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
	/* Since the scheduling plan already decides which cpus a vcpu can run on,
	 * pick_cpu only return the current physical cpu of the vcpu.
	 */
	int cpu;

	cpu = vc->processor;

	printk("[SCHED_ASYM] cpu_pick()\n");

	return cpu;
}
static void *
asym_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
	struct asym_vcpu *svc;

    /* Allocate per-VCPU info */
    svc = xzalloc(struct asym_vcpu);
    if ( svc == NULL )
        return NULL;

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->sdom_elem);
    svc->sdom = dd;
    svc->vcpu = vc;

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] Allocating vdata of vcpu %i.\n", vc->vcpu_id);
	//printk("\t Belongs to domain %i.\n", vc->domain->domain_id);	
#endif

	if(svc->sdom != NULL){
#ifdef	ASYM_DEBUG
		printk("\t Belongs to domain %i.\n", svc->sdom->dom->domain_id);
#endif
		list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);
	}
	else{
		DEBUGMSG("\t This is an idle vcpu.\n");
	}

	svc->on_cpu = UINT_MAX;
	svc->info.vcpuNum = UINT_MAX;
	svc->info.requRes = UINT_MAX;
	INIT_LIST_HEAD(&svc->info.plan);
	INIT_LIST_HEAD(&svc->info.act_elem);
	
	//SCHED_VCPU_STATS_RESET(svc);
	SCHED_STAT_CRANK(vcpu_init);

	DEBUGMSG("[SCHED_ASYM] allocate vdata successful.\n");

    return svc;
}

static void
asym_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct asym_vcpu *svc = vc->sched_priv;

	DEBUGMSG("[SCHED_ASYM] asym_vcpu_insert().\n");

	if ( is_idle_vcpu(vc) )
        return;

	if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )
		__runq_insert(vc->processor, svc);

	/* if this vcpu belong does not belong to any dom, somethings wrong */	
	/*
	if(!list_empty(&svc->sdom_elem))
		list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);		
	
	BUG_ON(list_empty(&svc->sdom_elem));
	*/
}

static void
asym_free_vdata(const struct scheduler *ops, void *priv)
{
	struct asym_vcpu *svc = priv;

    BUG_ON( !list_empty(&svc->runq_elem) );

    xfree(svc);
}

static void
asym_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{	
    struct asym_vcpu * const svc = ASYM_VCPU(vc);
    struct asym_dom * const sdom = svc->sdom;
    //unsigned long flags;

	SCHED_STAT_CRANK(vcpu_destroy);

	if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);

    if ( !list_empty(&svc->sdom_elem) ){
		/* remove the vcpu from domain	*/
		list_del_init(&svc->sdom_elem);
	}

	if ( !list_empty(&svc->info.act_elem) ){
		/* remove the vcpu from the active vcpu list	*/
		list_del_init(&svc->info.act_elem);
	}

	BUG_ON( sdom == NULL );
    BUG_ON( !list_empty(&svc->runq_elem) );
	BUG_ON( !list_empty(&svc->info.act_elem) );
}

static void
asym_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
	struct asym_vcpu * const svc = ASYM_VCPU(vc);

    SCHED_STAT_CRANK(vcpu_sleep);

    BUG_ON( is_idle_vcpu(vc) );

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] vcpu_sleep() is called by vcpu %i.\n", vc->vcpu_id);
#endif

	if ( curr_on_cpu(vc->processor) == vc ){
		/* if running on pcpu	*/
		DEBUGMSG("[SCHED_ASYM] cpu_raise_softirq() in asym_vcpu_sleep()\n");
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);	
	}
	else if ( __vcpu_on_runq(svc) ){
		/* only on runqueue	*/
        __runq_remove(svc);
	}
}

static void
asym_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
	struct asym_vcpu * const svc = ASYM_VCPU(vc);
    const unsigned int cpu = vc->processor;

	BUG_ON( is_idle_vcpu(vc) );

#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] vcpu_wake() is called by vcpu %i\n", vc->vcpu_id);
#endif

	if ( unlikely(curr_on_cpu(cpu) == vc) )
    {
        SCHED_STAT_CRANK(vcpu_wake_running);
        return;
    }
    if ( unlikely(__vcpu_on_runq(svc)) )
    {
        SCHED_STAT_CRANK(vcpu_wake_onrunq);
        return;
    }

    if ( likely(vcpu_runnable(vc)) )
        SCHED_STAT_CRANK(vcpu_wake_runnable);
    else
        SCHED_STAT_CRANK(vcpu_wake_not_runnable);

	/* Put the VCPU into the runq, and tirgger the re-scheduling on the cpu	*/
	__runq_insert(cpu, svc);
	__runq_tickle(cpu, svc);	
}

static void
asym_vcpu_yield(const struct scheduler *ops, struct vcpu *vc)
{
	/* one solution is to switch the execution slice with another vcpu
	 * that will not violate the constraints of the scheduling plan.
	 * However, this is too complicate.
	 * Therefore, do nothing for now.
	 */
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] vcpu_yield() is called by vcpu %i.\n", vc->vcpu_id);
#endif
	__runq_migrate(vc->processor, ASYM_VCPU(vc));
}

static int
asym_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
	struct xen_domctl_scheduler_op *op)
{
	//struct asym_dom * const sdom = ASYM_DOM(d);
    //struct asym_vcpu *svc;
    //struct list_head *iter;

	printk("Here we are in asym_dom_cntl()\n"
		   "\tcmd:%d\n",op->cmd);

	/* ???
    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_getinfo:
		break;
    case XEN_DOMCTL_SCHEDOP_putinfo:		
        break;
    }
	*/

	return 0;
}

static int
asym_sys_cntl(const struct scheduler *ops,
                        struct xen_sysctl_scheduler_op *sc)
{
	printk("Here we are in asym_sys_cntl()\n"
		   "\tcmd:%d\n",sc->cmd);

	/* ???		
    switch ( sc->cmd )
    {
    case XEN_SYSCTL_SCHEDOP_putinfo:
		break;
	case XEN_SYSCTL_SCHEDOP_getinfo:
		break;
	}
	*/
	return 0;
}
/* Domain related actions	*/
static void *
asym_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{		
    struct asym_dom *sdom;
    struct asym_private * prv = ASYM_PRIV(ops);
	unsigned long flags;

    sdom = xzalloc(struct asym_dom);
    if ( sdom == NULL )
        return NULL;
    
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] Allocating domdata for domain %i\n", dom->domain_id);
#endif

    INIT_LIST_HEAD(&sdom->vcpu);
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;

	/* spinlock here to insert the dom */
    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&sdom->sdom_elem, &prv->sdom);
    spin_unlock_irqrestore(&prv->lock, flags);

	DEBUGMSG("[SCHED_ASYM] allocate domdata successful.\n");

    return (void *)sdom;
}

static int
asym_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct asym_dom *sdom;

    if ( is_idle_domain(dom) )
        return 0;

    sdom = asym_alloc_domdata(ops, dom);
    if ( sdom == NULL )
        return -ENOMEM;

    dom->sched_priv = sdom;

    return 0;
}

static void
asym_free_domdata(const struct scheduler *ops, void *data)
{
	unsigned long flags;
    struct asym_dom *sdom = data;
    struct asym_private *prv = ASYM_PRIV(ops);

    spin_lock_irqsave(&prv->lock, flags);
    list_del_init(&sdom->sdom_elem);
    spin_unlock_irqrestore(&prv->lock, flags);
	xfree(data);
}

static void
asym_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
	asym_free_domdata(ops, ASYM_DOM(dom));
}

/* The physica core efficieness	*/
static inline unsigned int
__pcpu_effi(struct asym_pcpu *spc){
	/* For now, justify accroding to core type	*/	
	return spc->info.type;
}
static inline void
__dump_plan(void){
	/* TODO */
	//printk("[SCHED_ASYM] Dump plan\n");
}

/* return the virtual core in the list with the specific vcpu number	*/
static inline struct asym_vcpu *
__fetch_vcpu_num(int num, struct list_head *active_vcpu){	
	struct asym_vcpu *target_vcpu = NULL;		
	/*list_for_each_entry(type *cursor, struct list_head *list, member)	*/
	list_for_each_entry(target_vcpu, active_vcpu, info.act_elem)
	{
		if(target_vcpu->info.vcpuNum == num)
			break;
	}
	return target_vcpu;
}
static inline unsigned int
__computeInc(struct asym_exeSlice *currSlice, int *roadMap, int amountPCPU){
	unsigned int inc = 0;
	unsigned int vCore = 0;
	for(int i = 0; i < amountPCPU; i++){
		vCore = currSlice->mapping[i];
		if((vCore != -1) && (roadMap[vCore] != -1) && (roadMap[vCore] != i)){
			inc++;
		}
	}
	return inc;	
}
static inline unsigned int
__interaction(struct asym_exeSlice *slice, struct list_head *exeSlice, int amountPCPU, int amountVCPU){
	unsigned int times = 0;
	unsigned int vCore = 0;
	struct asym_exeSlice* currSlice;	
	int* tempMap = xzalloc_array(int, amountVCPU);

	for(int i=0; i < amountVCPU; i++){
		tempMap[i] = -1;
	}
	for(int i=0; i < amountPCPU; i++){
		vCore = slice->mapping[i];
		if(vCore != -1){
			tempMap[vCore] = i;
		}
	}
	
	list_for_each_entry(currSlice, exeSlice, slice_elem){
		times += __computeInc(currSlice, tempMap, amountPCPU);
	};

	xfree(tempMap);

	return times;
}

static bool_t
assignV2P(int *matching, int level, struct asym_decSet *decSet){
	bool_t findAns = false;

	if(level == decSet->numPCPU){
		findAns = true;
		for(int i = 0; i < decSet->numPCPU; i++){
			// check if every tight pcore is picked
			if(decSet->MachTight[i] && !decSet->MachPicked[i]){
				findAns = false;
				break;
			}
		}
		if(findAns){
			for(int i = 0; i < decSet->numVCPU; i++){
				// check if every tight vcore is picked
				if(decSet->JobTight[i] && !decSet->JobPicked[i]){
					findAns = false;
					break;
				}
			}
		}
	}
	else{
		matching[level] = -1;
		if(assignV2P(matching, level+1, decSet) == true){
			findAns = true;
		}
		else{
			for(int i = 0;i < decSet->numVCPU; i++){
				if((decSet->JobPicked[i] == false) && (decSet->cpu[level]->info.workloads[i] != 0)){
					matching[level] = i;
					decSet->MachPicked[level] = true;
					decSet->JobPicked[i] = true;
					if(assignV2P(matching, level+1, decSet) == true){
						findAns = true;
						break;
					}
					else{
						decSet->JobPicked[i] = false;
						decSet->MachPicked[level] = false;
					}
				}
			}
		}
	}

	return findAns;
}

/* Greedy Assignment
 * Assign vcpu to the most "efficient" pcpu with load less than 100%
 */
static void
__phase_1(struct asym_private *prv, struct list_head *active_vcpu){
	struct asym_vcpu *svc;
	struct asym_pcpu *cand_cpu;
	struct list_head *iter_svc;
	uint32_t percentage;

	DEBUGMSG("[SCHED_ASYM] Phase 1 start!\n");

	iter_svc = active_vcpu->next;
	while( iter_svc != active_vcpu )
	{
		svc = list_entry(iter_svc, struct asym_vcpu, info.act_elem);
		cand_cpu = NULL;
		for(int i = 0; i < CORE_AMOUNT; i++){
			/* if current pcpu is for dom0 or is fully loaded, pass	*/
			if(i == prv->master)
				continue;
			if(prv->cpuArray[i]->info.load == ASYM_INTERVAL_TS)
				continue;
			if( cand_cpu == NULL
				|| (__pcpu_effi(cand_cpu) < __pcpu_effi(prv->cpuArray[i])))
			{
				cand_cpu = prv->cpuArray[i];
			}
		}
		if(cand_cpu == NULL){
			printk("[SCHED_ASYM] Cannot find a propoer physical core for the virtual core\n");
			break;
		}
		/* assign vcpu to pcpu	*/				
		percentage = 0U;
		if(cand_cpu->info.provRes >= svc->info.requRes){
			percentage = svc->info.requRes/cand_cpu->info.provRes;
			cand_cpu->info.workloads[svc->info.vcpuNum] = (unsigned int)(percentage*ASYM_INTERVAL_TS);
			cand_cpu->info.load += (unsigned int)(percentage*ASYM_INTERVAL_TS);
			svc->info.requRes = 0;
			iter_svc = iter_svc->next;
		}
		else{
			cand_cpu->info.workloads[svc->info.vcpuNum] = ASYM_INTERVAL_TS-cand_cpu->info.load;
			svc->info.requRes -= cand_cpu->info.provRes;
			cand_cpu->info.load = ASYM_INTERVAL_TS;
		}
	};
}
/* Phase 2
 * Open-shop scheduling
 */
static void
__phase_2(struct asym_private *prv, struct list_head* exeSlice, unsigned int amountPCPU, unsigned int amountVCPU){
	
	int	LowerBound;
	int *assignment;
	unsigned int *vcoreLoad;
	
	struct asym_decSet decSet;
	struct asym_exeSlice *new_slice;
	
	vcoreLoad = xzalloc_array(unsigned int, amountVCPU);
	assignment = xzalloc_array(int, amountPCPU);

	DEBUGMSG("[SCHED_ASYM] Phase 2 start!\n");

	/* descrete set	*/
	decSet.MachTight = (bool_t*)xzalloc_array(bool_t, amountPCPU);
	decSet.JobTight = (bool_t*)xzalloc_array(bool_t, amountVCPU);
	decSet.MachPicked = (bool_t*)xzalloc_array(bool_t, amountPCPU);
	decSet.JobPicked = (bool_t*)xzalloc_array(bool_t, amountVCPU);	

	decSet.numPCPU = amountPCPU;
	decSet.numVCPU = amountVCPU;
	decSet.cpu = &prv->cpuArray[0];

	/* Initialization	*/
	for(int i = 0;i < amountVCPU;i++){
		vcoreLoad[i] = 0;
	}		
	for(int i = 0;i < amountPCPU; i++){
		for(int j = 0;j < amountVCPU; j++){			
			vcoreLoad[j] += prv->cpuArray[i]->info.workloads[j];
		}
	}

	while(1){
		/* find lower bound	*/
		LowerBound = 0;
		for(int i = 0;i < amountPCPU;i++){
			if((prv->master != i)
				&& (prv->cpuArray[i]->info.load > LowerBound)){
				LowerBound = prv->cpuArray[i]->info.load;
			}
		}
		for(int i = 0;i < amountVCPU;i++){
			if(vcoreLoad[i] > LowerBound){
				LowerBound = vcoreLoad[i];
			}
		}
		if(LowerBound == 0){
			// no vcore, we are done in phase 2
			break;
		}
		// init decreSet
		decSet.delta = LowerBound;
		for(int i = 0 ; i < amountPCPU; i++){
			decSet.MachTight[i] = false;
			decSet.MachPicked[i] = false;
		}
		for(int i = 0 ; i < amountVCPU; i++){
			decSet.JobTight[i] = false;
			decSet.JobPicked[i] = false;
		}
		
		// find a decreSet
		for(int i = 0;i < amountPCPU;i++){
			if(prv->master == i)
				continue;
			if(prv->cpuArray[i]->info.load == LowerBound){
				decSet.MachTight[i] = true;
			}
			else{
				decSet.MachTight[i] = false;
				if(decSet.delta > (LowerBound - prv->cpuArray[i]->info.load)){
					decSet.delta = LowerBound - prv->cpuArray[i]->info.load;
				}
			}
		}
		for(int i = 0;i < amountVCPU;i++){
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

		// pick an ansignment
		if(!assignV2P(assignment, 0, &decSet)){
			// something wrong
			printk("Cannot find an ansignment on virtual core to physical core!\n");
		}
		// compute delta
		for(int i = 0;i< amountPCPU; i++){
			if(prv->master == i)
				continue;
			if((assignment[i] != -1) && (prv->cpuArray[i]->info.workloads[assignment[i]] < decSet.delta)){
				decSet.delta = prv->cpuArray[i]->info.workloads[assignment[i]];
			}
		}
		// reduct workload
		for(int i = 0;i < amountPCPU; i++){
			if(prv->master == i)
				continue;
			if(assignment[i] != -1){
				prv->cpuArray[i]->info.workloads[assignment[i]] -= decSet.delta;
				prv->cpuArray[i]->info.load -= decSet.delta;
				vcoreLoad[assignment[i]] -= decSet.delta;
			}			
		}

		// create an execution slice
		new_slice = xzalloc(struct asym_exeSlice);
		new_slice->timeslice = decSet.delta;
		new_slice->mapping = xzalloc_array(int, amountPCPU);
		for(int i = 0;i < amountPCPU; i++){
			new_slice->mapping[i] = assignment[i];
		}
		/* add to the list of execution slice
		 * Assume that there will be no duplication.
		 */
		DEBUGMSG("[SCHED_ASYM] Add a new slice into the list.\n");
		list_add_tail(&new_slice->slice_elem, exeSlice);
	};
	/* free allocated memory in phase 2	*/
	xfree(decSet.MachTight);
	xfree(decSet.JobTight);
	xfree(decSet.MachPicked);
	xfree(decSet.JobPicked);
	xfree(assignment);
	xfree(vcoreLoad);
}
static void
__phase_3(struct list_head *exeSlice, struct list_head *exePlan, unsigned int amountPCPU, unsigned int amountVCPU){
	
	unsigned int numSwitching = 0;
	unsigned int numIncrease, minIncrease = 0;	
	
	int* roadMap;
	struct asym_exeSlice *candSlice, *currSlice;
		
	struct list_head *iter_slice;
	unsigned int vCore = 0;

	DEBUGMSG("[SCHED_ASYM] Phase 3 start!\n");

	roadMap = xzalloc_array(int, amountVCPU);	
	for(int i = 0; i < amountVCPU; i++){
		roadMap[i] = -1;
	}

	while(!list_empty(exeSlice)){
		iter_slice = exeSlice->next;
		candSlice = list_entry(iter_slice, struct asym_exeSlice, slice_elem);		
		minIncrease = __computeInc(candSlice, roadMap, amountPCPU);

		DEBUGMSG("[SCHED_ASYM] Choosing a candidate\n");

		/* find the next candidate slice */
		list_for_each_entry(currSlice, exeSlice, slice_elem){
			// compute increase			
			numIncrease = __computeInc(currSlice, roadMap, amountPCPU); 
			if(numIncrease < minIncrease){
				candSlice = currSlice;
			}
			else if(numIncrease == minIncrease){
				if(__interaction(candSlice, exeSlice, amountPCPU, amountVCPU) > __interaction(currSlice, exeSlice, amountPCPU, amountVCPU)){
					candSlice = currSlice;
				}
			}
			iter_slice = iter_slice->next;
		}
		
		DEBUGMSG("[SCHED_ASYM] Candidate found!\n");

		/* update current status	*/
		vCore = 0;
		minIncrease += numSwitching;
		for(int i = 0; i < amountPCPU; i++){
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
			printk("[SCHED_ASYM] Switching time anomaly\n");
		}
				
		/* remove from queue and insert candSlice into plan	*/
		list_move_tail(&candSlice->slice_elem, exePlan);

		DEBUGMSG("[SCHED_ASYM] Insert candidate sucessful!\n");
	};
	xfree(roadMap);
}

static void
asym_gen_plan(void *dummy){
	/* declarations: general */

	struct asym_private *prv = dummy;
	struct asym_dom *sdom, *dom0 = NULL;
	struct asym_vcpu *svc;
	struct asym_pcpu *spc;
	unsigned long flags;

	struct list_head active_vcpu;
	struct list_head *iter_sdom, *iter_svc;
	
	unsigned int amountPCPU = 0;
	unsigned int amountVCPU = 0, dom0VCPU = 0;	

	/* declarations: phase 1 */	
	unsigned int prov, requ;
	unsigned int total_prov = 0, total_requ = 0;
	uint16_t scale = 0;	

	/* declarations: phase 2 */	
	struct list_head exeSlice;
	
	/* declarations: phase 3 */
	struct list_head exePlan;

	/* declarations: final */
	struct asym_exeSlice *currSlice;
	struct asym_vcpuPlan *newPlanElem;
	unsigned int now = 0;

	DEBUGMSG("[SCHED_ASYM] Start generating a scheduling plan.\n");

	spin_lock_irqsave(&prv->lock, flags);

	INIT_LIST_HEAD(&active_vcpu);

	/* First, fetch the frequencies of the vCPUs,
	 * and compute the total resource required
	 */		
	list_for_each( iter_sdom, &prv->sdom ){
		/* list_entry(ptr,type,member) */
		sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		if(sdom->dom->domain_id == DOM0_ID){
			DEBUGMSG("[SCHED_ASYM] found dom0.\n");
			dom0 = sdom;
		}		
		list_for_each( iter_svc, &sdom->vcpu )
		{			
			svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);			
			if(sdom->dom->domain_id == DOM0_ID){				
				dom0VCPU++;
			}
			else{
				/* accumulate total requirement	*/
				requ = __fetch_vcore_frequ(svc->vcpu);
				svc->info.requRes = requ;
				total_requ += requ;
				/* add vcpu into active vcpu list	*/
				INIT_LIST_HEAD(&svc->info.act_elem);
				list_add_tail(&svc->info.act_elem, &active_vcpu);
				/* assign vcpu number	*/
				svc->info.vcpuNum = amountVCPU;
				amountVCPU++;
			}
        }
	}

	/* Then fetch the frequencies of the pCPUs,
	 * and compute the total resource required.
	 * Also allocate the memory of vCPUs on each pCPU
	 */	
	for(int i = 0; i < CORE_AMOUNT; i++){
		if((i != prv->master) 
			&& (prv->cpuArray[i] != NULL)){
			spc = prv->cpuArray[i];
			spc->info.load = 0;
			/* accumulate total provision	*/
			prov = __fetch_pcore_frequ(i);
			spc->info.provRes = prov;
			total_prov += prov;
			spc->info.workloads = xzalloc_array(unsigned int, amountVCPU);
			amountPCPU++;
		}
	}	

	/* Phase 1
	 * compute scale factor
	 * scale: number of right shift on resource requirement
	 */
	scale = 0;
	while(total_prov < total_requ){
		total_requ = total_requ>>1;
		scale++;
	};
	list_for_each_entry(svc, &active_vcpu, info.act_elem){
		svc->info.requRes = svc->info.requRes>>scale;
	}

	__phase_1(prv, &active_vcpu);

	/* Phase 2
	 * Open-shop scheduling
	 */
	INIT_LIST_HEAD(&exeSlice);
	__phase_2(prv, &exeSlice, amountPCPU, amountVCPU);
	
	/* Phase 3
	 * 
	 */
	INIT_LIST_HEAD(&exePlan);
	__phase_3(&exeSlice, &exePlan, amountPCPU, amountVCPU);
	
	/* finalize	
	 * assign the whole plan to each virtual core
	 */	
	DEBUGMSG("[SCHED_ASYM] finish three phases.\n");

	/* clean up the plan_element of each vritual core	*/
	list_for_each( iter_sdom, &prv->sdom ){
		/* list_entry(ptr,type,member) */
		sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		list_for_each( iter_svc, &sdom->vcpu )
		{
			svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);			
			INIT_LIST_HEAD(&svc->info.plan);			
        }
	}
	DEBUGMSG("[SCHED_ASYM] Assigning vcpus.\n");

	/* 
	 * Assignment of vcpu in dom0 on a dedicated pcpu
	 */
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] There are %i virtual cores in Dom0.\n", dom0VCPU);
#endif
	list_for_each( iter_svc, &dom0->vcpu )
	{
		newPlanElem = xzalloc(struct asym_vcpuPlan);
		newPlanElem->pcpu = prv->master;
		newPlanElem->start_slice = 0;
		newPlanElem->end_slice = ASYM_INTERVAL_TS;
		svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);
		list_add_tail(&newPlanElem->plan_elem, &svc->info.plan);
#ifdef	ASYM_DEBUG
		printk("[SCHED_ASYM] plan for vCPU %i of Dom0\n", svc->vcpu->vcpu_id);
#endif
	}	
	/* 
	 * Assignment of the other vcpu
	 */

	DEBUGMSG("[SCHED_ASYM] Assigning virtual core of the other domains.\n");

	while(!list_empty(&exePlan)){
		currSlice = list_entry(exePlan.next, struct asym_exeSlice, slice_elem);
		for(int i = 0; i < amountPCPU; i++){
			if(currSlice->mapping[i] != -1){
				newPlanElem = xzalloc(struct asym_vcpuPlan);
				newPlanElem->pcpu = i;
				newPlanElem->start_slice = now;
				newPlanElem->end_slice = now + currSlice->timeslice;
				/* get corresponding vcpu, and insert the plan */
				svc = __fetch_vcpu_num(currSlice->mapping[i], &active_vcpu);
				if(svc != NULL){
					list_add_tail(&newPlanElem->plan_elem, &svc->info.plan);
				}
				else{
					printk("[SCHED_ASYM] Cannot find virtual core %i by number.\n", currSlice->mapping[i]);
				}
			}
		}
		now += currSlice->timeslice;
		list_del_init(exePlan.next);
	};

	__dump_plan();

	/* free allocated memory	*/
	for(int i = 0; i < CORE_AMOUNT; i++){
		if((i != prv->master) 
			&& (prv->cpuArray[i] != NULL)){
			spc = prv->cpuArray[i];
			xfree(spc->info.workloads);
		}
	}

	for(int i = 0; i < CORE_AMOUNT; i++){
		/* */
		__runq_check(i);
		/* */
		per_cpu(curr_slice, i) = 0;
	}

	set_timer(&prv->timer_gen_plan,
			NOW() + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS));

	spin_unlock_irqrestore(&prv->lock, flags);

}
/* Increase current slice number of each physical core	*/
static void
asym_tick(void *_cpu)
{
	unsigned int cpu = (unsigned long)_cpu;
	struct asym_pcpu *spc = ASYM_PCPU(cpu);
    
	set_timer(&spc->ticker, NOW() + MILLISECS(ASYM_TIMESLICE_MS) );

	/* spc->curr_slice++	*/
	get_cpu_var(curr_slice)++;
	put_cpu_var(curr_slice);           
}

/*
 * This function is in the critical path. It is designed to be simple and
 * fast for the common case.
 */
static struct task_slice
asym_schedule(
    const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
	const int cpu = smp_processor_id();
    struct list_head * const runq = RUNQ(cpu);
    struct asym_vcpu * const scurr = ASYM_VCPU(current);	/* can us "current" directly?	*/
    //struct asym_private *prv = ASYM_PRIV(ops);
    struct asym_vcpu *snext;
    struct task_slice ret;
	struct asym_vcpuPlan *plan;
	
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] asym_scheduler() is called by cpu %i (slice:%i).\n", cpu, get_cpu_var(curr_slice));
	printk("\tThe current vcpu is %i.\n", current->vcpu_id);
#endif

	SCHED_STAT_CRANK(schedule);

	/* First, compare the end_time of current running vcpu with the curr_slice
	 * If the end_time is larger, the next vcpu = the current one.
	 * Else, migrate the vcpu to the next pcpu according to the scheduling plan,
	 * and fetch the next vcpu from run queue.
	 */
	plan = __plan_elem(scurr);
	/* Tasklet work (which runs in idle VCPU context) overrides all else. */
    if ( tasklet_work_scheduled )
    {
		DEBUGMSG("[SCHED_ASYM] tasklet_work_scheduled = true\n");
        snext = ASYM_VCPU(idle_vcpu[cpu]);
		goto out;
    }
	else if( !(scurr->vcpu == idle_vcpu[cpu])
		&& vcpu_runnable(current)
		&& plan != NULL
		&& plan->end_slice >= per_cpu(curr_slice, cpu)){
			//if(scurr->vcpu->domain->domain_id != DOM0_ID){
				DEBUGMSG("[SCHED_ASYM] snext = scurr\n");
				snext = scurr;
				ret.task = snext->vcpu;
				ret.migrated = 0;
				goto out;
			//}
			//else{
#ifdef	ASYM_DEBUG
				//printk("[SCHED_ASYM] vCPU %i of Dom0\n", scurr->vcpu->vcpu_id);
#endif
				//__runq_migrate(cpu, scurr);
			//}
	}
	else{
		/* check where the next pcpu the current vcpu should go */
		if(scurr->vcpu == idle_vcpu[cpu]){			
			//DEBUGMSG("[SCHED_ASYM] the CPU was idling.\n");
		}		
		else{
			if(plan != NULL
				&& plan->end_slice < per_cpu(curr_slice, cpu)){
				/* remove the first plan element from current vCPU*/
				plan = __pop_elem(scurr);
			}
			if(plan == NULL){
				/* nowhere to go, put to the end of the queue	*/
				DEBUGMSG("[SCHED_ASYM] vcpu has no plan.\n");
			}		
			else if(plan->pcpu == cpu){
				DEBUGMSG("[SCHED_ASYM] vcpu should stay on the same pcpu.\n");				
			}
			else{
				/* migrate to target pcpu*/
				DEBUGMSG("[SCHED_ASYM] run queue migration.\n");								
			}
			__runq_migrate(cpu, scurr);
		}
	}
	
	/*
	* Select next runnable local VCPU (ie, top of local runq)
	*/
	snext = __runq_elem(runq->next);
	
	/* If its start_time is less or equal to curr_slice, start the execution
	 * else, idle for a time slice
	 */
	ret.task = idle_vcpu[cpu];
	if(unlikely(snext == NULL)){
		DEBUGMSG("[SCHED_ASYM] empty runqueue.\n");			
	}
	else{
		plan = __plan_elem(snext);
		if(plan != NULL
			&& vcpu_runnable(current)
			&& plan->start_slice <= get_cpu_var(curr_slice)){				
			ret.task = snext->vcpu;
			DEBUGMSG("[SCHED_ASYM] next time slice will be working.\n");

			if ( snext->vcpu->processor != cpu ){
				snext->vcpu->processor = cpu;
				ret.migrated = 1;
#ifdef	ASYM_DEBUG
	printk("[SCHED_ASYM] Should migrate vcpu %i from %i to %i.\n", snext->info.vcpuNum, snext->vcpu->processor, cpu);
#endif
			}
			else{
				ret.migrated = 0;
			}
		}
		else{
			DEBUGMSG("[SCHED_ASYM] next time slice will be idling.\n");				
		}	
	}

out:
	ret.time = MILLISECS(ASYM_TIMESLICE_MS);
	return ret;	 
}

static void
asym_dump_vcpu(struct asym_vcpu *svc)
{
	ASSERT(svc != NULL);
    /* idle vcpu */
	/*
    if( svc->sdom == NULL )
    {
        printk("\n");
        return;
    }*/

    printk("[SCHED_ASYM] (%i.%i) on_cpu=%i actual_cpu=%i\n",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->on_cpu,            
            svc->vcpu->processor);    
}

static void
asym_dump_pcpu(const struct scheduler *ops, int cpu)
{
	struct list_head *runq, *iter;
    struct asym_pcpu *spc;
    struct asym_vcpu *svc;
    int loop;
#define cpustr keyhandler_scratch

    spc = ASYM_PCPU(cpu);
    runq = &spc->runq;

	cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_core_mask, cpu));
    printk("[SCHED_ASYM] core=%s\n", cpustr);

	/* current VCPU */
    svc = ASYM_VCPU(curr_on_cpu(cpu));
    if ( svc )
    {
        printk("\trun: ");
        asym_dump_vcpu(svc);
    }
	/* VCPU the runqueue*/
    loop = 0;
    list_for_each( iter, runq )
    {
        svc = __runq_elem(iter);
        if ( svc )
        {
            printk("\t%3d: ", ++loop);
            asym_dump_vcpu(svc);
        }
    }
#undef cpustr
}

static void
asym_dump(const struct scheduler *ops)
{
	struct list_head *iter_sdom, *iter_svc;
    struct asym_private *prv = ASYM_PRIV(ops);
	struct asym_dom *sdom;
	struct asym_vcpu *svc;
    int loop;
    unsigned long flags;

    spin_lock_irqsave(&(prv->lock), flags);

	printk("Global RunQueue info:\n");

    printk("info:\n"
           "\tmaster = %u\n"
		   "\tlength of timeslice(msec) = %d\n"
		   "\t# of tslice in an interval = %d\n",
		   prv->master,
           ASYM_TIMESLICE_MS,
		   ASYM_INTERVAL_TS);

	//printk("active vcpus:\n");
	printk("Domain info:\n");
    loop = 0;
    list_for_each( iter_sdom, &prv->sdom )
    {
        sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		printk("\tdomain: %d\n", sdom->dom->domain_id);

        list_for_each( iter_svc, &sdom->vcpu )
        {
            svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);

            printk("\t%3d: ", ++loop);
            asym_dump_vcpu(svc);
        }
    }

    spin_unlock_irqrestore(&(prv->lock), flags);
}

static int
asym_init(struct scheduler *ops)
{
	/* allocate private data*/
	struct asym_private *prv;
	int i = 0;

    prv = xzalloc(struct asym_private);
    if ( prv == NULL )
        return -ENOMEM;

	ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);

	/* array of ptr to physical cores */
	prv->master = 0;	
	for( i = 0 ; i < CORE_AMOUNT; i++){
		prv->cpuArray[i] = NULL;
	}
		
	printk("[SCHED_ASYM] Scheduler initialization Successful.\n");
	
	return 0;
}

static void
asym_deinit(const struct scheduler *ops)
{
    struct asym_private *prv;

    prv = ASYM_PRIV(ops);
    if ( prv != NULL )
    {
        xfree(prv);
    }
}

static struct asym_private _asym_priv;

const struct scheduler sched_asym_def = {	
    .name           = "Asymmetric Aware Scheduler",
    .opt_name       = "asym",
    .sched_id       = XEN_SCHEDULER_ASYM,
    .sched_data     = &_asym_priv,

    .init_domain    = asym_dom_init,
    .destroy_domain = asym_dom_destroy,

    .insert_vcpu    = asym_vcpu_insert,
    .remove_vcpu    = asym_vcpu_remove,

    .sleep          = asym_vcpu_sleep,
    .wake           = asym_vcpu_wake,
    .yield          = asym_vcpu_yield,

    .adjust         = asym_dom_cntl,
    .adjust_global  = asym_sys_cntl,

    .pick_cpu       = asym_cpu_pick,
    .do_schedule    = asym_schedule,

    .dump_cpu_state = asym_dump_pcpu,
    .dump_settings  = asym_dump,
    .init           = asym_init,
    .deinit         = asym_deinit,
    .alloc_vdata    = asym_alloc_vdata,
    .free_vdata     = asym_free_vdata,
    .alloc_pdata    = asym_alloc_pdata,
    .free_pdata     = asym_free_pdata,
    .alloc_domdata  = asym_alloc_domdata,
    .free_domdata   = asym_free_domdata,    
};

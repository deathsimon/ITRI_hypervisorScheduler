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
#define CORE_AMOUNT				6	/* 0~3: A53, 4~5: A57	*/
#define AMOUNT_EFFI				4

#define	ASYM_TIMESLICE_MS			25	/* timeslice in ms	*/
#define ASYM_INTERVAL_TS			40	/* # of timeslices in an interval 	*/

#define	TYPE_PERFORMANCE			0
#define	TYPE_EFFICIENCY				1

#define	FREQ_PERFORMANCE			57600	/* KHz	*/
#define	FREQ_EFFICIENCY				19200
#define FREQ_MINIMUM				FREQ_EFFICIENCY/4

#define	DOM0_ID					0

#define	DOM0_MASTER_CORE			0
#define DOM0_TIMER_CORE				1

/*
 * Flags
 */
#define ASYM_FLAG_VCPU_YIELD	0x1	/* VCPU yielding */
#define ASYM_FLAG_VCPU_LOCK	0x2	/* VCPU lock */
#define ASYM_FLAG_VCPU_EARLY	0x4 /* execute the VPCU earlier since the current one is not runnable */

//#define	ASYM_DEBUG

//#define PASSIVE_MIGRATION

#ifdef	ASYM_DEBUG
#define DEBUGMSG(msg, arg...)	printk(msg, ##arg)
#else
#define DEBUGMSG(msg, arg...)
#endif

#define TICK_TIMER	/* use timer to count time slice */

/* From xen/common/vcpu_freq.c by Isaac	*/
#define NUM_DOMAIN	16
#define MAX_VCPU	16
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
#ifdef TICK_TIMER
DEFINE_PER_CPU(unsigned int, curr_slice);
#endif
struct asym_pcpu {
	struct list_head runq;
#ifdef TICK_TIMER
	struct timer ticker;
#else
	atomic_t curr_slice;      /* record current time slice in an interval     */
#endif
	/* for scheduling algorithm*/
        struct asym_pcpuinfo{
                unsigned int type;
                unsigned int provRes;			/* in MHz       */
				unsigned int resRemain;			/* in MHz       */
                unsigned int load;
                unsigned int* workloads;
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
	//unsigned int on_cpu;
	/* for scheduling algorithm*/
	struct asym_vcpuinfo{
		unsigned int vcpuNum;
		unsigned int requRes;		/* in MHz	*/
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
	struct domain *dom;		/* pointer to upper domain	*/
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
	unsigned int master_timer;
	unsigned int master;
	struct asym_pcpu* cpuArray[CORE_AMOUNT];
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
	struct asym_pcpu** cpuMapping;
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
#ifdef TICK_TIMER
static void asym_tick(void *_cpu);
#endif
static void asym_gen_plan(void *dummy);
static bool_t assignV2P(int* assignment, int level, struct asym_decSet *decSet);
static inline struct asym_vcpuPlan* __plan_elem(struct asym_vcpu* svc);
/*
 * Inline functions from credit-base scheduler
 */
static inline int
__vcpu_on_runq(struct asym_vcpu *svc)
{	
	return !list_empty(&svc->runq_elem);
}

static inline struct asym_vcpu *
__runq_elem(struct list_head *elem)
{	
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
	//BUG_ON( cpu != svc->vcpu->processor );

	/* DEBUGMSG("[SCHED_ASYM] insert vcore %i into the run queue on core %i \n",	
		svc->vcpu->vcpu_id, cpu); */
	
	planElem = __plan_elem(svc);
	if(planElem != NULL){		
		/*
		DEBUGMSG("[SCHED_ASYM] vcpu(%i,%i) has planElem (%i, %i).\n",
				svc->vcpu->domain->domain_id, svc->vcpu->vcpu_id, planElem->start_slice, planElem->end_slice);
		*/
		vcpu_start = planElem->start_slice;
		/* list_for_each(pos, head)  */
		list_for_each( iter, runq )
		{
			//struct asym_vcpu * iter_svc = __runq_elem(iter);
			//planElem = __plan_elem(iter_svc);
			planElem = __plan_elem(__runq_elem(iter));
			if(planElem != NULL){
				curr_start = planElem->start_slice;
				//DEBUGMSG("\tvcpu has plan (%i, %i).\n", planElem->start_slice, planElem->end_slice);
				/* insert according to the start_time of the vcpu	*/
				if ( vcpu_start < curr_start ){
					break;
				}
			}			
		}	
	}
	else{
		// DEBUGMSG("[SCHED_ASYM] vcpu(%i,%i) has no planElem, add to tail.\n", svc->vcpu->domain->domain_id, svc->vcpu->vcpu_id);		
		iter = RUNQ(cpu);
	}
	list_add_tail(&svc->runq_elem, iter);

	BUG_ON( !__vcpu_on_runq(svc) );
}

static inline void
__runq_remove(struct asym_vcpu *svc)
{
	BUG_ON( !__vcpu_on_runq(svc) );
	/*
	DEBUGMSG("[SCHED_ASYM] remove vcore %i from the run queue\n",
		svc->vcpu->vcpu_id);
	*/
	list_del_init(&svc->runq_elem);	
}
static inline void
__runq_tickle(unsigned int cpu, struct asym_vcpu *new)
{
	/* 
	 * Raise softirq to tell the cpu to re-schedule.
	 */	
	cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);   	
}
static inline void
__runq_migrate(unsigned int target, struct asym_vcpu *svc)
{
	if ( __vcpu_on_runq(svc) )
		__runq_remove(svc);
	
	/*
	DEBUGMSG("[SCHED_ASYM] migrate vcore (%i, %i) from %i to %i.\n", 
		svc->vcpu->domain->domain_id, svc->vcpu->vcpu_id, sorurce, target );	
	*/
#ifndef PASSIVE_MIGRATION
	svc->vcpu->processor = target;
#endif
	__runq_insert(target, svc);
	__runq_tickle(target, svc);
}
static inline void
__runq_check(unsigned int cpu)
{
	const struct list_head * const runq = RUNQ(cpu);
	struct list_head *iter, *iter_next;
	struct asym_vcpu * iter_svc;
	struct asym_vcpuPlan *planElem;

	list_for_each_safe( iter, iter_next, runq )
	{
		iter_svc = __runq_elem(iter);
		planElem = __plan_elem(iter_svc);
		if(planElem != NULL
			&& planElem->pcpu != cpu){
#ifdef PASSIVE_MIGRATION
			DEBUGMSG("[SCHED_ASYM] set migrating bit: runq_check.\n");
			set_bit(_VPF_migrating, &iter_svc->vcpu->pause_flags);
			__runq_tickle(cpu, iter_svc);
#else
			__runq_migrate(planElem->pcpu, iter_svc);
#endif
		}
	}
}
/* get the first plan element of a virtual core
 * return NULL if there is no plan element
 */
static inline struct asym_vcpuPlan* 
__plan_elem(struct asym_vcpu* svc)
{
	struct asym_vcpuPlan* vplan = NULL;

	while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));

	if(!list_empty(&svc->info.plan)){
		vplan = list_entry(svc->info.plan.next, struct asym_vcpuPlan, plan_elem);
	}
	
	clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);

	return vplan;
}
static inline struct asym_vcpuPlan* 
__pop_elem(struct asym_vcpu* svc)
{
	struct asym_vcpuPlan* vplan = NULL;
	struct asym_vcpuPlan* del_elem = NULL;

	BUG_ON(list_empty(&svc->info.plan));
	
	while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));

	if(!list_empty(&svc->info.plan)){
		del_elem = list_entry(svc->info.plan.next, struct asym_vcpuPlan, plan_elem);		
		list_del_init(svc->info.plan.next);
	}
	
	if(!list_empty(&svc->info.plan)){
		vplan = list_entry(svc->info.plan.next, struct asym_vcpuPlan, plan_elem);
	}

	clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);

	if(del_elem != NULL){
		xfree(del_elem);
	}

	return vplan;
}
static inline void
__clean_elem(struct asym_vcpu* svc)
{
	struct asym_vcpuPlan* del_elem;
	struct list_head * const plan_head = &svc->info.plan;
	struct list_head *iter, *iter_next;

	// DEBUGMSG("[SCHED_ASYM] Clean elements on vcpu %i.\n", svc->vcpu->vcpu_id);

	while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));

	list_for_each_safe(iter, iter_next, plan_head){
		del_elem = list_entry(iter, struct asym_vcpuPlan, plan_elem);
		list_del_init(iter);
		if(del_elem)
			xfree(del_elem);
	}
	
	//INIT_LIST_HEAD(&svc->info.plan);	

	clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);

	BUG_ON(!list_empty(&svc->info.plan));
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
	
	if(freq == 0){		
		freq = FREQ_MINIMUM;
	}
	/*
	else{		
		if(vcpu->domain->domain_id != DOM0_ID){
		DEBUGMSG("__fetch_vcore_freq() : vcpu (%i, %i) , freq = %i.\n",
			vcpu->domain->domain_id, vcpu->vcpu_id, freq);
		}		
	}
	*/
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
	unsigned long flags;

	if ( spc == NULL )
        return;

	DEBUGMSG("[SCHED_ASYM] free pdata of cpu %i.\n", cpu);

	spin_lock_irqsave(&prv->lock, flags);
	
	/* remove the pcpu from the pcpu array */
	prv->cpuArray[cpu] = NULL;
	
	/* clean runq */
	while(!list_empty(&spc->runq)){
		svc = list_entry(spc->runq.next, struct asym_vcpu, runq_elem);
		list_del_init(&svc->runq_elem);
	};
	/* remove timer */
#ifdef TICK_TIMER
	kill_timer(&spc->ticker);
#endif
	if (prv->master_timer == cpu){
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

	spin_lock_irqsave(&prv->lock, flags);

	/* Initialize/update system-wide config */
	
	/* set timer	*/
	if(cpu == prv->master_timer){
		init_timer(&prv->timer_gen_plan, asym_gen_plan, prv, cpu);
		set_timer(&prv->timer_gen_plan,
			NOW() + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS));
	}
#ifdef TICK_TIMER
	init_timer(&spc->ticker, asym_tick, (void *)(unsigned long)cpu, cpu);
	set_timer(&spc->ticker, NOW() + MILLISECS(ASYM_TIMESLICE_MS) );
	/* set per-CPU timeslice */
	this_cpu(curr_slice) = 0;
#else
	atomic_set(&spc->curr_slice, 0);
#endif

	/* set info	*/
	spc->info.type = __fetch_core_type(cpu);
	spc->info.provRes = 0;
	spc->info.resRemain = 0;
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

	return spc;
}

static int
asym_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
	/* Since the scheduling plan already decides which cpus a vcpu can run on,
	 * pick_cpu only return the current physical cpu of the vcpu.
	 */
	int cpu;
	struct asym_vcpuPlan* planElem;

	planElem = __plan_elem(ASYM_VCPU(vc));

	/* if no plan, put to the runqueue of pcpu first	*/
	(planElem != NULL)?(cpu = planElem->pcpu):(cpu = 2);//(vc->domain->domain_id + 1));
	//(planElem != NULL)?(cpu = planElem->pcpu):(cpu = vc->processor);

	DEBUGMSG("[SCHED_ASYM] cpu_pick() choose pcpu %i for vcpu %i of domain %i\n", cpu, vc->vcpu_id, vc->domain->domain_id);

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

	clear_bit(ASYM_FLAG_VCPU_YIELD, &svc->flags);
	clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);
	clear_bit(ASYM_FLAG_VCPU_EARLY, &svc->flags);

	/* If not idle vcpu, insert to domain */
	if(svc->sdom != NULL){		
		list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);
	}
		
	svc->info.vcpuNum = UINT_MAX;
	svc->info.requRes = UINT_MAX;
	INIT_LIST_HEAD(&svc->info.plan);
	INIT_LIST_HEAD(&svc->info.act_elem);
	
	SCHED_STAT_CRANK(vcpu_init);

	return svc;
}

static void
asym_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct asym_vcpu *svc = vc->sched_priv;

	//DEBUGMSG("[SCHED_ASYM] insert vcpu (%i, %i).\n", vc->domain->domain_id, vc->vcpu_id);
	
	if ( is_idle_vcpu(vc) )
        return;
		
	if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )		
		__runq_insert(vc->processor, svc);
}

static void
asym_free_vdata(const struct scheduler *ops, void *priv)
{
	struct asym_vcpu *svc = priv;

	if( svc == NULL)
		return;

    BUG_ON( !list_empty(&svc->runq_elem) );

	__clean_elem(svc);

    xfree(svc);
}

static void
asym_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{	
    struct asym_vcpu * const svc = ASYM_VCPU(vc);
    struct asym_dom * const sdom = svc->sdom;

	SCHED_STAT_CRANK(vcpu_destroy);

	if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);

	if ( !list_empty(&svc->info.act_elem) ){
		/* remove the vcpu from the active vcpu list	*/
		list_del_init(&svc->info.act_elem);
	}

	/* remove the plan elements */
	__clean_elem(svc);

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

	//DEBUGMSG("[SCHED_ASYM] vcpu_sleep() is called by vcpu %i of domain %i.\n", vc->vcpu_id, vc->domain->domain_id);

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

	DEBUGMSG("[SCHED_ASYM] vcpu_wake(): vcpu %i of domain %i on cpu %i,\n", vc->vcpu_id, vc->domain->domain_id, cpu);
	DEBUGMSG("\t called by vcpu %i of domain %i\n", current->vcpu_id, current->domain->domain_id);

	if ( likely(vcpu_runnable(vc)) ){
		//DEBUGMSG("vcpu_wake_runnable\n");
		SCHED_STAT_CRANK(vcpu_wake_runnable);
	}
	else{
		//DEBUGMSG("vcpu_wake_not_runnable\n");
		SCHED_STAT_CRANK(vcpu_wake_not_runnable);
	}

	/* Put the VCPU into the runq, and tirgger the re-scheduling on the cpu	*/	
	__runq_insert(cpu, svc);
	__runq_tickle(cpu, svc);
}

static void
asym_vcpu_yield(const struct scheduler *ops, struct vcpu *vc)
{
	struct asym_vcpu * const svc = ASYM_VCPU(vc);

	//DEBUGMSG("[SCHED_ASYM] vcpu_yield() is called by vcpu %i.\n", vc->vcpu_id);
	set_bit(ASYM_FLAG_VCPU_YIELD, &svc->flags);
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

	DEBUGMSG("asym_dom_cntl()\n");

	/*
	switch ( op->cmd )
	{
    case XEN_DOMCTL_SCHEDOP_getinfo:
		printk("\tXEN_DOMCTL_SCHEDOP_getinfo\t");
        break;
    case XEN_DOMCTL_SCHEDOP_putinfo:
        printk("\tXEN_DOMCTL_SCHEDOP_putinfo");
        break;
    }
	*/

	return 0;
}

static int
asym_sys_cntl(const struct scheduler *ops,
                        struct xen_sysctl_scheduler_op *sc)
{
	DEBUGMSG("asym_sys_cntl()\n");

	/*
	switch ( sc->cmd )
    {
    case XEN_SYSCTL_SCHEDOP_putinfo:
		printk("\tXEN_SYSCTL_SCHEDOP_putinfo\t");
		break;
	case XEN_SYSCTL_SCHEDOP_getinfo:
		printk("\tXEN_SYSCTL_SCHEDOP_getinfo\t");
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
    	  
	DEBUGMSG("[SCHED_ASYM] Allocating domdata for domain %i\n", dom->domain_id);

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
	/* TODO */
	/* For now, justify accroding to core type	*/	
	return spc->info.type;
}

static inline void
__dump_plan(struct asym_private *prv){

	struct asym_dom *sdom;
	struct asym_vcpu *svc;
	struct asym_vcpuPlan *planElem;
	struct list_head *iter_sdom, *iter_svc, *iter_plan;

	list_for_each( iter_sdom, &prv->sdom ){	
		sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		if(sdom->dom->domain_id != DOM0_ID){
		list_for_each( iter_svc, &sdom->vcpu )
		{			
			svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);
			
			while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));

			list_for_each( iter_plan, &svc->info.plan ){
				planElem = list_entry(iter_plan, struct asym_vcpuPlan, plan_elem);				
				DEBUGMSG("\tpcpu:%i;(%i,%i)", planElem->pcpu, planElem->start_slice, planElem->end_slice);
			};
			DEBUGMSG("\n");

			clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);
		}
		}
	}
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
	/*
	DEBUGMSG("[SCHED_ASYM] find target vcpu %i of domain %i\n",
		target_vcpu->vcpu->vcpu_id, target_vcpu->vcpu->domain->domain_id);
	*/
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

	if( tempMap == NULL)
		return 0;

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
assignV2P(int *assignment, int level, struct asym_decSet *decSet){
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
		assignment[level] = -1;
		if(assignV2P(assignment, level+1, decSet) == true){
			findAns = true;
		}
		else{
			for(int i = 0;i < decSet->numVCPU; i++){
				if((decSet->JobPicked[i] == false) && (decSet->cpuMapping[level]->info.workloads[i] != 0)){
					assignment[level] = i;
					decSet->MachPicked[level] = true;
					decSet->JobPicked[i] = true;
					if(assignV2P(assignment, level+1, decSet) == true){
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
static inline bool_t 
__is_master_core(struct asym_private *prv, int num){
	bool_t isMaster = false;
	if(num == prv->master || num == prv->master_timer)
		isMaster = true;
	
	return isMaster;
}

/* Greedy Assignment
 * Assign vcpu to the most "efficient" pcpu with load less than 100%
 */
static void
__phase_1(struct asym_private *prv, struct list_head *active_vcpu){
	struct asym_vcpu *svc;
	struct asym_pcpu *cand_cpu;
	struct list_head *iter_svc;
	int tempRes;
	uint16_t percentage;

	/* find pcpu for each vcpu */
	iter_svc = active_vcpu->next;
	while( iter_svc != active_vcpu )
	{
		svc = list_entry(iter_svc, struct asym_vcpu, info.act_elem);		
		cand_cpu = NULL;
		for(int i = 0; i < CORE_AMOUNT; i++){
			/* if current pcpu is for dom0 or is fully loaded, pass	*/
			if(__is_master_core(prv, i))
				continue;
			if(prv->cpuArray[i]->info.load == ASYM_INTERVAL_TS 
				|| prv->cpuArray[i]->info.provRes < svc->info.requRes)
				continue;
			if( cand_cpu == NULL
				|| (__pcpu_effi(cand_cpu) < __pcpu_effi(prv->cpuArray[i]))){
					cand_cpu = prv->cpuArray[i];
			}
		}
		if(cand_cpu == NULL){
			printk("[SCHED_ASYM] Cannot find a propoer physical core for the virtual core\n");
			break;
		}
		//DEBUGMSG("[SCHED_ASYM] vcpu (%i, %i) found a candidate cpu.\n", svc->vcpu->domain->domain_id, svc->vcpu->vcpu_id);
		
		/* assign vcpu to pcpu	*/				
		percentage = 0;
		if(cand_cpu->info.resRemain >= svc->info.requRes){			
			tempRes = svc->info.requRes*100;
			while(tempRes > 0){
				tempRes -= cand_cpu->info.provRes;
				percentage++;
			};			
			cand_cpu->info.workloads[svc->info.vcpuNum] = (unsigned int)(percentage*ASYM_INTERVAL_TS/100);
			cand_cpu->info.load += (unsigned int)(percentage*ASYM_INTERVAL_TS/100);
			//DEBUGMSG("[SCHED_ASYM] percentage = %i, load = %i\n", percentage, cand_cpu->info.load);
			cand_cpu->info.resRemain -= svc->info.requRes;
			svc->info.requRes = 0;
			iter_svc = iter_svc->next;
		}
		else{
			//DEBUGMSG("[SCHED_ASYM] Candidate cpu full! Find another one.\n");
			cand_cpu->info.workloads[svc->info.vcpuNum] = ASYM_INTERVAL_TS - cand_cpu->info.load;
			svc->info.requRes -= cand_cpu->info.resRemain;
			cand_cpu->info.resRemain = 0;
			cand_cpu->info.load = ASYM_INTERVAL_TS;
		}
	};

}
/* Phase 2
 * Open-shop scheduling
 */
static void
__phase_2(struct asym_private *prv, struct list_head* exeSlice, unsigned int amountPCPU, unsigned int amountVCPU){
	
	int LowerBound;
	int *assignment;
	unsigned int *vcoreLoad;
	unsigned int counter;
	
	struct asym_decSet decSet;
	struct asym_exeSlice *new_slice;

	if(amountVCPU == 0){
		/* Normally this should never happens, just in case of something wrong. */
		DEBUGMSG("[SCHED_ASYM] No virtual core from DomU!\n");
		return;
	}

	vcoreLoad = xzalloc_array(unsigned int, amountVCPU);
	assignment = xzalloc_array(int, amountPCPU);

	/* descrete set	*/
	decSet.MachTight = (bool_t*)xzalloc_array(bool_t, amountPCPU);
	decSet.JobTight = (bool_t*)xzalloc_array(bool_t, amountVCPU);
	decSet.MachPicked = (bool_t*)xzalloc_array(bool_t, amountPCPU);
	decSet.JobPicked = (bool_t*)xzalloc_array(bool_t, amountVCPU);
	decSet.cpuMapping = (struct asym_pcpu**)xzalloc_array(struct asym_pcpu*, amountPCPU);

	decSet.numPCPU = amountPCPU;
	decSet.numVCPU = amountVCPU;	

	BUG_ON(vcoreLoad == NULL);

	/* Initialization	*/	
	for(int i = 0;i < amountVCPU;i++){
		vcoreLoad[i] = 0;
	}
	counter = 0;	
	for(int i = 0;i < CORE_AMOUNT; i++){
		if(__is_master_core(prv, i))
			continue;
		decSet.cpuMapping[counter] = prv->cpuArray[i];
		counter++;
		for(int j = 0;j < amountVCPU; j++){
			if(prv->cpuArray[i] != NULL){				
				vcoreLoad[j] += prv->cpuArray[i]->info.workloads[j];
			}
		}
	}	
	
	//DEBUGMSG("[SCHED_ASYM] start OSSP.\n");
	while(1){
		/* find lower bound	*/
		LowerBound = 0;
		for(int i = 0;i < amountPCPU;i++){
			if(decSet.cpuMapping[i]->info.load > LowerBound){
				LowerBound = decSet.cpuMapping[i]->info.load;
			}
		}
		for(int i = 0;i < amountVCPU;i++){
			if(vcoreLoad[i] > LowerBound){
				LowerBound = vcoreLoad[i];
			}
		}
		if(LowerBound == 0){
			/* no load, we are done in phase 2 */
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
			if(decSet.cpuMapping[i]->info.load == LowerBound){			
				decSet.MachTight[i] = true;
			}
			else{
				decSet.MachTight[i] = false;				
				if(decSet.delta > (LowerBound - decSet.cpuMapping[i]->info.load))
					decSet.delta = LowerBound - decSet.cpuMapping[i]->info.load;				
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
			if((assignment[i] != -1) && (decSet.cpuMapping[i]->info.workloads[assignment[i]] < decSet.delta)){			
				decSet.delta = decSet.cpuMapping[i]->info.workloads[assignment[i]];				
			}
		}
		// reduct workload		
		for(int i = 0;i < amountPCPU; i++){			
			if(assignment[i] != -1){				
				decSet.cpuMapping[i]->info.workloads[assignment[i]] -= decSet.delta;
				decSet.cpuMapping[i]->info.load -= decSet.delta;
				vcoreLoad[assignment[i]] -= decSet.delta;
			}
		}

		// create an execution slice
		new_slice = xzalloc(struct asym_exeSlice);
		new_slice->timeslice = decSet.delta;
		new_slice->mapping = xzalloc_array(int, amountPCPU);
		/*DEBUGMSG("[SCHED_ASYM] Add a new slice into the list. \n"
			"\t:("); */
		for(int i = 0;i < amountPCPU; i++){
			new_slice->mapping[i] = assignment[i];
			//DEBUGMSG("%i ", new_slice->mapping[i]);
		}
		//DEBUGMSG("), #:%i\n", new_slice->timeslice);
		/* add to the list of execution slice
		 * Assume that there will be no duplication.
		 */		
		list_add_tail(&new_slice->slice_elem, exeSlice);
	};
	//DEBUGMSG("[SCHED_ASYM] finish OSSP.\n");

	/* free allocated memory in phase 2	*/
	xfree(decSet.cpuMapping);
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

	if(amountVCPU == 0){
		/* Normally this should never happens, just in case of something wrong. */
		DEBUGMSG("[SCHED_ASYM] No virtual core from DomU!\n");
		return;
	}

	roadMap = xzalloc_array(int, amountVCPU);
	for(int i = 0; i < amountVCPU; i++){
		roadMap[i] = -1;
	}

	while(!list_empty(exeSlice)){
		iter_slice = exeSlice->next;
		candSlice = list_entry(iter_slice, struct asym_exeSlice, slice_elem);	
		minIncrease = __computeInc(candSlice, roadMap, amountPCPU);

		/*
		DEBUGMSG("[SCHED_ASYM] Choosing a candidate."
			"minInc = %i.\n",minIncrease);
		*/

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
		
		// DEBUGMSG("[SCHED_ASYM] Candidate found!\n");

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

	};
	// DEBUGMSG("[SCHED_ASYM] #Switching = %i.\n", numSwitching);
	xfree(roadMap);
}

static void
asym_gen_plan(void *dummy){
	/* declarations: general */

	struct asym_private *prv = dummy;
	struct asym_dom *sdom, *dom0 = NULL;
	struct asym_vcpu *svc;
	struct asym_pcpu *spc;
	unsigned int *cpuMapping = NULL;
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
	struct asym_vcpuPlan *lastPlanElem;	
	unsigned int now;

	s_time_t now_time = NOW();

	spin_lock_irqsave(&prv->lock, flags);

	INIT_LIST_HEAD(&active_vcpu);
	INIT_LIST_HEAD(&exeSlice);
	INIT_LIST_HEAD(&exePlan);

	/* First, fetch the frequencies of the vCPUs,
	 * and compute the total resource required
	 */		
	list_for_each( iter_sdom, &prv->sdom ){
		/* list_entry(ptr,type,member) */
		sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		/* Record Dom0 */
		if(sdom->dom->domain_id == DOM0_ID){
			dom0 = sdom;
		}		
		list_for_each( iter_svc, &sdom->vcpu )
		{			
			svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);
			/* If Dom0, count the number of vcpu;
			 * Else, accumulate total requirement.
			 */
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

	/* If no vcpu from DomU */
	if(amountVCPU == 0)
		goto assign_vcpu;

	/* Then fetch the frequencies of the pCPUs,
	 * and compute the total resource provided.
	 * Also allocate the memory of vCPUs on each pCPU
	 */
	cpuMapping = (unsigned int*)xzalloc_array(unsigned int, CORE_AMOUNT);
	for(int i = 0; i < CORE_AMOUNT; i++){
		if(!__is_master_core(prv, i)
			&& (prv->cpuArray[i] != NULL)){
			spc = prv->cpuArray[i];
			spc->info.load = 0;
			/* accumulate total provision	*/
			prov = __fetch_pcore_frequ(i);
			spc->info.provRes = prov;
			spc->info.resRemain = spc->info.provRes;
			total_prov += prov;
			spc->info.workloads = xzalloc_array(unsigned int, amountVCPU);
			if ( spc->info.workloads == NULL ){
				printk("[SCHED_ASYM] Allocate info.workloads fails.\n");
			}
			cpuMapping[amountPCPU] = i;
			amountPCPU++;
		}
	}	

	/* Phase 1
	 * compute scaling factor
	 */
	scale = 1;
	while(total_prov < total_requ){
		total_requ -= total_prov;
		scale++;
	};
	//DEBUGMSG("[SCHED_ASYM] req = %i, prov = %i, scale = %i\n", total_requ, total_prov, scale);
	list_for_each_entry(svc, &active_vcpu, info.act_elem){
		svc->info.requRes = (svc->info.requRes / scale);		
	}	
	__phase_1(prv, &active_vcpu);

	/* Phase 2
	 * Open-shop scheduling
	 */	
	__phase_2(prv, &exeSlice, amountPCPU, amountVCPU);
	
	/* Phase 3
	 * 
	 */
	__phase_3(&exeSlice, &exePlan, amountPCPU, amountVCPU);	
	
	/* finalize	
	 * assign the whole plan to each virtual core
	 */	
assign_vcpu:
	/* clean up the plan_element of each vritual core	*/
	list_for_each( iter_sdom, &prv->sdom ){
		/* list_entry(ptr,type,member) */
		sdom = list_entry(iter_sdom, struct asym_dom, sdom_elem);
		list_for_each( iter_svc, &sdom->vcpu )
		{
			svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);
			__clean_elem(svc);
		}
	}	

	/* 
	 * Assignment of vcpu in dom0 on a dedicated pcpu
	 */	
	list_for_each( iter_svc, &dom0->vcpu )
	{
		svc = list_entry(iter_svc, struct asym_vcpu, sdom_elem);

		newPlanElem = xzalloc(struct asym_vcpuPlan);		
		((svc->vcpu->vcpu_id + 1 )% 2)?(newPlanElem->pcpu = prv->master):(newPlanElem->pcpu = prv->master_timer);
		newPlanElem->start_slice = 0;
		newPlanElem->end_slice = ASYM_INTERVAL_TS;

		while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));
		list_add_tail(&newPlanElem->plan_elem, &svc->info.plan);
		clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);		
	}
	/* 
	 * Assignment of the other vcpu
	 */	
	now = 0;	
	while(!list_empty(&exePlan)){				
		currSlice = list_entry(exePlan.next, struct asym_exeSlice, slice_elem);
		for(int i = 0; i < amountPCPU; i++){
			if(currSlice->mapping[i] != -1){
				/* get corresponding vcpu */
				svc = __fetch_vcpu_num(currSlice->mapping[i], &active_vcpu);				
				if(svc == NULL){
					printk("[SCHED_ASYM] Cannot find virtual core %i by number.\n", currSlice->mapping[i]);
					continue;
				}
				lastPlanElem = list_entry(svc->info.plan.prev, struct asym_vcpuPlan, plan_elem);
				if(lastPlanElem->pcpu == cpuMapping[i]
					&& lastPlanElem->end_slice == now){
					lastPlanElem->end_slice = now + currSlice->timeslice;
				}
				else{
					newPlanElem = xzalloc(struct asym_vcpuPlan);
					newPlanElem->pcpu = cpuMapping[i];
					newPlanElem->start_slice = now;
					newPlanElem->end_slice = now + currSlice->timeslice;
					/* insert the plan */				
					
					while(test_and_set_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags));
					list_add_tail(&newPlanElem->plan_elem, &svc->info.plan);
					clear_bit(ASYM_FLAG_VCPU_LOCK, &svc->flags);									
				}
			}
		}
		now += currSlice->timeslice;
		list_del_init(exePlan.next);
	};	

	/*
	 * Set the processor of each vcpu from DomU
	 */	
	/*
	list_for_each_entry(svc, &active_vcpu, info.act_elem){
		newPlanElem = __plan_elem(svc);
		if( newPlanElem != NULL
			&& newPlanElem->pcpu != svc->vcpu->processor ){
				DEBUGMSG("runq migrate: should not happen for now.\n");
			__runq_migrate(svc->vcpu->processor, svc);
		}
		svc->vcpu->processor = __plan_elem(svc)->pcpu;
	}
	*/
	

#ifdef ASYM_DEBUG
	//__dump_plan(prv);
#endif

	/* free allocated memory	*/
	for(int i = 0; i < CORE_AMOUNT; i++){
		if(!__is_master_core(prv, i)
			&& (prv->cpuArray[i] != NULL)){
			spc = prv->cpuArray[i];
			if(spc->info.workloads){
				xfree(spc->info.workloads);
			}
		}
	}
	if(cpuMapping != NULL)
		xfree(cpuMapping);

	for(int i = 0; i < CORE_AMOUNT; i++){
		/* */
		__runq_check(i);
		/* */
#ifdef TICK_TIMER
		per_cpu(curr_slice, i) = 0;
#else
		if(prv->cpuArray[i] != NULL){
			atomic_set(&prv->cpuArray[i]->curr_slice, 0);			
		}
#endif
	}

	BUG_ON((now_time + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS)) < NOW());

	set_timer(&prv->timer_gen_plan,
			now_time + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS));

	spin_unlock_irqrestore(&prv->lock, flags);

}
/* Increase current slice number of each physical core	*/
#ifdef TICK_TIMER
static void
asym_tick(void *_cpu)
{
	unsigned int cpu = (unsigned long)_cpu;
	struct asym_pcpu *spc = ASYM_PCPU(cpu);
    
	set_timer(&spc->ticker, NOW() + MILLISECS(ASYM_TIMESLICE_MS) );

	this_cpu(curr_slice)++;
}
#endif

static bool
__in_range(struct asym_vcpuPlan *plan, unsigned int slice){
	return (plan->start_slice <= slice
			&& plan->end_slice > slice);
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
	struct asym_vcpu * const scurr = ASYM_VCPU(current);
	//struct asym_private *prv = ASYM_PRIV(ops);
	struct asym_vcpu *snext, *svc;
	struct task_slice ret;
	struct asym_vcpuPlan *plan;
	struct list_head *iter;
	unsigned int slice;

#ifdef TICK_TIMER
	slice = this_cpu(curr_slice);
#else	
	slice = atomic_read(&prv->cpuArray[cpu]->curr_slice);	
#endif
	
	//DEBUGMSG("[SCHED_ASYM] asym_scheduler()\t cpu %i (slice:%i) was running vcpu (%i, %i).\n", cpu, slice, current->domain->domain_id, current->vcpu_id);
	
	SCHED_STAT_CRANK(schedule);

	clear_bit(ASYM_FLAG_VCPU_YIELD, &scurr->flags);
	ret.time = MILLISECS(ASYM_TIMESLICE_MS);
	ret.migrated = 0;
			
	/* First, compare the end_time of current running vcpu with the curr_slice
	 * If the end_time is larger, the next vcpu = the current one.
	 * Else, migrate the vcpu to the next pcpu according to the scheduling plan,
	 * and fetch the next vcpu from run queue.
	 */
	/* Tasklet work (which runs in idle VCPU context) overrides all else. */
	if ( tasklet_work_scheduled )
	{		
		printk("[SCHED_ASYM] tasklet_work_scheduled = true\n");
		clear_bit(ASYM_FLAG_VCPU_EARLY, &scurr->flags);
		snext = ASYM_VCPU(idle_vcpu[cpu]);		
		goto out;
	}

	/* check if there is any vcpu needs to be miragted first. 
	 * If so, grab it.
	 */
#ifdef PASSIVE_MIGRATION
	if(!test_bit(_VPF_migrating, &current->pause_flags)){
		list_for_each( iter, runq ){
			svc = __runq_elem(iter);
			if(test_bit(_VPF_migrating, &svc->vcpu->pause_flags)){
				snext = svc;
				ret.time = MILLISECS(1);
				goto out;
			}
		}
	}
#endif
	
	if(test_and_clear_bit(ASYM_FLAG_VCPU_EARLY, &scurr->flags)){
		//DEBUGMSG("[SCHED_ASYM] goto select_next.\n");
		goto select_next;
	}

	/* if scurr is idle, do nothing and fetch the next vcpu directly. */
	if( !is_idle_vcpu(current) ){		
		plan = __plan_elem(scurr);
		/* if runnable and can run on current time slice
		 * snext = scurr;
		 */
		if( vcpu_runnable(current)
			&& plan != NULL
			&& __in_range(plan, slice)){
			snext = scurr;
			goto out;
		}
		else if( !vcpu_runnable(current) 
			&& plan != NULL
			&& __in_range(plan, slice)){

			/* if scurr is not runnable, pick the next runnable vcpu in the runqueue.
			 * Execute the runnable vcpu for half of the time slice.
			 */						
			snext = NULL;
			list_for_each( iter, runq ){
				svc = __runq_elem(iter);
				if(vcpu_runnable(svc->vcpu)){
					snext = svc;
					break;
				}
			}
			if(snext != NULL){
				set_bit(ASYM_FLAG_VCPU_EARLY, &snext->flags);
				ret.time /= 2;
			}
			else{			
			/*
			 * If no runnable vcpu available, should idle.
			 * But idle will cause long booting time.
			 * snext = scurr anyway...
			 */
				//snext = ASYM_VCPU(idle_vcpu[cpu]);
				snext = scurr;
			}
			goto out;
		}
		else{			
			/* vcpu either with no plan or not in executable range
			 * deal with scurr 
			 */
			if(plan != NULL
				&& plan->end_slice <= slice){
				/* remove the first plan element from current vCPU*/
				plan = __pop_elem(scurr);
			}
			
			if(plan != NULL
				&& plan->pcpu != cpu){
				/* migrate to target runq*/
				DEBUGMSG("[SCHED_ASYM] set migrating bit: do_schedule() .\n");
#ifdef PASSIVE_MIGRATION
				set_bit(_VPF_migrating, &scurr->vcpu->pause_flags);
#else
				__runq_migrate(plan->pcpu, scurr);
#endif
			}					
		}	
	}
	
	/*
	* Select next runnable local VCPU (ie, top of local runq)
	*/
select_next:	
	snext = NULL;
	list_for_each( iter, runq ){		
		svc = __runq_elem(iter);
		plan = __plan_elem(svc);
		if(plan != NULL
			&& __in_range(plan, slice)){
				snext = svc;
				break;
		}		
	}
	
	/* If its start_time is less or equal to curr_slice, start the execution
	 * else, idle for a time slice
	 */
	if( unlikely(snext == NULL) ){
		/* empty runqueue or no vcpu to execute */
		snext = ASYM_VCPU(idle_vcpu[cpu]);
	}/*
	else if( !vcpu_runnable(snext->vcpu) ){
		snext = ASYM_VCPU(idle_vcpu[cpu]);
		ret.time = MILLISECS(2);
	}*/
	else{
		/* migrate if the next vcpu is not on this pcpu. */
		if ( snext->vcpu->processor != cpu ){
			printk("[SCHED_ASYM] Should migrate vcpu %i from %i to %i.\n", 
				snext->info.vcpuNum, snext->vcpu->processor, cpu);
#ifndef PASSIVE_MIGRATION
			snext->vcpu->processor = cpu;
			//set_bit(_VPF_migrating, &scurr->vcpu->pause_flags);
			ret.migrated = 1;
#endif
		}		
	}

out:
	ret.task = snext->vcpu;
#ifndef TICK_TIMER
	atomic_inc(&prv->cpuArray[cpu]->curr_slice);
#endif
	return ret;	 
}
static void
asym_migrate(const struct scheduler *ops, struct vcpu *vc, unsigned int new_cpu){

#ifdef PASSIVE_MIGRATION
	struct asym_vcpu * const svc = ASYM_VCPU(vc);

	// migrate runqueue here
	__runq_migrate(new_cpu, svc);
	clear_bit(_VPF_migrating, &vc->pause_flags);
#endif
	vc->processor = new_cpu;

	DEBUGMSG("[SCHED_ASYM] asym_migrate():vcpu (%i, %i) from cpu %i to %i.\n", vc->domain->domain_id, vc->vcpu_id, vc->processor, new_cpu);
}

static void
asym_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
	if ( unlikely(test_bit(_VPF_migrating, &vc->pause_flags)) )
		DEBUGMSG("[SCHED_ASYM] asym_context_saved():vcpu (%i, %i) should trigger vcpu_migrate.\n", vc->domain->domain_id, vc->vcpu_id);
}

static void
asym_dump_vcpu(struct asym_vcpu *svc)
{
	ASSERT(svc != NULL);
	/* idle vcpu */
	if( svc->sdom == NULL )
	{
		printk("\n");
		return;
	}

   	printk("[SCHED_ASYM] (%i.%i) on_cpu=%i\n",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,            
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
		"\tmaster_timer = %u\n"
		"\tlength of timeslice(msec) = %d\n"
		"\t# of tslice in an interval = %d\n",
		prv->master,
		prv->master_timer,
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
	prv->master = DOM0_MASTER_CORE;
	prv->master_timer = DOM0_TIMER_CORE;	
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
	
	.context_saved  = asym_context_saved,
	.migrate		= asym_migrate,
};

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

#define	ASYM_TIMESLICE_MS			10		/* timeslice = 10 ms	*/
#define ASYM_INTERVAL_TS			100		/* interval = 100 timeslice	*/

#define	TYPE_PERFORMANCE			0
#define	TYPE_EFFICIENCY				1

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
	unsigned int type;	
    //unsigned int curr_slice;	/* record current time slice in an interval	*/
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
	/* excution period on a pcpu	*/
	unsigned int target_cpu;
	unsigned int start_slice;
	unsigned int end_slice;
		
	unsigned flags;
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

static void asym_tick(void *_cpu);
static void asym_gen_plan(void *dummy);

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

    BUG_ON( __vcpu_on_runq(svc) );
    BUG_ON( cpu != svc->vcpu->processor );

	svc->target_cpu = cpu;
	/* list_for_each(pos, head)  */
    list_for_each( iter, runq )
    {
        const struct asym_vcpu * const iter_svc = __runq_elem(iter);
		/* insert according to the start_time of the vcpu	*/
        if ( svc->start_time < iter_svc->start_time )
            break;
    }	
	list_add_tail(&svc->runq_elem, iter);
}

static inline void
__runq_remove(struct asym_vcpu *svc)
{
    BUG_ON( !__vcpu_on_runq(svc) );
    list_del_init(&svc->runq_elem);
}

static inline void
__runq_tickle(unsigned int cpu, struct asym_vcpu *new)
{
    /* After inserting a vcpu to the runqueue of a pcpu, 
	 * raise softirq to tell the cpu to re-schedule.
	 */
	cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);   	
}
/*	get the type of a pcpu	*/
static inline int
__fetch_core_type(unsigned int cpu)
{
	unsigned int type = UINT_MAX;
	/* for JUNO board	*/
	(cpu >= AMOUNT_EFFI)?(type = TYPE_PERFORMANCE):(type = TYPE_EFFICIENCY);
	/* for general cases, 
	 * Victor: To get the physical CPU type info, you can access it in Xen by get part_number field from MIDR register with current_cpu_data.midr.part_number
	 */	
	return type;
}

static void
asym_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
	struct asym_private *prv = ASYM_PRIV(ops);
    struct asym_pcpu *spc = pcpu;
    unsigned long flags;

	if ( spc == NULL )
        return;
	
	spin_lock_irqsave(&prv->lock, flags);
	
	/* remove the pcpu from the pcpu array */
	prv->cpuArray[cpu] = NULL;
	
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

	spin_lock_irqsave(&prv->lock, flags);

    /* Initialize/update system-wide config */
	
	/* set timer	*/
	if(cpu == prv->master){
		init_timer(&prv->timer_gen_plan, asym_gen_plan, prv, cpu);
		set_timer(&prv->timer_gen_plan,
			NOW() + MILLISECS(ASYM_INTERVAL_TS*ASYM_TIMESLICE_MS));		
	}
	init_timer(&spc->ticker, asym_tick, (void *)(unsigned long)cpu, cpu);
    set_timer(&spc->ticker, NOW() + MICROSECS(ASYM_TIMESLICE_MS) );		
	/* set type	*/
	spc->type = __fetch_core_type(cpu);	
	/* set per-CPU timeslice	*/
	get_cpu_var(sockets_in_use) = 0;
	put_cpu_var(sockets_in_use);
	
	prv->cpuArray[cpu] = spc;
	/* Initialize runqueue	*/
    INIT_LIST_HEAD(&spc->runq);
    
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

	cpu = vc->processor;

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
	/*
	svc->target_cpu = UINT_MIN;
    svc->start_slice = UINT_MIN;
	svc->end_slice = UINT_MIN;
	*/
	SCHED_VCPU_STATS_RESET(svc);
	SCHED_STAT_CRANK(vcpu_init);
    
    return svc;
}

static void
asym_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct asym_vcpu *svc = vc->sched_priv;

    if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )
        __runq_insert(vc->processor, svc);

	/* if this vcpu belong does not belong to any dom, somethings wrong */
	if(!list_empty(&svc->sdom_elem))
		list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);	
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
    unsigned long flags;

	SCHED_STAT_CRANK(vcpu_destroy);

	if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);

    if ( !list_empty(&svc->sdom_elem) ){
		/* remove the vcpu from domain	*/
		list_del_init(&svc->sdom_elem);
	}	    

    BUG_ON( sdom == NULL );
    BUG_ON( !list_empty(&svc->runq_elem) );
}

static void
asym_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
	struct asym_vcpu * const svc = ASYM_VCPU(vc);

    SCHED_STAT_CRANK(vcpu_sleep);

    BUG_ON( is_idle_vcpu(vc) );

	if ( curr_on_cpu(vc->processor) == vc )
		/* if running on pcpu	*/
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);	
    else if ( __vcpu_on_runq(svc) )
		/* only on runqueue	*/
        __runq_remove(svc);
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
}

static int
asym_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
	struct xen_domctl_scheduler_op *op)
{
	struct asym_dom * const sdom = ASYM_DOM(d);
    struct asym_vcpu *svc;
    struct list_head *iter;

	printk("Here we are in asym_dom_cntl()\n"
		   "\tcmd:%d\n",op->cmd););

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
	unsigned long flags;
    struct asym_dom *sdom;
    struct asym_private * prv = ASYM_PRIV(ops);

    sdom = xzalloc(struct asym_dom);
    if ( sdom == NULL )
        return NULL;
    
    INIT_LIST_HEAD(&sdom->vcpu);
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;

	/* spinlock here to insert the dom */
    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&sdom->sdom_elem, &(prv->sdom));
    spin_unlock_irqrestore(&prv->lock, flags);

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

/* Increase current slice number of each physical core	*/
static void
asym_tick(void *_cpu)
{
	unsigned int cpu = (unsigned long)_cpu;
	struct asym_pcpu *spc = ASYM_PCPU(cpu);
    
	/* spc->curr_slice++	*/
	get_cpu_var(curr_slice)++;
	put_cpu_var(curr_slice);
        
    set_timer(&spc->ticker, NOW() + MICROSECS(ASYM_TIMESLICE_MS) );
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
    struct asym_private *prv = ASYM_PRIV(ops);
    struct asym_vcpu *snext;
    struct task_slice ret;
	
	SCHED_STAT_CRANK(schedule);

	/* TODO */
    /* First check the current cpu is master(for dom0 only) or the others.	*/

	/* If dom0, apply a round-robin fashion to execute the vcpus from dom0*/

	/* else, compare the end_time of current running vcpu with the curr_slice
	 *
	 * If the end_time is larger, the next vcpu = the current one.
	 * Else, migrate the vcpu to the next pcpu according to the scheduling plan,
	 * and fetch the next vcpu from run queue.
	 
	 if(last_cpu->target_cpu != this_cpu){query and migrate}

	 */
	/* If its start_time is less or equal to curr_slice, start the execution
	 * else, idle for a time slice
	 */
	if(snext->vcpu->start_time <= curr_slice){
		ret.time = ASYM_TIMESLICE_MS; 
		ret.task = snext->vcpu;
	else{
		ret.time = -1; 
		ret.task = ;
	}	
	 
	return ret;	 
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

    printk("[%i.%i] target_cpu=%i timeslice=(%i,%i) curr_cpu=%i\n",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->target_cpu,
            svc->start_slice,
			svc->end_slice,
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
    printk("core=%s\n", cpustr);

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
	/* TODO */
}

static int
asym_init(struct scheduler *ops)
{
	/* allocate private data*/
	struct asym_private *prv;

    prv = xzalloc(struct asym_private);
    if ( prv == NULL )
        return -ENOMEM;

	ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);

	/* array of ptr to physical cores */
	prv->master = 0;
	int i = 0;
	for( i = 0 ; i < CORE_AMOUNT; i++){
		cpuArray[i] = NULL;
	}
		
	printk("Scheduler initialization Successful.\n");
	
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

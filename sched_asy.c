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
#define	ASYM_TIMESLICE_MS			10		/* timeslice = 10 ms	*/
#define ASYM_INTERVAL_TS			100		/* interval = 100 timeslice	*/

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
struct asym_pcpu {
	struct list_head runq;
	
    unsigned int curr_slice;	/* record current time slice in an interval	*/
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
	unsigned int start_slice;
	unsigned int end_slice;
		
	unsigned flags;
};

/*
 * Domain
 */
struct asym_dom {
    struct list_head vcpu;		/* link its VCPUs	*/
    struct list_head sdom_elem;	/* link list on rt_priv	*/
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
    struct timer  timer_gen_plan;
    unsigned int master;
    	
    cpumask_var_t cpus;       
};

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

    BUG_ON( __vcpu_on_runq(svc) );
    BUG_ON( cpu != svc->vcpu->processor );

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
__runq_tickle(unsigned int cpu, struct csched_vcpu *new)
{
    /* TODO
     * After inserting a vcpu to the runqueue of a pcpu, 
	 * do something here.
     */   	
}

/* TODOs */
static void
asym_free_pdata(const struct scheduler *ops, void *pcpu, int cpu){}

static void *
asym_alloc_pdata(const struct scheduler *ops, int cpu){}

static int
asym_cpu_pick(const struct scheduler *ops, struct vcpu *vc){}

static void *
asym_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd){}

static void
asym_vcpu_insert(const struct scheduler *ops, struct vcpu *vc){}

static void
asym_free_vdata(const struct scheduler *ops, void *priv){}

static void
asym_vcpu_remove(const struct scheduler *ops, struct vcpu *vc){}

static void
asym_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc){}

static void
asym_vcpu_wake(const struct scheduler *ops, struct vcpu *vc){}

static void
asym_vcpu_yield(const struct scheduler *ops, struct vcpu *vc){}

static int
asym_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
	struct xen_domctl_scheduler_op *op){}

static int
asym_sys_cntl(const struct scheduler *ops,
                        struct xen_sysctl_scheduler_op *sc){}

static void *
asym_alloc_domdata(const struct scheduler *ops, struct domain *dom){}

static int
asym_dom_init(const struct scheduler *ops, struct domain *dom){}

static void
asym_free_domdata(const struct scheduler *ops, void *data){}

static void
asym_dom_destroy(const struct scheduler *ops, struct domain *dom){}

/*
 * This function is in the critical path. It is designed to be simple and
 * fast for the common case.
 */
static struct task_slice
asym_schedule(
    const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    /* First check the current cpu is master(for dom0 only) or the others.	*/

	/* If dom0, apply a round-robin fashion to execute the vcpus from dom0*/

	/* else, compare the end_time of current running vcpu with the curr_slice
	 *
	 * If the end_time is larger, the next vcpu = the current one.
	 * Else, migrate the vcpu to the next pcpu according to the scheduling plan,
	 * and fetch the next vcpu from run queue.
	 *
	 * If its start_time is less or equal to curr_slice, start the execution
	 * else, idle for a time slice
	 *
	 * curr_slice++
	 */
}

static void
asym_dump_vcpu(struct csched_vcpu *svc){}

static void
asym_dump_pcpu(const struct scheduler *ops, int cpu){}

static void
asym_dump(const struct scheduler *ops){}

static int
asym_init(struct scheduler *ops)
{
	/* allocate private data*/
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

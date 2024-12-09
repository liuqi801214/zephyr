#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "zephyr/kernel.h"
#include "zephyr/kernel/thread.h"
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/multi_heap.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <kernel_internal.h> //for z_idle_threads and arch_kernel_init

#include "patch_os.h"
#include "mem_types.h"
#include "trace.h"
typedef enum
{
    SCHEDULER_SUSPENDED = 0,
    SCHEDULER_NOT_STARTED = 1,
    SCHEDULER_RUNNING = 2
} SCHEDULER_STATE;
#define TASK_SEM_ARRAY_NUMBER 5
#define CONFIG_ZEPHYR_PRI_MAX 6

typedef struct
{
    void *task_handle;
    void *sem_handle;
} task_sem_item;
task_sem_item task_sem_array[TASK_SEM_ARRAY_NUMBER] = {0};


#if (CONFIG_REALTEK_BT_CONTROLLER_STACK_SIZE < (1024+512))
#error "CONFIG_REALTEK_CONTROLLER_STACK_SIZE should be larger than 1.5KB"
#endif
#define LOWSTACK_STACKSIZE CONFIG_REALTEK_BT_CONTROLLER_STACK_SIZE

// K_THREAD_STACK_DEFINE(lowstack_stack, LOWSTACK_STACKSIZE);
struct z_thread_stack_element *lowstack_stack;
struct k_thread lowstack_thread_handle;

typedef struct timer_info
{
    void *timer_handle;
    uint32_t timer_id;
} Timer_Info;
#define TIMER_NUMBER_MAX 64
Timer_Info timer_number_array[TIMER_NUMBER_MAX];

BOOL_PATCH_FUNC patch_osif_os_task_name_get;
extern void sys_clock_announce_only_add_ticks(int32_t ticks);
/************************************************************ Mem Management ************************************************************/

// static struct sys_heap data_on_heap;
// static struct sys_heap buffer_on_heap;

static struct k_heap k_heap_array[RAM_TYPE_NUM];
struct sys_multi_heap multi_heap;

#define MULTI_HEAP_ADD(ram_type, node_label) \
    k_heap_init(&k_heap_array[ram_type],    \
                (void *)(DT_REG_ADDR(DT_NODELABEL(node_label))),    \
                DT_REG_SIZE(DT_NODELABEL(node_label))); \
    sys_multi_heap_add_heap(&multi_heap, &k_heap_array[ram_type].heap, NULL);

static void *os_heap_choice(struct sys_multi_heap *mheap, void *cfg, size_t align, size_t size)
{
    void *p = NULL;
    RAM_TYPE ram_type = (RAM_TYPE)cfg;

    /* debug */
    // DBG_DIRECT("os_heap_choice, ram_type:%d",ram_type);

    // k_spinlock_key_t key = k_spin_lock((&k_heap_array[ram_type])->lock);
    // p = sys_heap_aligned_alloc((&k_heap_array[ram_type])->heap, align, size);
    // k_spin_unlock((&k_heap_array[ram_type])->lock, key);

    p = k_heap_aligned_alloc(&k_heap_array[ram_type], align, size, K_NO_WAIT);

    if (p == NULL)
    {
        DBG_DIRECT("os_mem_alloc_intern_zephyr alloc failed! ram_type:%d alloc_size:0x%x", ram_type, size);
#ifdef CONFIG_SYS_HEAP_RUNTIME_STATS
        extern bool os_mem_peek_zephyr(RAM_TYPE ram_type, size_t *p_size);
        size_t p_size;
        os_mem_peek_zephyr(RAM_TYPE_DATA_ON, &p_size);
        os_mem_peek_zephyr(RAM_TYPE_BUFFER_ON, &p_size);
#endif
    }

    return p;
}

void os_heap_init_zephyr(void)
{
    // init all heap region, such as data on region, bufferON region

    static atomic_t state;

    if (!atomic_cas(&state, 0, 1))
    {
        DBG_DIRECT("os_heap_ini atomic operation failed!");
        return;
    }
    sys_multi_heap_init(&multi_heap, os_heap_choice);
    atomic_set(&state, 1);

    /* ram type: RAM_TYPE_DATA_ON */
    MULTI_HEAP_ADD(RAM_TYPE_DATA_ON, heap0);

    /* ram type: RAM_TYPE_BUFFER_ON */
    MULTI_HEAP_ADD(RAM_TYPE_BUFFER_ON, heap1);

}

bool os_init_zephyr(void)
{
    //shared multi-heap init
    os_heap_init_zephyr();

    return true;
}

/****************************************************************************/
/* Allocate memory                                                          */
/****************************************************************************/
bool os_mem_alloc_intern_zephyr(RAM_TYPE ram_type, size_t size,
                                const char *p_func, uint32_t file_line, void **pp)
{
    ARG_UNUSED(p_func);
    ARG_UNUSED(file_line);
    *pp = NULL;
    *pp = sys_multi_heap_alloc(&multi_heap, (void *)ram_type, size);
    if (*pp == NULL)
    {
        // DBG_DIRECT("os_mem_alloc_intern_zephyr alloc failed!");
    }
    return true;
}

bool os_mem_zalloc_intern_zephyr(RAM_TYPE ram_type, size_t size,
                                 const char *p_func, uint32_t file_line, void **pp)
{
    ARG_UNUSED(p_func);
    ARG_UNUSED(file_line);
    *pp = NULL;
    *pp = sys_multi_heap_alloc(&multi_heap, (void *)ram_type, size);
    if (*pp == NULL)
    {
        // DBG_DIRECT("os_mem_zalloc_intern_zephyr alloc failed! ram_type:%d", ram_type);
    }
    else
    {
        memset(*pp, 0, size);
    }
    return true;
}

/****************************************************************************/
/* Allocate aligned memory                                                  */
/****************************************************************************/
bool os_mem_aligned_alloc_intern_zephyr(RAM_TYPE ram_type, size_t size, uint8_t alignment,
                                        const char *p_func, uint32_t file_line, void **pp)
{
    ARG_UNUSED(p_func);
    ARG_UNUSED(file_line);
    *pp = NULL;
    *pp = sys_multi_heap_aligned_alloc(&multi_heap, (void *)ram_type,
                                       alignment, size);
    if (*pp == NULL)
    {
        DBG_DIRECT("os_mem_aligned_alloc_intern_zephyr alloc failed!");
        return 0;
    }
    return true;
}

/****************************************************************************/
/* Free memory                                                              */
/****************************************************************************/
bool os_mem_free_zephyr(void *p_block)
{
    sys_multi_heap_free(&multi_heap, p_block);
    return true;
}

/****************************************************************************/
/* Free aligned memory                                                      */
/****************************************************************************/
bool os_mem_aligned_free_zephyr(void *p_block)
{
    sys_multi_heap_free(&multi_heap, p_block);
    return true;
}

/****************************************************************************/
/* Peek unused (available) memory size                                      */
/****************************************************************************/
bool os_mem_peek_zephyr(RAM_TYPE ram_type, size_t *p_size)
{

#ifdef CONFIG_SYS_HEAP_RUNTIME_STATS
    struct sys_memory_stats stats;
    uint32_t heap_size;

    sys_heap_runtime_stats_get(&(k_heap_array[ram_type].heap), &stats);
    *p_size = stats.free_bytes;
    if (ram_type == RAM_TYPE_DATA_ON)
    {
        heap_size =  DT_REG_SIZE(DT_NODELABEL(heap0));
    }
    else
    {
        heap_size =  DT_REG_SIZE(DT_NODELABEL(heap1));
    }


    // printk("RAM type of heap: %d, heap size: %zu, allocated %zu, free %zu, max allocated %zu\n",
    //        ram_type, heap_size, stats.allocated_bytes, stats.free_bytes,
    //        stats.max_allocated_bytes);

    /* use DBG_DIRECT when zephyr log system is not initialized*/
    DBG_DIRECT("RAM type of heap: %d, heap size: %d, allocated %d, free %d, max allocated %d\n",
               ram_type, heap_size, stats.allocated_bytes, stats.free_bytes,
               stats.max_allocated_bytes);

#else
    // printk("System heap runtime statistics not enabled");

    /* use DBG_DIRECT when zephyr log system is not initialized*/
    DBG_DIRECT("System heap runtime statistics not enabled");
#endif
    return true;
}
void os_mem_peek_printf_zephyr(void)
{
    return;
}
/****************************************************************************/
/* Check os heap usage                                                      */
/****************************************************************************/
//todo
void os_mem_check_heap_usage_zephyr(void)
{
    return;
}

/************************************************************ msg queue ************************************************************/
bool os_msg_queue_create_intern_zephyr(void **pp_handle, uint32_t msg_num, uint32_t msg_size,
                                       const char *p_func, uint32_t file_line, bool *p_result)
{
    int ret;
    struct k_msgq *queue_obj;
    void *queue_buffer;
    size_t total_size = msg_size * msg_num;
    if (pp_handle)
    {
        /* alloc k_msgq */
        // queue_obj = (struct k_msgq *) shared_multi_heap_alloc(RAM_TYPE_DATA_ON, sizeof(struct k_msgq));
        queue_obj = sys_multi_heap_alloc(&multi_heap, (void *)RAM_TYPE_DATA_ON, sizeof(struct k_msgq));
        if (queue_obj != NULL)
        {
            memset(queue_obj, 0, sizeof(struct k_msgq));
            *pp_handle = queue_obj;

            /* alloc msgq buffer */
            // queue_buffer = shared_multi_heap_aligned_alloc(RAM_TYPE_DATA_ON, 4, total_size);
            queue_buffer = sys_multi_heap_aligned_alloc(&multi_heap, (void *)RAM_TYPE_DATA_ON,
                                                        4, total_size);
            if (queue_buffer != NULL)
            {
                k_msgq_init(queue_obj, queue_buffer, msg_size, msg_num);
                queue_obj->flags = K_MSGQ_FLAG_ALLOC;
                ret = 0;
            }
            else
            {
                // shared_multi_heap_free(queue_obj);
                sys_multi_heap_free(&multi_heap, queue_obj);
                DBG_DIRECT("alloc queue buffer failed because data ram heap is full");
                ret = -ENOMEM;
            }
        }
        else
        {
            DBG_DIRECT("alloc queue object failed because data ram heap is full");
            ret = -ENOMEM;
        }

        *p_result = (ret == 0) ? true : false;

    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_msg_queue_delete_intern_zephyr(void *p_handle, const char *p_func, uint32_t file_line,
                                       bool *p_result)
{
    struct k_msgq *obj;

    obj = (struct k_msgq *)p_handle;

    if ((obj->flags & K_MSGQ_FLAG_ALLOC) != 0U)
    {
        // shared_multi_heap_free(obj->buffer_start);
        sys_multi_heap_free(&multi_heap, obj->buffer_start);
        obj->buffer_start = NULL;
        obj->flags &= ~K_MSGQ_FLAG_ALLOC;
        // shared_multi_heap_free(obj);
        sys_multi_heap_free(&multi_heap, obj);
        *p_result = true;
    }
    *p_result = false;
    return true;
}

bool os_msg_queue_peek_intern_zephyr(void *p_handle, uint32_t *p_msg_num,
                                     const char *p_func, uint32_t file_line, bool *p_result)
{
    struct k_msgq *obj;

    if (p_handle)
    {
        obj = (struct k_msgq *)p_handle;
        // *p_msg_num = obj->used_msgs;
        *p_msg_num = k_msgq_num_used_get(obj);
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }
    return true;

}

bool os_msg_send_intern_zephyr(void *p_handle, void *p_msg, uint32_t wait_ms,
                               const char *p_func, uint32_t file_line, bool *p_result)
{
    int ret;
    struct k_msgq *obj;
    k_timeout_t wait_ticks;

    obj = (struct k_msgq *)p_handle;

    wait_ticks = (wait_ms == 0xFFFFFFFFUL) ? K_FOREVER : K_MSEC(wait_ms);
    ret = k_msgq_put(obj, p_msg, wait_ticks);

    *p_result = (ret == 0) ? true : false;
    return true;
}

bool os_msg_recv_intern_zephyr(void *p_handle, void *p_msg, uint32_t wait_ms,
                               const char *p_func, uint32_t file_line, bool *p_result)
{
    int ret;
    struct k_msgq *obj;
    k_timeout_t wait_ticks;

    obj = (struct k_msgq *)p_handle;

    wait_ticks = (wait_ms == 0xFFFFFFFFUL) ? K_FOREVER : K_MSEC(wait_ms);
    ret = k_msgq_get(obj, p_msg, wait_ticks);

    *p_result = (ret == 0) ? true : false;
    return true;
}
/************************************************************ SCHEDULAR ************************************************************/

bool os_sched_start_zephyr(bool *p_result)
{
    //TODO
    *p_result = true;
    return true;
}

bool os_sched_suspend_zephyr(bool *p_result)
{
    *p_result = true;
    k_sched_lock();
    return true;
}

bool os_sched_resume_zephyr(bool *p_result)
{
    *p_result = true;
    k_sched_unlock();
    return true;
}
extern struct k_thread *z_swap_next_thread(void);

bool os_sched_state_get_zephyr(SCHEDULER_STATE *p_result)
{

    if (z_swap_next_thread() == NULL)
    {
        *p_result = SCHEDULER_NOT_STARTED;
    }
    else if (_current->base.sched_locked != 0U)
    {
        *p_result = SCHEDULER_SUSPENDED;
    }
    else
    {
        *p_result = SCHEDULER_RUNNING;
    }
    return true;
}

bool os_sched_is_start_zephyr(bool *p_result)
{
    if (z_swap_next_thread() != NULL)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }
    return true;
}

bool os_delay_zephyr(uint32_t ms)
{
    k_sleep(K_MSEC(ms));
    return true;
}

/************************************************************ SYSTICK ************************************************************/
bool os_systick_handler_zephyr(void)
{
    // ToDo
    // extern void sys_clock_isr(void *arg);
    // sys_clock_isr(NULL);
    sys_clock_announce_only_add_ticks(1);
    return true;
}

bool os_sys_time_get_zephyr(uint64_t *p_time_ms)
{
    *p_time_ms = k_uptime_get();
    return true;
}

bool os_sys_tick_get_zephyr(uint64_t *p_sys_tick)
{
    *p_sys_tick = (uint64_t)k_uptime_ticks();

    return true;
}

bool os_sys_tick_increase_zephyr(uint32_t tick_increment,
                                 uint64_t *p_old_tick)
{
    *p_old_tick = sys_clock_tick_get();
    sys_clock_announce_only_add_ticks(tick_increment);
    return true;
}
/************************************************************ sync ************************************************************/

bool os_lock_zephyr(uint32_t *p_flags)
{
   #if defined(CONFIG_SEGGER_SYSTEMVIEW)
        SEGGER_SYSVIEW_PrintfHost("os_lock_zephyr\r\n");
	#endif 
    *p_flags = arch_irq_lock();
    return true;
}

bool os_unlock_zephyr(uint32_t flags)
{
    arch_irq_unlock(flags);
    return true;
}

bool os_sem_create_zephyr(void **pp_handle, const char *p_name, uint32_t init_count,
                          uint32_t max_count, bool *p_result)
{
    struct k_sem *sem_obj;
    sem_obj = (struct k_sem *) sys_multi_heap_alloc(&multi_heap, RAM_TYPE_DATA_ON,
                                                    sizeof(struct k_sem));

    if (sem_obj != NULL)
    {
        memset(sem_obj, 0, sizeof(struct k_sem));
        *pp_handle = sem_obj;
    }
    else
    {
        DBG_DIRECT("alloc sem object failed because data ram heap is full");
        *p_result = false;
    }

    k_sem_init(sem_obj, init_count, max_count);
    *p_result = true;
    return true;
}

bool os_sem_delete_zephyr(void *p_handle, bool *p_result)
{
    struct k_sem *obj;

    if (p_handle == NULL)
    {
        *p_result = false;
    }

    obj = (struct k_sem *)p_handle;

    sys_multi_heap_free(&multi_heap, obj);
    *p_result = true;
    return true;
}

bool os_sem_take_zephyr(void *p_handle, uint32_t wait_ms, bool *p_result)
{
    int ret;

    struct k_sem *obj;
    k_timeout_t wait_ticks;

    if (p_handle == NULL)
    {
        *p_result = false;
        return true;
    }

    obj = (struct k_sem *)p_handle;

    wait_ticks = (wait_ms == 0xFFFFFFFFUL) ? K_FOREVER : K_MSEC(wait_ms);
    ret = k_sem_take(obj, wait_ticks);

    *p_result = (ret == 0) ? true : false;
    return true;
}

bool os_sem_give_zephyr(void *p_handle, bool *p_result)
{
    struct k_sem *obj;

    if (p_handle == NULL)
    {
        *p_result  = false;
        return true;
    }
    else
    {
        obj = (struct k_sem *)p_handle;
        k_sem_give(obj);

        *p_result  = true;
        return true;
    }

}

bool os_mutex_create_zephyr(void **pp_handle, bool *p_result)
{
    struct k_mutex *mutex_obj;
    mutex_obj = (struct k_mutex *) sys_multi_heap_alloc(&multi_heap, RAM_TYPE_DATA_ON,
                                                        sizeof(struct k_mutex));

    if (mutex_obj != NULL)
    {
        memset(mutex_obj, 0, sizeof(struct k_mutex));
        *pp_handle = mutex_obj;
        k_mutex_init(mutex_obj);
        *p_result  = true;
        return true;
    }
    else
    {
        DBG_DIRECT("alloc mutex object failed because data ram heap is full");
        *p_result  = false;
        return true;
    }
}


bool os_mutex_delete_zephyr(void *p_handle, bool *p_result)
{
    struct k_mutex *obj;

    if (p_handle == NULL)
    {
        *p_result  = false;
        return true;
    }

    obj = (struct k_mutex *)p_handle;

    sys_multi_heap_free(&multi_heap, obj);
    *p_result  = true;
    return true;
}

bool os_mutex_take_zephyr(void *p_handle, uint32_t wait_ms, bool *p_result)
{
    int ret;

    struct k_mutex *obj;
    k_timeout_t wait_ticks;

    if (p_handle == NULL)
    {
        *p_result  = false;
        return true;
    }

    obj = (struct k_mutex *)p_handle;

    wait_ticks = (wait_ms == 0xFFFFFFFFUL) ? K_FOREVER : K_MSEC(wait_ms);
    ret = k_mutex_lock(obj, wait_ticks);

    *p_result = (ret == 0) ? true : false;
    return true;
}

bool os_mutex_give_zephyr(void *p_handle, bool *p_result)
{
    int ret;

    struct k_mutex *obj;

    if (p_handle == NULL)
    {
        *p_result  = false;
        return true;
    }

    obj = (struct k_mutex *)p_handle;

    ret = k_mutex_unlock(obj);

    *p_result = (ret == 0) ? true : false;
    return true;

}
/* ************************************************* OSIF TASK ************************************************* */
bool os_task_create_zephyr(void **pp_handle, const char *p_name, void (*p_routine)(void *),
                           void *p_param, uint16_t stack_size, uint16_t priority, bool *p_is_create_success)
{
    k_tid_t thread_id;

    /* The lowstack thread must be able to preempt all native cooperative threads in Zephyr
    (such as bt tx thread, mesh adv thread, BT Mesh settings workq and so on..).
    Otherwise, if lowstack thread is not processed in a timely manner within approximately 40ms,
    it may result in lowstack crashing and subsequently causing no event reply after hci command is sent, leading to a hardfault.

    Therefore, we need to make the lowstack thread a "Meta-IRQ thread", which can preempt coop threads and meta-irq threads.
    Specifically, the priority of lowstack thread is set to K_HIGHEST_THREAD_PRIO with CONFIG_NUM_METAIRQ_PRIORITIES=1.

    Reference: https://docs.zephyrproject.org/latest/kernel/services/threads/index.html#meta-irq-priorities  */

    if (strcmp(p_name, "low_stack_task") == 0)
    {
        /* place lowstack_stack(3KB) at RAM_TYPE_BUFFER_ON */
        k_thread_stack_t *lowstack_stack;
        lowstack_stack = (k_thread_stack_t *) \
                         sys_multi_heap_aligned_alloc(&multi_heap, RAM_TYPE_BUFFER_ON,
                                                      8, LOWSTACK_STACKSIZE);
        if (lowstack_stack == NULL)
        {
            DBG_DIRECT("alloc thread stack failed because buffer ram heap is full");
            *p_is_create_success = false;
            return true;
        }
        *pp_handle = &lowstack_thread_handle;
        thread_id = k_thread_create(&lowstack_thread_handle, lowstack_stack, LOWSTACK_STACKSIZE,
                                    (k_thread_entry_t) p_routine, p_param, NULL, NULL,
                                    K_HIGHEST_THREAD_PRIO, 0, K_MSEC(10));
        k_thread_name_set(thread_id, p_name);
        *p_is_create_success = true;
        return true;
    }

    k_tid_t thread_handle;

    int switch_priority = CONFIG_ZEPHYR_PRI_MAX - priority;

    thread_handle = (struct k_thread *) sys_multi_heap_alloc(&multi_heap, RAM_TYPE_DATA_ON,
                                                             sizeof(struct k_thread));
    if (thread_handle != NULL)
    {
        memset(thread_handle, 0, sizeof(struct k_thread));
        *pp_handle = thread_handle;
    }
    else
    {
        DBG_DIRECT("alloc thread object failed because data ram heap is full");
        *p_is_create_success = false;
        return true;
    }
    /****************************** dynamic stack ******************************/
    k_thread_stack_t *stack_buffer;
    stack_buffer = (k_thread_stack_t *) sys_multi_heap_aligned_alloc(&multi_heap, RAM_TYPE_DATA_ON,
                                                                     8, stack_size);
    if (stack_buffer == NULL)
    {
        sys_multi_heap_free(&multi_heap, thread_handle);
        thread_handle = NULL;
        DBG_DIRECT("alloc thread stack failed because data ram heap is full");
        *p_is_create_success = false;
        return true;
    }
    memset(stack_buffer, 0, stack_size);
    thread_id = k_thread_create(*pp_handle, stack_buffer, stack_size,
                                (k_thread_entry_t) p_routine, p_param, NULL, NULL,
                                switch_priority, 0, K_MSEC(10));

    if (thread_id != NULL)
    {
        k_thread_name_set(thread_id, p_name);
    }

    *p_is_create_success = true;
    return true;

}

bool os_task_delete_zephyr(void *p_handle, bool *p_result)
{
    struct k_thread *obj;

    obj = (struct k_thread *)p_handle;

    k_thread_abort(obj);

    *p_result = true;
    return true;
}

bool os_task_suspend_zephyr(void *p_handle, bool *p_result)
{
    struct k_thread *obj;

    obj = (struct k_thread *)p_handle;

    k_thread_suspend(obj);

    *p_result = true;
    return true;
}

bool os_task_resume_zephyr(void *p_handle, bool *p_result)
{
    struct k_thread *obj;

    obj = (struct k_thread *)p_handle;

    k_thread_resume(obj);

    *p_result = true;
    return true;
}

bool os_task_yield_zephyr(bool *p_result)
{
    k_yield();

    *p_result = true;
    return true;
}

bool os_task_handle_get_zephyr(void **pp_handle, bool *p_result)
{
    if (pp_handle == NULL)
    {
        *p_result = false;
        return true;
    }
    else
    {
        *pp_handle = k_current_get();
        *p_result = true;
        return true;
    }
}

bool os_task_priority_get_zephyr(void *p_handle, uint16_t *p_priority, bool *p_result)
{
    if (p_priority == NULL)
    {
        *p_result = false;
        return true;
    }
    struct k_thread *obj;
    uint16_t priority;

    if (p_handle != NULL)
    {
        obj = (struct k_thread *)p_handle;
    }
    else
    {
        obj = k_current_get();
    }
    priority = k_thread_priority_get(obj);
    *p_priority = CONFIG_ZEPHYR_PRI_MAX - priority;

    *p_result = true;
    return true;
}

bool os_task_priority_set_zephyr(void *p_handle, uint16_t priority, bool *p_result)
{
    struct k_thread *obj;
    uint16_t switch_priority = CONFIG_ZEPHYR_PRI_MAX - priority;

    if (p_handle != NULL)
    {
        obj = (struct k_thread *)p_handle;
    }
    else
    {
        obj = k_current_get();
    }

    k_thread_priority_set(obj, switch_priority);

    *p_result = true;
    return true;
}

bool os_task_signal_create_zephyr(void *p_handle, uint32_t count, bool *p_result)
{
    uint8_t i = 0;
    for (i = 0; i < TASK_SEM_ARRAY_NUMBER; i++)
    {
        if (task_sem_array[i].task_handle == NULL && task_sem_array[i].sem_handle == NULL)
        {
            break;
        }
    }

    struct k_sem *sem_obj;
    sem_obj = (struct k_sem *) sys_multi_heap_alloc(&multi_heap, RAM_TYPE_DATA_ON,
                                                    sizeof(struct k_sem));

    if (sem_obj != NULL)
    {
        memset(sem_obj, 0, sizeof(struct k_sem));
    }
    else
    {
        DBG_DIRECT("alloc sem object failed because data ram heap is full");
        *p_result = false;
        return true;
    }

    if (i < TASK_SEM_ARRAY_NUMBER)
    {
        int ret = k_sem_init(sem_obj, count, 0xFFFF);
        if (ret == 0)
        {
            #if defined(CONFIG_SEGGER_SYSTEMVIEW)
              SEGGER_SYSVIEW_PrintfHost("signal irq_lock\r\n");
	         #endif 
            uint32_t key = arch_irq_lock();
            task_sem_array[i].task_handle = p_handle;
            task_sem_array[i].sem_handle = sem_obj ;
            arch_irq_unlock(key);
            *p_result = true;
        }
        else
        {
            DBG_DIRECT("k_sem_init failed");
            sys_multi_heap_free(&multi_heap, sem_obj);
            *p_result = false;
        }
    }
    else
    {
        DBG_DIRECT("out of TASK_SEM_ARRAY_NUMBER");
        sys_multi_heap_free(&multi_heap, sem_obj);
        *p_result = false;

    }

    return true;
}
bool os_task_notify_take_zephyr(long xClearCountOnExit, uint32_t xTicksToWait,
                                uint32_t *p_notify, bool *p_result)
{
    k_timeout_t wait_ticks;
    wait_ticks = (xTicksToWait == 0xFFFFFFFFUL) ? K_FOREVER : K_TICKS(xTicksToWait);
    struct k_thread *thread_obj = k_current_get();

    for (uint8_t i = 0; i < TASK_SEM_ARRAY_NUMBER; i++)
    {

        if (task_sem_array[i].task_handle == thread_obj)
        {
            struct k_sem *sem = task_sem_array[i].sem_handle;
            int ret = k_sem_take(sem, wait_ticks);
            if (ret == 0)
            {
                *p_notify = sem->count + 1;
                *p_result = true;
            }
            else
            {
                *p_notify = 0;
                *p_result = false;
            }

            if (xClearCountOnExit == true)
            {
                sem->count = 0;
            }

            return true;
        }
    }
    *p_notify = 0;
    *p_result = false;
    return true;

}

bool os_task_notify_give_zephyr(void *p_handle, bool *p_result)
{
    *p_result = false;
    for (uint8_t i = 0; i < TASK_SEM_ARRAY_NUMBER; i++)
    {
        if (task_sem_array[i].task_handle == p_handle)
        {
            struct k_sem *sem = task_sem_array[i].sem_handle;

            k_sem_give(sem);
            *p_result = true;
            return true;
        }
    }
    return true;
}

bool os_task_status_dump_zephyr(void)
{
    struct k_thread *current_thread  = k_current_get();
    uint16_t priority = k_thread_priority_get(current_thread);
    /* To check */
    DBG_DIRECT("pxCurrentTCB %p, pxTopOfStack %p,taks_pri:%d",
               current_thread, current_thread ->callee_saved.psp,
               CONFIG_ZEPHYR_PRI_MAX - priority);
    return true;
}
/* ************************************************* OSIF TIMER ************************************************* */
bool os_timer_create_zephyr(void **pp_handle, const char *p_timer_name, uint32_t timer_id,
                            uint32_t interval_ms, bool reload, void (*p_timer_callback)(), bool *p_result)
{
    uint8_t i;
    k_timeout_t timer_ticks;

    timer_ticks = K_MSEC(interval_ms);
    /* Find a free space */
    for (i = 0; i < TIMER_NUMBER_MAX; i++)
    {
        if (timer_number_array[i].timer_handle == 0)
        {
            break;
        }
    }

    if (i == TIMER_NUMBER_MAX)
    {
        *p_result = false;
        return true;
    }
    else
    {
        struct k_timer *timer_obj;
        timer_obj = (struct k_timer *) sys_multi_heap_alloc(&multi_heap,
                                                            RAM_TYPE_DATA_ON, sizeof(struct k_timer));
        if (timer_obj != NULL)
        {
            memset(timer_obj, 0, sizeof(struct k_timer));
            *pp_handle = timer_obj;
        }
        else
        {
            DBG_DIRECT("alloc timer object failed because data ram heap is full, timer name:%s", p_timer_name);
            *p_result = false;
            return true;
        }

        if (reload == true)
        {
            k_timer_init(timer_obj, (k_timer_expiry_t) p_timer_callback, NULL);
            timer_obj->period = timer_ticks;
        }
        else
        {
            k_timer_init(timer_obj, (k_timer_expiry_t) p_timer_callback, NULL);
            timer_obj->period = K_NO_WAIT;
        }

        k_timeout_t *duration = sys_multi_heap_alloc(&multi_heap, RAM_TYPE_DATA_ON, sizeof(k_timeout_t));
        if (duration != NULL)
        {
            *duration = timer_ticks;
            k_timer_user_data_set(timer_obj, (void *)duration);
        }
        else
        {
            DBG_DIRECT("alloc timer userdata failed because data ram heap is full, timer name:%s",
                       p_timer_name);
            *p_result = false;
            return true;
        }
        #if defined(CONFIG_SEGGER_SYSTEMVIEW)
              SEGGER_SYSVIEW_PrintfHost("timer create irq_lock\r\n");
	    #endif 
        uint32_t key = arch_irq_lock();
        timer_number_array[i].timer_handle = (void *)(*pp_handle);
        timer_number_array[i].timer_id = timer_id;
        arch_irq_unlock(key);

        *p_result = true;
        return true;
    }
}

bool os_timer_id_get_zephyr(void **pp_handle, uint32_t *p_timer_id, bool *p_result)
{
    if (pp_handle && *pp_handle)
    {
        for (uint8_t i = 0; i < TIMER_NUMBER_MAX; i++)
        {
            if (timer_number_array[i].timer_handle == *pp_handle)
            {
                *p_timer_id = timer_number_array[i].timer_id;
                *p_result = true;
                return true;
            }
        }
    }
    *p_result = false;
    return true;
}

bool os_timer_start_zephyr(void **pp_handle, bool *p_result)
{
    struct k_timer *timer = *pp_handle;
    k_timeout_t *duration;
    if (timer)
    {
        if (!K_TIMEOUT_EQ(timer->period, K_NO_WAIT))
        {
            k_timer_start(timer, timer->period, timer->period);
        }
        else
        {

            duration = k_timer_user_data_get(timer);
            k_timer_start(timer, *duration, K_NO_WAIT);
        }
        *p_result = true;
        return true;
    }
    *p_result = false;
    return true;
}

bool os_timer_restart_zephyr(void **pp_handle, uint32_t interval_ms, bool *p_result)
{
    struct k_timer *timer = *pp_handle;
    k_timeout_t timer_ticks;
    k_timeout_t *duration;

    timer_ticks = K_MSEC(interval_ms);

    if (timer && (timer_ticks.ticks > 0))
    {
        if (!K_TIMEOUT_EQ(timer->period, K_NO_WAIT))
        {
            k_timer_start(timer, timer_ticks, timer_ticks);
        }
        else
        {
            duration = k_timer_user_data_get(timer);
            duration->ticks = timer_ticks.ticks;
            k_timer_start(timer, timer_ticks, K_NO_WAIT);
        }
        *p_result = true;
        return true;
    }
    *p_result = false;
    return true;
}

bool os_timer_stop_zephyr(void **pp_handle, bool *p_result)
{
    struct k_timer *timer = *pp_handle;
    if (timer)
    {
        k_timer_stop(timer);
        *p_result = true;
        return true;
    }
    *p_result = false;
    return true;

}

bool os_timer_delete_zephyr(void **pp_handle, bool *p_result)
{
    struct k_timer *timer = *pp_handle;
    k_timeout_t *duration;

    if (timer)
    {
        #if defined(CONFIG_SEGGER_SYSTEMVIEW)
              SEGGER_SYSVIEW_PrintfHost("timer dlete irq_lock\r\n");
	    #endif 
        uint32_t key = arch_irq_lock();
        for (uint8_t i = 0; i < TIMER_NUMBER_MAX; i++)
        {
            if (timer_number_array[i].timer_handle == *pp_handle)
            {
                timer_number_array[i].timer_handle = 0;
                break;
            }
        }
        arch_irq_unlock(key);

        k_timer_stop(timer);

        duration = k_timer_user_data_get(timer);
        sys_multi_heap_free(&multi_heap, duration);
        duration = NULL;
        sys_multi_heap_free(&multi_heap, timer);
        timer = NULL;

        *pp_handle = NULL;

        *p_result = true;
        return true;
    }
    *p_result = false;
    return true;
}

bool os_timer_dump_zephyr(bool *p_result)
{
    return true;
}

bool os_timer_state_get_zephyr(void **pp_handle, uint32_t *p_timer_state, bool *p_result)
{
    /* implements according to z_abort_timeout */
    if (pp_handle && *pp_handle)
    {
        struct k_timer *obj;

        obj = (struct k_timer *)*pp_handle;

        if (sys_dnode_is_linked(&obj->timeout.node))
        {
            *p_timer_state = 1;
        }
        else
        {
            *p_timer_state = 0;
        }
        *p_result = true;
        return true;
    }
    *p_result = false;
    return true;
}

bool os_timer_init_zephyr(void)
{
    return true;
}

bool os_timer_number_get_zephyr(void **pp_handle, uint32_t *p_timer_num, bool *p_result)
{
    if (pp_handle && *pp_handle)
    {
        for (uint8_t i = 0; i < TIMER_NUMBER_MAX; i++)
        {
            if (timer_number_array[i].timer_handle == *pp_handle)
            {
                *p_timer_num = i;
                *p_result = true;
                return true;
            }
        }
    }
    else
    {
        *p_timer_num = 0xFF;
        *p_result = false;
        return true;
    }

    *p_result = false;
    return true;
}
typedef void (*PendedFunctionOS_t)(void *para1, uint32_t para2);
typedef struct pend_call
{
    struct k_work work;
    PendedFunctionOS_t Pend_func;
    void *para1;
    uint32_t para2;
} Pend_Call;



void pendcall_handler(struct k_work *item)
{
    Pend_Call *cb =
        CONTAINER_OF(item, Pend_Call, work);
    DBG_DIRECT("pendcall para1: %x para2: %x\n", (uint32_t)cb->para1, cb->para2);
    cb->Pend_func(cb->para1, cb->para2);
    sys_multi_heap_free(&multi_heap, cb);
}

uint32_t xPendFunctionCall_zephyr(PendedFunctionOS_t xFunctionToPend, void *para1, uint32_t para2)
{
    int err;
    Pend_Call *cb = sys_multi_heap_alloc(&multi_heap, (void *)RAM_TYPE_DATA_ON,
                                         sizeof(Pend_Call));
    if (cb != NULL)
    {
        cb->Pend_func = xFunctionToPend;
        cb->para1 = para1;
        cb->para2 = para2;
        k_work_init(&cb->work, pendcall_handler);
        err = k_work_submit(&cb->work);
    }
    else
    {
        DBG_DIRECT("alloc pendcall data failed because data ram heap is full");
        err = 1;
    }
    return err;
}

bool os_timer_pendcall_zephyr(PendedFunctionOS_t xFunctionToPend, void *para1, uint32_t para2,
                              bool *p_result)
{
    uint32_t ret;
    ret = xPendFunctionCall_zephyr(xFunctionToPend, para1, para2);
    *p_result = (ret == 0) ? true : false;
    return true;
}

bool os_timer_next_timeout_value_get_zephyr(uint32_t *p_value)
{
    return true;
}



bool os_task_name_get_zephyr(void *p_handle, char **p_task_name, bool *p_result)
{
    struct k_thread *thread_obj;

    if (p_handle != NULL)
    {
        thread_obj = (struct k_thread *)p_handle;
    }
    else
    {
        thread_obj = k_current_get();
    }

    *p_task_name = k_thread_name_get(thread_obj);
    *p_result = true;
    return true;
}
/* ************************************************* OSIF PATCH ************************************************* */
void osif_mem_func_init_zephyr()
{
    patch_osif_os_mem_alloc_intern         = os_mem_alloc_intern_zephyr;
    patch_osif_os_mem_zalloc_intern        = os_mem_zalloc_intern_zephyr;
    patch_osif_os_mem_aligned_alloc_intern = os_mem_aligned_alloc_intern_zephyr;
    patch_osif_os_mem_free                 = os_mem_free_zephyr;
    patch_osif_os_mem_aligned_free         = os_mem_aligned_free_zephyr;
    patch_osif_os_mem_peek                 = os_mem_peek_zephyr;
    patch_osif_os_mem_check_heap_usage     = os_mem_check_heap_usage_zephyr;
}
void osif_msg_func_init_zephyr()
{
    patch_osif_os_msg_queue_create_intern = os_msg_queue_create_intern_zephyr;
    patch_osif_os_msg_queue_delete_intern = os_msg_queue_delete_intern_zephyr;
    patch_osif_os_msg_queue_peek_intern = os_msg_queue_peek_intern_zephyr;
    patch_osif_os_msg_send_intern = os_msg_send_intern_zephyr;
    patch_osif_os_msg_recv_intern = os_msg_recv_intern_zephyr;
    // patch_osif_os_msg_peek_intern = os_msg_peek_intern_zephyr;
}
void osif_sched_func_init_zephyr(void)
{
    patch_osif_os_init = os_init_zephyr;

    patch_osif_os_systick_handler = os_systick_handler_zephyr;
    patch_osif_os_delay = os_delay_zephyr;
    patch_osif_os_sys_time_get = os_sys_time_get_zephyr;
    patch_osif_os_sys_tick_get = os_sys_tick_get_zephyr;
    patch_osif_os_sched_start = os_sched_start_zephyr;
    // patch_osif_os_sched_stop = os_sched_stop_zephyr;
    patch_osif_os_sched_suspend = os_sched_suspend_zephyr;
    patch_osif_os_sched_resume = os_sched_resume_zephyr;
    patch_osif_os_sched_state_get = os_sched_state_get_zephyr;
    patch_osif_os_sched_is_start = os_sched_is_start_zephyr;
}

void osif_sync_func_init_zephyr(void)
{
    patch_osif_os_lock = os_lock_zephyr;
    patch_osif_os_unlock = os_unlock_zephyr;
    patch_osif_os_sem_create = os_sem_create_zephyr;
    patch_osif_os_sem_delete = os_sem_delete_zephyr;
    patch_osif_os_sem_take = os_sem_take_zephyr;
    patch_osif_os_sem_give = os_sem_give_zephyr;
    patch_osif_os_mutex_create = os_mutex_create_zephyr;
    patch_osif_os_mutex_delete = os_mutex_delete_zephyr;
    patch_osif_os_mutex_take = os_mutex_take_zephyr;
    patch_osif_os_mutex_give = os_mutex_give_zephyr;
}

void osif_task_func_init_zephyr(void)
{
    patch_osif_os_task_create = os_task_create_zephyr;
    patch_osif_os_task_delete = os_task_delete_zephyr;
    patch_osif_os_task_suspend = os_task_suspend_zephyr;
    patch_osif_os_task_resume = os_task_resume_zephyr;
    patch_osif_os_task_yield = os_task_yield_zephyr;
    patch_osif_os_task_handle_get = os_task_handle_get_zephyr;
    patch_osif_os_task_priority_get = os_task_priority_get_zephyr;
    patch_osif_os_task_priority_set = os_task_priority_set_zephyr;
    patch_osif_os_task_signal_create = os_task_signal_create_zephyr;
    patch_osif_os_task_notify_take = os_task_notify_take_zephyr;
    patch_osif_os_task_notify_give = os_task_notify_give_zephyr;
    patch_osif_os_task_status_dump = os_task_status_dump_zephyr;
    patch_osif_os_task_name_get = os_task_name_get_zephyr;
}

void osif_timer_func_init_zephyr(void)
{
    patch_osif_os_timer_id_get = os_timer_id_get_zephyr;
    patch_osif_os_timer_create =  os_timer_create_zephyr;
    patch_osif_os_timer_start = os_timer_start_zephyr;
    patch_osif_os_timer_restart = os_timer_restart_zephyr;
    patch_osif_os_timer_stop = os_timer_stop_zephyr;
    patch_osif_os_timer_delete = os_timer_delete_zephyr;
    patch_osif_os_timer_dump = os_timer_dump_zephyr;
    patch_osif_os_timer_state_get = os_timer_state_get_zephyr;
    patch_osif_os_timer_init = os_timer_init_zephyr;
    patch_osif_os_timer_number_get = os_timer_number_get_zephyr;
}

void os_zephyr_patch_init(void)
{
    osif_mem_func_init_zephyr();
    osif_msg_func_init_zephyr();
    // os_queue_func_init();
    osif_sched_func_init_zephyr();
    osif_sync_func_init_zephyr();
    osif_task_func_init_zephyr();
    osif_timer_func_init_zephyr();
    // osif_pm_func_init_zephyr();
}

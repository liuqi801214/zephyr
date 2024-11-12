/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>

#include "board.h"

#include "zephyr/kernel.h"
extern bool os_mem_peek_zephyr(uint8_t ram_type, size_t *p_size);

#include <zephyr\sys\sys_heap.h>
extern struct sys_heap z_malloc_heap;
void log_isr_stack_usage(void);


#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

static void attention_on(struct bt_mesh_model *mod)
{
	board_led_set(true);
}

static void attention_off(struct bt_mesh_model *mod)
{
	board_led_set(false);
}

static const struct bt_mesh_health_srv_cb health_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static const char *const onoff_str[] = { "off", "on" };

static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;

/* OnOff messages' transition time and remaining time fields are encoded as an
 * 8 bit value with a 6 bit step field and a 2 bit resolution field.
 * The resolution field maps to:
 * 0: 100 ms
 * 1: 1 s
 * 2: 10 s
 * 3: 20 min
 */
static const uint32_t time_res[] = {
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};

static inline int32_t model_time_decode(uint8_t val)
{
	uint8_t resolution = (val >> 6) & BIT_MASK(2);
	uint8_t steps = val & BIT_MASK(6);

	if (steps == 0x3f) {
		return SYS_FOREVER_MS;
	}

	return steps * time_res[resolution];
}

static inline uint8_t model_time_encode(int32_t ms)
{
	if (ms == SYS_FOREVER_MS) {
		return 0x3f;
	}

	for (int i = 0; i < ARRAY_SIZE(time_res); i++) {
		if (ms >= BIT_MASK(6) * time_res[i]) {
			continue;
		}

		uint8_t steps = DIV_ROUND_UP(ms, time_res[i]);

		return steps | (i << 6);
	}

	return 0x3f;
}

static int onoff_status_send(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

	remaining = k_ticks_to_ms_floor32(
			    k_work_delayable_remaining_get(&onoff.work)) +
		    onoff.transition_time;

	/* Check using remaining time instead of "work pending" to make the
	 * onoff status send the right value on instant transitions. As the
	 * work item is executed in a lower priority than the mesh message
	 * handler, the work will be pending even on instant transitions.
	 */
	if (remaining) {
		net_buf_simple_add_u8(&buf, !onoff.val);
		net_buf_simple_add_u8(&buf, onoff.val);
		net_buf_simple_add_u8(&buf, model_time_encode(remaining));
	} else {
		net_buf_simple_add_u8(&buf, onoff.val);
	}

	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

static void onoff_timeout(struct k_work *work)
{
	if (onoff.transition_time) {
		/* Start transition.
		 *
		 * The LED should be on as long as the transition is in
		 * progress, regardless of the target value, according to the
		 * Bluetooth Mesh Model specification, section 3.1.1.
		 */
		board_led_set(true);

		k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
		onoff.transition_time = 0;
		return;
	}

	board_led_set(onoff.val);
}

/* Generic OnOff Server message handlers */

static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	uint8_t val = net_buf_simple_pull_u8(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	int32_t trans = 0;
	int32_t delay = 0;

	if (buf->len) {
		trans = model_time_decode(net_buf_simple_pull_u8(buf));
		delay = net_buf_simple_pull_u8(buf) * 5;
	}

	/* Only perform change if the message wasn't a duplicate and the
	 * value is different.
	 */
	if (tid == onoff.tid && ctx->addr == onoff.src) {
		/* Duplicate */
		return 0;
	}

	if (val == onoff.val) {
		/* No change */
		return 0;
	}

	printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
	       trans);

	onoff.tid = tid;
	onoff.src = ctx->addr;
	onoff.val = val;
	onoff.transition_time = trans;

	/* Schedule the next action to happen on the delay, and keep
	 * transition time stored, so it can be applied in the timeout.
	 */
	k_work_reschedule(&onoff.work, K_MSEC(delay));

	return 0;
}

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Generic OnOff Client */

static int gen_onoff_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	uint8_t present = net_buf_simple_pull_u8(buf);

	if (buf->len) {
		uint8_t target = net_buf_simple_pull_u8(buf);
		int32_t remaining_time =
			model_time_decode(net_buf_simple_pull_u8(buf));

		printk("OnOff status: %s -> %s: (%d ms)\n", onoff_str[present],
		       onoff_str[target], remaining_time);
		return 0;
	}

	printk("OnOff status: %s\n", onoff_str[present]);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
	BT_MESH_MODEL_OP_END,
};

/* This application only needs one element to contain its models */
static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
		      NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* Provisioning */

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);

	board_output_number(action, number);

	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	board_prov_complete();
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16];

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

/** Send an OnOff Set message from the Generic OnOff Client to all nodes. */
//static int gen_onoff_send(bool val)
static int gen_onoff_send(bool val,uint16_t dst)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[3].keys[0], /* Use the bound key */
		//.addr = BT_MESH_ADDR_ALL_NODES,
		.addr = dst,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	static uint8_t tid;

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Client must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
	net_buf_simple_add_u8(&buf, val);
	net_buf_simple_add_u8(&buf, tid++);

	printk("Sending OnOff Set: %s\n", onoff_str[val]);

	return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);
}

static void button_pressed(struct k_work *work)
{
	if (bt_mesh_is_provisioned()) {
		//(void)gen_onoff_send(!onoff.val);
		(void)gen_onoff_send(!onoff.val,0);
		return;
	}

	/* Self-provision with an arbitrary address.
	 *
	 * NOTE: This should never be done in a production environment.
	 *       Addresses should be assigned by a provisioner, and keys should
	 *       be generated from true random numbers. It is done in this
	 *       sample to allow testing without a provisioner.
	 */
	static uint8_t net_key[16];
	static uint8_t dev_key[16];
	static uint8_t app_key[16];
	uint16_t addr;
	int err;

	if (IS_ENABLED(CONFIG_HWINFO)) {
		addr = sys_get_le16(&dev_uuid[0]) & BIT_MASK(15);
	} else {
		addr = k_uptime_get_32() & BIT_MASK(15);
	}

	printk("Self-provisioning with address 0x%04x\n", addr);
	err = bt_mesh_provision(net_key, 0, 0, 0, addr, dev_key);
	if (err) {
		printk("Provisioning failed (err: %d)\n", err);
		return;
	}

	/* Add an application key to both Generic OnOff models: */
	err = bt_mesh_app_key_add(0, 0, app_key);
	if (err) {
		printk("App key add failed (err: %d)\n", err);
		return;
	}

	/* Models must be bound to an app key to send and receive messages with
	 * it:
	 */
	models[2].keys[0] = 0;
	models[3].keys[0] = 0;

	printk("Provisioned and configured!\n");
}


char addr_s[BT_ADDR_LE_STR_LEN];//2024-10-29

/*********************************************************** */
#define NUM_ENTRIES 5 // 二维数组的大小
// 定义存储 PC、LR 和顺序标记的结构
typedef struct {
    uint32_t pc;
    uint32_t lr;
    uint32_t msp;
    uint32_t psp;
    uint32_t sequence; // 顺序标记
} cpu_state_t;

volatile cpu_state_t cpu_state_array[NUM_ENTRIES];
volatile uint32_t count = 0; // 存储位置计数器
volatile uint32_t global_sequence = 0; // 全局顺序计数器

#include "trace.h"
void save_cpu_state(uint32_t pc, uint32_t lr, uint32_t msp, uint32_t psp) {
    // 更新 cpu_state_array
    cpu_state_array[count].pc = pc;
    cpu_state_array[count].lr = lr;
    cpu_state_array[count].msp = msp;
    cpu_state_array[count].psp = psp;
    cpu_state_array[count].sequence = global_sequence++;

    // 更新计数器，使用取模操作实现循环存储
    count = (count + 1) % NUM_ENTRIES;
}
void print_cpu_state(void) {


    for (int i = 0; i < NUM_ENTRIES; i++)
    {
       DBG_DIRECT("Entry %d: PC = 0x%08X, LR = 0x%08X, msp = 0x%08X, psp = 0x%08X,Seq = %u\n", \
               i, cpu_state_array[i].pc, cpu_state_array[i].lr,  \
                  cpu_state_array[i].msp, cpu_state_array[i].psp, cpu_state_array[i].sequence);
               
    }	
    
}

/******************************************************************************* */

#include "trace.h"
void sys_trace_thread_switched_out_user(void) {

    // k_tid_t new_thread =k_current_get();
    // DBG_DIRECT("thread out: %s\n", new_thread->name ? new_thread->name : "unknown");


}

/******************************************************************************* */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
 
#define WATER_LEVEL_KERNEL_STACK_SYM z_interrupt_stacks
#define WATER_LEVEL_KERNEL_STACK_NAME STRINGIFY(WATER_LEVEL_KERNEL_STACK_SYM)
K_KERNEL_STACK_DECLARE(WATER_LEVEL_KERNEL_STACK_SYM, CONFIG_ISR_STACK_SIZE);
void log_isr_stack_usage(void)
{
    uintptr_t stack_start = (uintptr_t) WATER_LEVEL_KERNEL_STACK_SYM;
    size_t stack_size = CONFIG_ISR_STACK_SIZE;
    uintptr_t stack_end = stack_start + stack_size;
 
    /* Find the first unused stack location */
    uint8_t *p = (uint8_t *)stack_start;
    while ((p < (uint8_t *)stack_end) && (*p == 0xaa)) {
        p++;
    }
    size_t used = stack_end - (uintptr_t)p;
 
    printk(WATER_LEVEL_KERNEL_STACK_NAME" size: %zu, used: %zu, available: %zu \n",
            stack_size, used, stack_size - used);
}


static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

    /*get mac address for uuid 2024-10-29*/
    bt_addr_le_t device_addr;
    size_t count = 1;
    bt_id_get(&device_addr, &count);
    memcpy(dev_uuid,device_addr.a.val,6);
    bt_addr_le_to_str(&device_addr, addr_s, sizeof(addr_s));
    printk("advertising addr as %s\n", addr_s);


	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

   size_t p_size;
   os_mem_peek_zephyr(0, &p_size);
   os_mem_peek_zephyr(1, &p_size);
   struct sys_memory_stats stats;
	// low level接口
	sys_heap_runtime_stats_get(&z_malloc_heap, &stats);
	log_isr_stack_usage();
	printk("stdlib malloc heap: heap size: %d, allocated %d, free %d, max allocated %d\n", CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE, stats.allocated_bytes, stats.free_bytes, stats.max_allocated_bytes);
	printk("Mesh initialized\n");
}

/*在main.c的static void bt_ready(int err)之后,main()之前，添加如下code*/



/*
 sw timer
*/
#if defined(CONFIG_BT_MESH_SHELL)

#include <zephyr/shell/shell.h>
#include <stdio.h>
#include "mesh/access.h"
#include "mesh/net.h"
#include "common/bt_str.h"
#include "mesh/shell/utils.h"
#include <zephyr/bluetooth/mesh/shell.h>

struct app_key_val {
    uint16_t net_idx;
    bool updated;
    struct bt_mesh_key val[2];
} __packed;

char net_key_index[8];
void read_app_key(uint16_t app_idx,struct app_key_val *app_key);
struct app_key_val read_appkey;
K_MSGQ_DEFINE(swtimer_msgq, 32, 10, 4);
typedef struct
{
    uint8_t buf;
    uint32_t len;
} T_RTL_MESH_SEND_BUF;
T_RTL_MESH_SEND_BUF mesh_send_buf;

uint16_t mesh_msg_send_count=0;
uint16_t mesh_msg_send_max_count=0;

void timer_expired_handler(struct k_timer *timer);
K_TIMER_DEFINE(test_timer, timer_expired_handler, NULL);

uint16_t send_dst = 0;


void heap_test_timer_expired_handler(struct k_timer *timer);
K_TIMER_DEFINE(heap_test_timer, heap_test_timer_expired_handler, NULL);

uint16_t heap_mesh_msg_send_count=0;
uint16_t heap_msg_send_max_count=0;

static int cmd_heap_test_start(const struct shell *sh, size_t argc,
			      char **argv, uint32_t period)
{
	ARG_UNUSED(argv);
	k_timer_start(&heap_test_timer, K_MSEC(period), K_MSEC(period));//K_NO_WAIT);

	return 0;
}

void heap_test_timer_expired_handler(struct k_timer *timer)
{
    //LOG_INF("Timer expired");
	print_cpu_state();
    if(heap_msg_send_max_count!=0)
    {
       heap_mesh_msg_send_count++;
       if(heap_mesh_msg_send_count<heap_msg_send_max_count)
       { 
         mesh_send_buf.buf =2;
         k_msgq_put(&swtimer_msgq, &mesh_send_buf, K_NO_WAIT);
       }
       else
       {
        heap_mesh_msg_send_count=0;
        k_timer_stop(&heap_test_timer);
       }
    }
    else
    {
         mesh_send_buf.buf =2;
         k_msgq_put(&swtimer_msgq, &mesh_send_buf, K_NO_WAIT); 
    }
    
}


static int cmd_heap_test_start_demo(const struct shell *sh, size_t argc,
				   char **argv)
{

	int err = 0;
	uint16_t sw_period =200;
    uint16_t max_count=0;
	sw_period = shell_strtoul(argv[1], 0, &err);
	max_count= shell_strtoul(argv[2], 0, &err);
    heap_msg_send_max_count=max_count;
	shell_print(sh, "argv[0]=%s,argv[1]=%s,argv[2]=%s,sw_period=%d,max_count=%d,",argv[0],argv[1],argv[1],sw_period,\
	                 max_count);
	return cmd_heap_test_start(sh, argc, argv, sw_period);
}



void timer_expired_handler(struct k_timer *timer)
{
    //LOG_INF("Timer expired");
    if(mesh_msg_send_max_count!=0)
    {
       mesh_msg_send_count++;
       if(mesh_msg_send_count<mesh_msg_send_max_count)
       { 
         mesh_send_buf.buf =1;
         k_msgq_put(&swtimer_msgq, &mesh_send_buf, K_NO_WAIT);
       }
       else
       {
        mesh_msg_send_count=0;
        k_timer_stop(&test_timer);
       }
    }
    else
    {
         mesh_send_buf.buf =1;
         k_msgq_put(&swtimer_msgq, &mesh_send_buf, K_NO_WAIT); 
    }
    
}


static int cmd_mesh_test_start(const struct shell *sh, size_t argc,
			      char **argv, uint32_t period)
{
	ARG_UNUSED(argv);
	k_timer_start(&test_timer, K_MSEC(period), K_MSEC(period));//K_NO_WAIT);

	return 0;
}

static int cmd_mesh_test_start_demo(const struct shell *sh, size_t argc,
				   char **argv)
{
	//char sw_period=20;
	//char max_count=0;
	//sscanf(argv[1], "%hhd", &sw_period);
	//sscanf(argv[2], "%hhd", &max_count);
	int err = 0;
	uint16_t sw_period =200;
    uint16_t max_count=0;
	send_dst = shell_strtoul(argv[1], 0, &err);
	sw_period= shell_strtoul(argv[2], 0, &err);
	max_count= shell_strtoul(argv[3], 0, &err);
	mesh_msg_send_max_count = max_count;
	shell_print(sh, "argv[0]=%s,argv[1]=%s,dst=%d,argv[2]=%s,sw_period=%d,argv[3]=%s,max_count=%d,",argv[0],argv[1],send_dst,\
	                 argv[2],sw_period,argv[3],mesh_msg_send_max_count);
	return cmd_mesh_test_start(sh, argc, argv, sw_period);
}




int mesh_key_info(const struct shell *sh, size_t argc, char **argv)
{
    int err = 0;
    struct bt_mesh_subnet *sub;
    struct bt_mesh_elem *elem;
    //shell_print(sh, "rtl8762gn_evb");
    sub = bt_mesh_subnet_get(0);
    shell_print(sh, "argv[0]=%s",argv[0]);
    //sscanf(argv[1], "%hhd", net_key_index);
    net_key_index[0]=shell_strtoul(argv[1], 0, &err);
    read_app_key(0,&read_appkey);
    //unsigned int net_key_index =hex_string_to_int(argv[1]);
    shell_print(sh, "argv[1]=%s",argv[1]);
    shell_print(sh, "net_key_index[0]=%d",net_key_index[0]);
    shell_print(sh,"NetKey %s", bt_hex(&sub->keys[0].net, sizeof(struct bt_mesh_key)));
    shell_print(sh,"devKey %s", bt_hex(&bt_mesh.dev_key, sizeof(struct bt_mesh_key)));
    shell_print(sh,"appKey %s", bt_hex(&read_appkey.val[0], sizeof(struct bt_mesh_key)));
    shell_print(sh, "IV Index is 0x%08x", bt_mesh.iv_index);
    uint16_t ele_addr=bt_mesh_primary_addr();
    shell_print(sh, "primary_addr=%x",ele_addr);
    elem = bt_mesh_elem_find(ele_addr); 
    for (uint16_t i = 0U; i < comp.elem[0].model_count; i++) {
          shell_print(sh,"model_id(elem 0)=%x", comp.elem[0].models[i].id);
           if(ele_addr !=0) {
                for (uint8_t j = 0; j < elem->models[i].keys_cnt; j++) {
                   if (elem->models[i].keys[j] != BT_MESH_KEY_UNUSED) {
                      shell_print(sh,"appkeys_bind_idx =%d",elem->models[i].keys[j]);
                }
            }
        }
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mesh_user_cmds,     
       SHELL_CMD_ARG(heap-test, NULL, "[<period:ms> count:(0=>unlimited)>]", cmd_heap_test_start_demo, 1,2), 
       SHELL_CMD_ARG(key-show, NULL, "netkey index", mesh_key_info, 1, 1),
       SHELL_CMD_ARG(mesh-send, NULL, "[<Dst:(0=>0xffff)> <period:ms> <count:(0=>unlimited)>]", cmd_mesh_test_start_demo, 1, 3),
       #if CONFIG_PM
       #endif
     SHELL_SUBCMD_SET_END
    );
 SHELL_CMD_ARG_REGISTER(mesh_user, &mesh_user_cmds, "mesh user define commands", NULL, 1, 1);


#endif




int main(void)
{
	static struct k_work button_work;
	int err = -1;

	printk("Initializing...\n");

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

	k_work_init(&button_work, button_pressed);

	err = board_init(&button_work);
	if (err) {
		printk("Board init failed (err: %d)\n", err);
		return 0;
	}

	k_work_init_delayable(&onoff.work, onoff_timeout);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

    #if defined(CONFIG_BT_MESH_SHELL)
    /*
     2024-10-29 lq_liu
     get info from my_fifo to send mesh message by gen_onoff_send 
     */
     //T_RTL_MESH_SEND_BUF *rx_data;
     while (1) {
    
        k_msgq_get(&swtimer_msgq, &mesh_send_buf, K_FOREVER);
        if(mesh_send_buf.buf==1)
        {
           mesh_send_buf.buf=0;
           //lq_threads_test(lq_sh);
           onoff.val=!onoff.val;
           err=gen_onoff_send(onoff.val,send_dst);
           if(err!=0)
           {
              k_timer_stop(&test_timer);
              printk("err=%d gen_onoff_send\n",err);
           }
            //rx_data->buf=0;
            
        }
		else if(mesh_send_buf.buf==2)
		{
            mesh_send_buf.buf=0;
		//    size_t p_size;
        //    os_mem_peek_zephyr(0, &p_size);
		//    os_mem_peek_zephyr(1, &p_size);
		//    struct sys_memory_stats stats;
		// 	// low level接口
		// 	sys_heap_runtime_stats_get(&z_malloc_heap, &stats);
		// 	log_isr_stack_usage();
		// 	printk("stdlib malloc heap: heap size: %d, allocated %d, free %d, max allocated %d\n", CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE, stats.allocated_bytes, stats.free_bytes, stats.max_allocated_bytes);
        //     print_cpu_state();
		 if(heap_mesh_msg_send_count==9)
		 {
			DBG_DIRECT("test");
			while (1)
			{
				/* code */
			}
			
		 }
		}
        k_yield();
     }
    #endif

	return 0;
}


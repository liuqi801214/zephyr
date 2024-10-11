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
static int gen_onoff_send(bool val,uint16_t dst)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[3].keys[0], /* Use the bound key */
		.addr = dst,//0xc000,//BT_MESH_ADDR_ALL_NODES,2024-8-11
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
		(void)gen_onoff_send(!onoff.val,0);//2024-8-11
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

char addr_s[BT_ADDR_LE_STR_LEN];//2024-8-11

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
		printk("settings_load\n");
		settings_load();
	}

    /*get mac address for uuid 2024-8-11*/
    bt_addr_le_t device_addr;
    size_t count = 1;
    bt_id_get(&device_addr, &count);
    memcpy(dev_uuid,device_addr.a.val,6);
    bt_addr_le_to_str(&device_addr, addr_s, sizeof(addr_s));
    printk("advertising addr as %s\n", addr_s);

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    


	printk("Mesh initialized\n");
}

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


static int cmd_log_test_start(const struct shell *sh, size_t argc,
			      char **argv, uint32_t period)
{
	ARG_UNUSED(argv);
	k_timer_start(&test_timer, K_MSEC(period), K_MSEC(period));//K_NO_WAIT);
	shell_print(sh, "Log test started\n");

	return 0;
}

static int cmd_log_test_start_demo(const struct shell *sh, size_t argc,
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
	return cmd_log_test_start(sh, argc, argv, sw_period);
}
#if defined(CONFIG_BT_EXT_ADV)
#include "trace.h"
// extern uint32_t ext_adv_report_num;
// void ext_adv_report_timer_expired_handler(struct k_timer *timer);
// K_TIMER_DEFINE(ext_adv_report_timer, ext_adv_report_timer_expired_handler, NULL);

// void ext_adv_report_timer_expired_handler(struct k_timer *timer)
// {
//   //DBG_DIRECT("ext_adv_report_num=%d",ext_adv_report_num);
//   printk("ext_adv_report_num=%d\n",ext_adv_report_num);
// }

// static int ext_adv_report_timer_start(const struct shell *sh, size_t argc,
// 			      char **argv, uint32_t period)
// {
// 	ARG_UNUSED(argv);
// 	k_timer_start(&ext_adv_report_timer, K_MSEC(period), K_MSEC(period));//K_NO_WAIT);
// 	shell_print(sh, "Log ext adv report timer started\n");

// 	return 0;
// }
// static int ext_adv_report_start_demo(const struct shell *sh, size_t argc,
// 				   char **argv)
// {
// 	int err = 0;
// 	uint16_t show_period=1000;
// 	//sscanf(argv[1], "%hhd", &show_period);
// 	show_period= shell_strtoul(argv[1], 0, &err);
// 	shell_print(sh, "argv[0]=%s,argv[1]=%s,show_period=%d",argv[0],argv[1],show_period);
// 	return ext_adv_report_timer_start(sh, argc, argv, show_period);
// }
#endif

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

#if CONFIG_PM
extern bool is_app_enabled_dlps;
int pm_user_ctl(const struct shell *sh, size_t argc, char **argv)
{
   int err = 0;

   uint16_t pm_ctl = shell_strtoul(argv[1], 0, &err);
   if(pm_ctl ==0) is_app_enabled_dlps =false;
   else is_app_enabled_dlps =true;

   return 0;
}

#endif

// /* SHELL_CMD 注册两个子命令， board和version，执行时会调用cmd_info_board和cmd_info_version函数
//    SHELL_STATIC_SUBCMD_SET_CREATE 将子命令组装成子命令集subinfo
//    SHELL_SUBCMD_SET_END表示子命令集的结束
//  */
//     SHELL_STATIC_SUBCMD_SET_CREATE(subinfo,
// 	SHELL_CMD(show, NULL, "Show key command.", mesh_key_info),
// 	// SHELL_CMD(sw_timer, NULL,
// 	// 	  "Start log timer which generates log message every 200ms.",
// 	// 	  cmd_log_test_start_demo),
// 	#if defined(CONFIG_BT_EXT_ADV)
// 	SHELL_CMD(extadv_report, NULL, "Show key command.", ext_adv_report_start_demo),
// 	#endif
//     SHELL_SUBCMD_SET_END /* Array terminated. */
// 	);

// /* 注册一个根命令shell_sample，执行根命令shell_sample时会调用cmd_shell_help
//     shell_sample的子命令集为
//  */
// SHELL_CMD_REGISTER(mesh_key, &subinfo, "Sample commands", NULL);


// /* SHELL_CMD 注册一个子命令， 执行时会调用
//    SHELL_STATIC_SUBCMD_SET_CREATE 将子命令组装成子命令集subinfo
//    SHELL_SUBCMD_SET_END表示子命令集的结束
//  */
//      SHELL_STATIC_SUBCMD_SET_CREATE(sendinfo,
//      SHELL_CMD(sw_timer, NULL,
// 		  "Start log timer which generates log message every 200ms.",
// 		  cmd_log_test_start_demo), 
//       SHELL_SUBCMD_SET_END /* Array terminated. */
// 	);
// /* 注册一个根命令shell_sample，执行根命令shell_sample时会调用cmd_shell_help
//     shell_sample的子命令集为
//  */
// SHELL_CMD_REGISTER(mesh_send, &sendinfo, "generic on off send commands", NULL);


   SHELL_STATIC_SUBCMD_SET_CREATE(mesh_user_cmds,          
       SHELL_CMD_ARG(key-show, NULL, "netkey index", mesh_key_info, 1, 1),
	   SHELL_CMD_ARG(mesh-send, NULL, "[<Dst:(0=>0xffff)> <period:ms> <count:(0=>unlimited)>]", cmd_log_test_start_demo, 1, 3),
	   #if defined(CONFIG_BT_EXT_ADV)
	   //SHELL_CMD_ARG(extadv_report, NULL, "show period", ext_adv_report_start_demo, 1, 1),
	   #endif
	   #if CONFIG_PM
	   SHELL_CMD_ARG(pm-ctl, NULL, "[<ctl:(0=>exit 1=>enter)>]", pm_user_ctl, 1, 1),
	   #endif
     SHELL_SUBCMD_SET_END
    );
 SHELL_CMD_ARG_REGISTER(mesh_user, &mesh_user_cmds, "mesh user define commands",
			NULL, 1, 1);


#endif

#if CONFIG_PM
#if defined(CONFIG_BT_MESH_SHELL)
#include "rtl_pinmux.h"
#endif
#include "mesh/adv.h"
#include "trace.h"
#include "power_manager_unit_platform.h"
#include "pm.h"
bool is_app_enabled_dlps = true;

#if defined(CONFIG_BT_MESH_SHELL)
// void System_Handler(void)
// {
//     printk("System_Handler\n");
// 	if (System_WakeUpInterruptValue(P3_1) == SET)
//     {
//          Pad_ClearWakeupINTPendingBit(P3_1);
//          System_WakeUpPinDisable(P3_1);
// 		 is_app_enabled_dlps=false;
//          //Add user code here.
//      }
// }
#endif

enum PMCheckResult app_enter_dlps_check(void) {
    //DBG_DIRECT("app check dlps flag %d", app_global_data.is_app_enabled_dlps);
    //return app_global_data.is_app_enabled_dlps ? PM_CHECK_PASS : PM_CHECK_FAIL;
	//DBG_DIRECT("dlps_check\n");
	return is_app_enabled_dlps ? PM_CHECK_PASS : PM_CHECK_FAIL;
	//return is_app_enabled_dlps;
}

void sync_entim_exit_dlps_cb(void)
{
	 //printk("exit_dlps_cb\n");
	 uint32_t reason= power_get_wakeup_reason();
	 //DBG_DIRECT("exit_dlps_cb reason=0x%x",reason);
	 #if defined(CONFIG_BT_MESH_SHELL)
	  if (System_WakeUpInterruptValue(P3_1) == SET)
      {
		//DBG_DIRECT("wakeup by P31");
		Pad_ClearWakeupINTPendingBit(P3_1);
		System_WakeUpPinDisable(P3_1);
		is_app_enabled_dlps=false;
	  }
	  Pad_ControlSelectValue(P3_0, PAD_PINMUX_MODE);
      Pad_ControlSelectValue(P3_1, PAD_PINMUX_MODE);
	 #endif
      
}
void sync_entim_enter_dlps_cb(void)
{
    //printk("enter_dlps_cb\n");
	//DBG_DIRECT("enter_dlps_cb");
	#if defined(CONFIG_BT_MESH_SHELL)
	 Pad_ControlSelectValue(P3_0, PAD_SW_MODE);//tx pin
     Pad_ControlSelectValue(P3_1, PAD_SW_MODE);//rx pin
     System_WakeUpPinEnable(P3_1, PAD_WAKEUP_POL_LOW, 0);
	#endif
     
}

static void app_dlps_check_cb_register(void) {
    platform_pm_register_callback_func((void *)app_enter_dlps_check, PLATFORM_PM_CHECK);
}
static void app_dlps_enter_cb_register(void) {
    platform_pm_register_callback_func((void *)sync_entim_enter_dlps_cb, PLATFORM_PM_STORE);
}
static void app_dlps_exit_cb_register(void) {
    platform_pm_register_callback_func((void *)sync_entim_exit_dlps_cb, PLATFORM_PM_RESTORE);
}
#endif




int main(void)
{
	static struct k_work button_work;
	int err = -1;

    #if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
     #include "os_mem.h"
	 size_t os_mem_peek_zephyr(RAM_TYPE ram_type);
	 os_mem_peek_zephyr(RAM_TYPE_DATA_ON);
     os_mem_peek_zephyr(RAM_TYPE_BUFFER_ON);
	#endif
	
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
    
	#if CONFIG_PM
	app_dlps_check_cb_register();
    app_dlps_enter_cb_register();
    app_dlps_exit_cb_register();
    //bt_mesh_scan_disable();
	/*
	  Because using p31, the low level of uart rx to wake dlps, there is a bug, 
	  even if p31 is configured to pull up, as the shell's uart, entering dlps will wake dlps immediately, 
	  even if there is no low level, so a wakeup flag needs to be cleared first
	*/
    //Pad_ClearWakeupINTPendingBit(P3_1);
    #endif

     #if defined(CONFIG_BT_MESH_SHELL)
    /*
     2024-8-11 lq_liu
     get info from my_fifo to send mesh message by gen_onoff_send 
     */
     //T_RTL_MESH_SEND_BUF *rx_data;
	 #if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
     #include "os_mem.h"
	 size_t os_mem_peek_zephyr(RAM_TYPE ram_type);
	 uint32_t data_ram = os_mem_peek_zephyr(RAM_TYPE_DATA_ON);
     uint32_t buffer_ram = os_mem_peek_zephyr(RAM_TYPE_BUFFER_ON);
	 printk("data_ram=%d,buffer_ram=%d\n",data_ram,buffer_ram);
	 #endif
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
            k_yield();
        }    
     }
    #endif

	return 0;
}

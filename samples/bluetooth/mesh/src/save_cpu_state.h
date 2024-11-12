#ifndef SAVE_CPU_STATE_H
#define SAVE_CPU_STATE_H

#include <zephyr/kernel.h>

#define NUM_ENTRIES 10 // 二维数组的大小

typedef struct {
    uint32_t pc;
    uint32_t lr;
	uint32_t msp;
    uint32_t psp;
    uint32_t sequence; // 顺序标记
} cpu_state_t;



extern cpu_state_t cpu_state_array[NUM_ENTRIES];
extern volatile uint32_t count;// 存储位置计数器
extern uint32_t global_sequence;// 全局顺序计数器

// inline void save_cpu_state(void)
// {
//     uint32_t pc, lr, msp, psp;

//     // 获取主堆栈指针 (MSP)
//     __asm volatile ("MRS %0, MSP" : "=r" (msp));
    
//     // 获取进程堆栈指针 (PSP)
//     __asm volatile ("MRS %0, PSP" : "=r" (psp));

//     // 获取 PC 和 LR
//     __asm volatile ("MOV %0, PC" : "=r" (pc));
//     __asm volatile ("MOV %0, LR" : "=r" (lr));

//     // 更新 cpu_state_array
//     cpu_state_array[count].pc = pc;
//     cpu_state_array[count].lr = lr;
//     cpu_state_array[count].msp = msp;
//     cpu_state_array[count].psp = psp;
//     cpu_state_array[count].sequence = global_sequence++;

//     // 更新计数器，使用取模操作实现循环存储
//     count = (count + 1) % NUM_ENTRIES;
// }

// void save_cpu_state(uint32_t pc, uint32_t lr, uint32_t msp, uint32_t psp) {
//     // 更新 cpu_state_array
//     cpu_state_array[count].pc = pc;
//     cpu_state_array[count].lr = lr;
//     cpu_state_array[count].msp = msp;
//     cpu_state_array[count].psp = psp;
//     cpu_state_array[count].sequence = global_sequence++;

//     // 更新计数器，使用取模操作实现循环存储
//     count = (count + 1) % NUM_ENTRIES;
// }

#endif // SAVE_CPU_STATE_H
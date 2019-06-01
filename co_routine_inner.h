/*
* Tencent is pleased to support the open source community by making Libco available.

* Copyright (C) 2014 THL A29 Limited, a Tencent company. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License"); 
* you may not use this file except in compliance with the License. 
* You may obtain a copy of the License at
*
*	http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, 
* software distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
* See the License for the specific language governing permissions and 
* limitations under the License.
*/


#ifndef __CO_ROUTINE_INNER_H__

#include "co_routine.h"
#include "coctx.h"
struct stCoRoutineEnv_t;
struct stCoSpec_t
{
	void *value;
};

/*
 * 协程栈的数据结构表示
 * co_alloc_stackmem 分配
 */
struct stStackMem_t
{
	stCoRoutine_t* occupy_co;  // stack由哪个co使用
	int stack_size;      // stack size
    char* stack_bp;      // stack_buffer + stack_size，指向栈尾的指针，因为栈向内存低地址生长
	char* stack_buffer;  // stack buffer, 动态分配by co_alloc_stackmem

};

// co_get_stackmem
struct stShareStack_t
{
	unsigned int alloc_idx;  // 每有一个协程接入就+1，返回alloc_idx % count对应的stack
	int stack_size;
	int count;  // stack_array size
	stStackMem_t** stack_array;
};



struct stCoRoutine_t
{
	stCoRoutineEnv_t *env;
	pfn_co_routine_t pfn;
	void *arg;
	coctx_t ctx;          // 上下文表示，reg + stack(size+sp)，sp指向stack_mem里的stack buffer

	char cStart;          // 在执行协程前先置1(co_resume), 置1以后不会再置0
    char cEnd;            // 在执行协程pfn以后置1
	char cIsMain;         // env下的第一个协程，pfn为NULL
	char cEnableSysHook;  // 自定义协程函数首次运行前调用co_enable_hook_sys()置位
	char cIsShareStack;

	void *pvEnv;  // ??

	stStackMem_t* stack_mem;  // 协程栈


	//save satck buffer while confilct on same stack_buffer;
	char* stack_sp; 
	unsigned int save_size;
	char* save_buffer;

	stCoSpec_t aSpec[1024];

};



//1.env
void 				co_init_curr_thread_env();
stCoRoutineEnv_t *	co_get_curr_thread_env();

//2.coroutine
void    co_free( stCoRoutine_t * co );
void    co_yield_env(  stCoRoutineEnv_t *env );

//3.func



//-----------------------------------------------------------------------------------------------

struct stTimeout_t;
struct stTimeoutItem_t ;

stTimeout_t *AllocTimeout( int iSize );
void 	FreeTimeout( stTimeout_t *apTimeout );
int  	AddTimeout( stTimeout_t *apTimeout, stTimeoutItem_t *apItem ,uint64_t allNow );

struct stCoEpoll_t;
stCoEpoll_t * AllocEpoll();
void 		FreeEpoll( stCoEpoll_t *ctx );

stCoRoutine_t *		GetCurrThreadCo();
void 				SetEpoll( stCoRoutineEnv_t *env, stCoEpoll_t *ev );

typedef void (*pfnCoRoutineFunc_t)();

#endif

#define __CO_ROUTINE_INNER_H__

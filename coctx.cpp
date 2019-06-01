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

#include "coctx.h"
#include <string.h>


#define ESP 0
#define EIP 1
#define EAX 2
#define ECX 3
// -----------
#define RSP 0
#define RIP 1
#define RBX 2
#define RDI 3
#define RSI 4

#define RBP 5
#define R12 6
#define R13 7
#define R14 8
#define R15 9
#define RDX 10
#define RCX 11
#define R8 12
#define R9 13


//----- --------
// 32 bit
// | regs[0]: ret |
// | regs[1]: ebx |
// | regs[2]: ecx |
// | regs[3]: edx |
// | regs[4]: edi |
// | regs[5]: esi |
// | regs[6]: ebp |
// | regs[7]: eax |  = esp
enum
{
	kEIP = 0,
	kESP = 7,
};

//-------------
// 64 bit
//low | regs[0]: r15 |
//    | regs[1]: r14 |
//    | regs[2]: r13 |
//    | regs[3]: r12 |
//    | regs[4]: r9  |
//    | regs[5]: r8  | 
//    | regs[6]: rbp |
//    | regs[7]: rdi |
//    | regs[8]: rsi |
//    | regs[9]: ret |  //ret func addr
//    | regs[10]: rdx |
//    | regs[11]: rcx | 
//    | regs[12]: rbx |
//hig | regs[13]: rsp |
enum
{
	kRDI = 7,
	kRSI = 8,
	kRETAddr = 9,
	kRSP = 13,
};

//64 bit
extern "C"
{
	extern void coctx_swap( coctx_t *,coctx_t* ) asm("coctx_swap");
};
#if defined(__i386__)
int coctx_init( coctx_t *ctx )
{
	memset( ctx,0,sizeof(*ctx));
	return 0;
}

/*
 * 置ctx的指令跳转地址IP，栈寄存器SP，协程函数的2个输入参数指针
 * input:
 *     pfn
 *     s
 *     s1
 * output:
 *     ctx
 */
int coctx_make( coctx_t* ctx, coctx_pfn_t pfn, const void* s, const void* s1 )
{
    // 栈：
    // 高地址
    // | param->s2: s |
    // | param->s1: s1|  <- sp
    // |              |  <- esp(不存数据吗?????)
    // 低地址

	//make room for coctx_param
	char* sp = ctx->ss_sp + ctx->ss_size - sizeof(coctx_param_t);  // 栈底内存为coctx_param_t预留，执行这个协程时的2个指针输入参数
	sp = (char*)((unsigned long)sp & -16L);                        // sp栈底（不在栈内）地址对齐，低位0x00，向低地址方向对齐所以没有问题
	
	coctx_param_t* param = (coctx_param_t*)sp ;
	param->s1 = s;
	param->s2 = s1;

	memset(ctx->regs, 0, sizeof(ctx->regs));

	ctx->regs[ kESP ] = (char*)(sp) - sizeof(void*);               // esp指向栈底（在栈内）
	ctx->regs[ kEIP ] = (char*)pfn;

	return 0;
}

#elif defined(__x86_64__)
int coctx_make( coctx_t *ctx, coctx_pfn_t pfn, const void *s, const void *s1 )
{
	char *sp = ctx->ss_sp + ctx->ss_size;        // ss_sp为栈buffer低地址，sp指向栈底+8（因为栈向低地址生长）
	sp = (char*) ((unsigned long)sp & -16LL  );  // sp栈底（不在栈内）地址对齐，低位0x00，向低地址方向对齐所以没有问题

	memset(ctx->regs, 0, sizeof(ctx->regs));     // register清零

	ctx->regs[ kRSP ] = sp - 8;                  // esp指向栈底（在栈内）

    ctx->regs[ kRETAddr] = (char*)pfn;           // 跳转地址

	ctx->regs[ kRDI ] = (char*)s;                // 1st argument ???
	tx->regs[ kRSI ] = (char*)s1;                // 2nd argument ???
	return 0;
}

int coctx_init( coctx_t *ctx )
{
	memset( ctx,0,sizeof(*ctx));
	return 0;
}

#endif


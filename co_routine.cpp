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

#include "co_routine.h"
#include "co_routine_inner.h"
#include "co_epoll.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>

#include <poll.h>
#include <sys/time.h>
#include <errno.h>

#include <assert.h>

#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <limits.h>

extern "C"
{
	extern void coctx_swap( coctx_t *,coctx_t* ) asm("coctx_swap");
};
using namespace std;
stCoRoutine_t *GetCurrCo( stCoRoutineEnv_t *env );
struct stCoEpoll_t;

struct stCoRoutineEnv_t
{
	stCoRoutine_t *pCallStack[ 128 ];  // 协程调度栈，**正在**运行的协程(co_resume)
	int iCallStackSize;                // 当前调度栈的大小
	stCoEpoll_t *pEpoll;

	//for copy stack log lastco and nextco
	stCoRoutine_t* pending_co;    // co_swap中share stack mem使用
	stCoRoutine_t* occupy_co;     // co_swap中share stack mem使用
};

//int socket(int domain, int type, int protocol);
void co_log_err( const char *fmt,... )
{
}


#if defined( __LIBCO_RDTSCP__) 
static unsigned long long counter(void)
{
	register uint32_t lo, hi;
	register unsigned long long o;
	__asm__ __volatile__ (
			"rdtscp" : "=a"(lo), "=d"(hi)::"%rcx"
			);
	o = hi;
	o <<= 32;
	return (o | lo);

}
static unsigned long long getCpuKhz()
{
	FILE *fp = fopen("/proc/cpuinfo","r");
	if(!fp) return 1;
	char buf[4096] = {0};
	fread(buf, 1, sizeof(buf),fp);
	fclose(fp);

	char *lp = strstr(buf,"cpu MHz");
	if(!lp) return 1;
	lp += strlen("cpu MHz");
	while(*lp == ' ' || *lp == '\t' || *lp == ':')
	{
		++lp;
	}

	double mhz = atof(lp);
	unsigned long long u = (unsigned long long)(mhz * 1000);
	return u;
}
#endif

/*
 * 当前时间
 */
static unsigned long long GetTickMS()
{
#if defined( __LIBCO_RDTSCP__) 
	static uint32_t khz = getCpuKhz();
	return counter() / khz;
#else
	struct timeval now = { 0 };
	gettimeofday( &now, NULL );
	unsigned long long u = now.tv_sec;
	u *= 1000;
	u += now.tv_usec / 1000;
	return u;
#endif
}

static pid_t GetPid()
{
    static __thread pid_t pid = 0;
    static __thread pid_t tid = 0;
    if( !pid || !tid || pid != getpid() )
    {
        pid = getpid();
#if defined( __APPLE__ )
		tid = syscall( SYS_gettid );
		if( -1 == (long)tid )
		{
			tid = pid;
		}
#elif defined( __FreeBSD__ )
		syscall(SYS_thr_self, &tid);
		if( tid < 0 )
		{
			tid = pid;
		}
#else 
        tid = syscall( __NR_gettid );
#endif

    }
    return tid;

}
/*
static pid_t GetPid()
{
	char **p = (char**)pthread_self();
	return p ? *(pid_t*)(p + 18) : getpid();
}
*/
template <class T, class TLink>
void RemoveFromLink(T *ap)
{
	TLink *lst = ap->pLink;
	if(!lst) return ;
	assert( lst->head && lst->tail );

	if( ap == lst->head )
	{
		lst->head = ap->pNext;
		if(lst->head)
		{
			lst->head->pPrev = NULL;
		}
	}
	else
	{
		if(ap->pPrev)
		{
			ap->pPrev->pNext = ap->pNext;
		}
	}

	if( ap == lst->tail )
	{
		lst->tail = ap->pPrev;
		if(lst->tail)
		{
			lst->tail->pNext = NULL;
		}
	}
	else
	{
		ap->pNext->pPrev = ap->pPrev;
	}

	ap->pPrev = ap->pNext = NULL;
	ap->pLink = NULL;
}

template <class TNode, class TLink>
void inline AddTail(TLink*apLink, TNode *ap)
{
	if( ap->pLink )
	{
		return ;
	}
	if(apLink->tail)
	{
		apLink->tail->pNext = (TNode*)ap;
		ap->pNext = NULL;
		ap->pPrev = apLink->tail;
		apLink->tail = ap;
	}
	else
	{
		apLink->head = apLink->tail = ap;
		ap->pNext = ap->pPrev = NULL;
	}
	ap->pLink = apLink;
}
template <class TNode, class TLink>
void inline PopHead( TLink*apLink )
{
	if( !apLink->head ) 
	{
		return ;
	}
	TNode *lp = apLink->head;
	if( apLink->head == apLink->tail )
	{
		apLink->head = apLink->tail = NULL;
	}
	else
	{
		apLink->head = apLink->head->pNext;
	}

	lp->pPrev = lp->pNext = NULL;
	lp->pLink = NULL;

	if( apLink->head )
	{
		apLink->head->pPrev = NULL;
	}
}

/*
 * 把apOther合并到apLink，apOther会被清空
 */
template <class TNode, class TLink>
void inline Join( TLink*apLink, TLink *apOther )
{
	//printf("apOther %p\n",apOther);
	if( !apOther->head )
	{
		return ;
	}
	TNode *lp = apOther->head;
	while( lp )
	{
		lp->pLink = apLink;
		lp = lp->pNext;
	}
	lp = apOther->head;
	if(apLink->tail)
	{
		apLink->tail->pNext = (TNode*)lp;
		lp->pPrev = apLink->tail;
		apLink->tail = apOther->tail;
	}
	else
	{
		apLink->head = apOther->head;
		apLink->tail = apOther->tail;
	}

	apOther->head = apOther->tail = NULL;
}

/////////////////for copy stack //////////////////////////
/*
 * 动态分配协程栈
 * @input stack_size: 栈大小
 */
stStackMem_t* co_alloc_stackmem(unsigned int stack_size)
{
	stStackMem_t* stack_mem = (stStackMem_t*)malloc(sizeof(stStackMem_t));
	stack_mem->occupy_co= NULL;
	stack_mem->stack_size = stack_size;
	stack_mem->stack_buffer = (char*)malloc(stack_size);
	stack_mem->stack_bp = stack_mem->stack_buffer + stack_size;
	return stack_mem;
}

stShareStack_t* co_alloc_sharestack(int count, int stack_size)
{
	stShareStack_t* share_stack = (stShareStack_t*)malloc(sizeof(stShareStack_t));
	share_stack->alloc_idx = 0;
	share_stack->stack_size = stack_size;

	//alloc stack array
	share_stack->count = count;
	stStackMem_t** stack_array = (stStackMem_t**)calloc(count, sizeof(stStackMem_t*));
	for (int i = 0; i < count; i++)
	{
		stack_array[i] = co_alloc_stackmem(stack_size);
	}
	share_stack->stack_array = stack_array;
	return share_stack;
}

static stStackMem_t* co_get_stackmem(stShareStack_t* share_stack)
{
	if (!share_stack)
	{
		return NULL;
	}
	int idx = share_stack->alloc_idx % share_stack->count;
	share_stack->alloc_idx++;

	return share_stack->stack_array[idx];
}


// ----------------------------------------------------------------------------
struct stTimeoutItemLink_t;
struct stTimeoutItem_t;

struct stCoEpoll_t
{
	int iEpollFd;
	static const int _EPOLL_SIZE = 1024 * 10;

	struct stTimeout_t *pTimeout;  // 要被监控的stTimeoutItem_t都扔这里

	struct stTimeoutItemLink_t *pstTimeoutList; // 超时要被调度的timeout

	struct stTimeoutItemLink_t *pstActiveList;  // 确认有事件要被调度的timeout

	co_epoll_res *result;   // epoll的事件结果列表

};

typedef void (*OnPreparePfn_t)( stTimeoutItem_t *, struct epoll_event &ev, stTimeoutItemLink_t *active );
typedef void (*OnProcessPfn_t)( stTimeoutItem_t *);

struct stTimeoutItem_t
{

	enum
	{
		eMaxTimeout = 40 * 1000 //40s
	};
	stTimeoutItem_t *pPrev;
	stTimeoutItem_t *pNext;
	stTimeoutItemLink_t *pLink;

	unsigned long long ullExpireTime;

	OnPreparePfn_t pfnPrepare;
	OnProcessPfn_t pfnProcess;

	void *pArg; // routine 
	bool bTimeout;  // 如果timeout后取出来处理会打上, 用以在active link区分是timeout还是事件触发 (co_eventloop
};

struct stTimeoutItemLink_t
{
	stTimeoutItem_t *head;
	stTimeoutItem_t *tail;

};

/*
 * AllocTimeout分配 */
struct stTimeout_t
{
	stTimeoutItemLink_t *pItems;  // 共iItemSize个Link，时间轮，1ms 1个，60s
	int iItemSize;

	unsigned long long ullStart;  // 上一次处理timeout时间，ms级 (TakeAllTimeout)
	long long llStartIdx;         // 对应与ullStart时间的那一格link的idx
};


/*
 * 时间轮的管理初始化
 * 分配stTimeout_t, 里面有size个stTimeoutItemLink_t
 */
stTimeout_t *AllocTimeout( int iSize )
{
	stTimeout_t *lp = (stTimeout_t*)calloc( 1, sizeof(stTimeout_t) );	

	lp->iItemSize = iSize;
	lp->pItems = (stTimeoutItemLink_t*)calloc( 1, sizeof(stTimeoutItemLink_t) * lp->iItemSize );

	lp->ullStart = GetTickMS();
	lp->llStartIdx = 0;

	return lp;
}

void FreeTimeout( stTimeout_t *apTimeout )
{
	free( apTimeout->pItems );
	free ( apTimeout );
}

/*
 * 将apItem放进apTimeout链表里面
 * @input allNow: item开始时间 (过期时间在item里面)
 */
int AddTimeout( stTimeout_t *apTimeout, stTimeoutItem_t *apItem, unsigned long long allNow )
{
    // 理论上都在AllocTimeout时已经初始化了，这里其实是兜底
	if( apTimeout->ullStart == 0 )
	{
		apTimeout->ullStart = allNow;
		apTimeout->llStartIdx = 0;
	}

    // check
	if( allNow < apTimeout->ullStart )
	{
		co_log_err("CO_ERR: AddTimeout line %d allNow %llu apTimeout->ullStart %llu",
					__LINE__, allNow, apTimeout->ullStart);

		return __LINE__;
	}
	if( apItem->ullExpireTime < allNow )
	{
		co_log_err("CO_ERR: AddTimeout line %d apItem->ullExpireTime %llu allNow %llu apTimeout->ullStart %llu",
					__LINE__, apItem->ullExpireTime, allNow, apTimeout->ullStart);

		return __LINE__;
	}

    // 如果超过时间轮一圈，则置为可处理的最大
	unsigned long long diff = apItem->ullExpireTime - apTimeout->ullStart;
	if( diff >= (unsigned long long)apTimeout->iItemSize )
	{
		diff = apTimeout->iItemSize - 1;
		co_log_err("CO_ERR: AddTimeout line %d diff %d",
					__LINE__, diff);

		//return __LINE__;
	}

    // 放到对应时间轮超时的那一个slot的link上
	AddTail( apTimeout->pItems + ( apTimeout->llStartIdx + diff ) % apTimeout->iItemSize , apItem );

	return 0;
}

/*
 * @input&output stTimeout_t: 时间轮监控的timeout link集合，处理完会更新里面的start time&idx
 * @input allNow: now ms
 * @output stTimeoutItemLink_t: 结果
 */
inline void TakeAllTimeout( stTimeout_t *apTimeout, unsigned long long allNow, stTimeoutItemLink_t *apResult )
{
    // 如果还没有设置就设置一下 start ms
	if( apTimeout->ullStart == 0 )
	{
		apTimeout->ullStart = allNow;
		apTimeout->llStartIdx = 0;
	}

    // check
	if( allNow < apTimeout->ullStart )
	{
		return ;
	}

    // 从start到now，每一格的link的item都认真超时，直接整个link取出来合并到result返回
	int cnt = allNow - apTimeout->ullStart + 1;
	if( cnt > apTimeout->iItemSize )
	{
		cnt = apTimeout->iItemSize;
	}
	if( cnt < 0 )
	{
		return;
	}
	for( int i = 0;i<cnt;i++)
	{
		int idx = ( apTimeout->llStartIdx + i) % apTimeout->iItemSize;
		Join<stTimeoutItem_t, stTimeoutItemLink_t>( apResult, apTimeout->pItems + idx  );
	}

    // 更新Timeout的更新时间与idx
	apTimeout->ullStart = allNow;
	apTimeout->llStartIdx += cnt - 1;
}

static int CoRoutineFunc( stCoRoutine_t *co, void * )
{
	if( co->pfn )
	{
		co->pfn( co->arg );
	}
	co->cEnd = 1;

	stCoRoutineEnv_t *env = co->env;

	co_yield_env( env );

	return 0;
}



/*
 * 在对应env下创建协程，分配栈，指定了pfn和arg但没有make真正的context
 * @input: env
 * @input: attr可能为NULL
 * @input: pfn可能为NULL
 * @input: arg可能为NULL
 */
struct stCoRoutine_t *co_create_env( stCoRoutineEnv_t * env, const stCoRoutineAttr_t* attr,
		pfn_co_routine_t pfn, void *arg )
{
    // 1. 栈大小
	stCoRoutineAttr_t at;
	if( attr )
	{
		memcpy( &at, attr, sizeof(at) );
	}
	if( at.stack_size <= 0 )
	{
        // 最小128K
		at.stack_size = 128 * 1024;
	}
	else if( at.stack_size > 1024 * 1024 * 8 )
	{
        // 最大8M
		at.stack_size = 1024 * 1024 * 8;
	}

	if( at.stack_size & 0xFFF ) 
	{
        // 内存对齐
		at.stack_size &= ~0xFFF;
		at.stack_size += 0x1000;
	}

    // 2. 分配一个stCoRoutine，分配协程栈，ctx指向对应栈
	stCoRoutine_t *lp = (stCoRoutine_t*)malloc( sizeof(stCoRoutine_t) );
	memset( lp, 0,(long)(sizeof(stCoRoutine_t)));

	lp->env = env;
	lp->pfn = pfn;
	lp->arg = arg;

	stStackMem_t* stack_mem = NULL;
	if( at.share_stack )
	{
		stack_mem = co_get_stackmem( at.share_stack);
		at.stack_size = at.share_stack->stack_size;
	}
	else
	{
		stack_mem = co_alloc_stackmem(at.stack_size);
	}
	lp->stack_mem = stack_mem;

	lp->ctx.ss_sp = stack_mem->stack_buffer;
	lp->ctx.ss_size = at.stack_size;

	lp->cStart = 0;
	lp->cEnd = 0;
	lp->cIsMain = 0;
	lp->cEnableSysHook = 0;
	lp->cIsShareStack = at.share_stack != NULL;

	lp->save_size = 0;
	lp->save_buffer = NULL;

	return lp;
}

/*
 * 在当前线程的env下创建协程，分配栈，指定了pfn和arg但没有make真正的context 
 */
int co_create( stCoRoutine_t **ppco, const stCoRoutineAttr_t *attr, pfn_co_routine_t pfn, void *arg )
{
    // 如果env还没有初始化，先初始化
	if( !co_get_curr_thread_env() ) 
	{
		co_init_curr_thread_env();
	}

    // 在当前env下创建协程
	stCoRoutine_t *co = co_create_env( co_get_curr_thread_env(), attr, pfn, arg );
	*ppco = co;
	return 0;
}

void co_free( stCoRoutine_t *co )
{
    if (!co->cIsShareStack) 
    {    
        free(co->stack_mem->stack_buffer);
        free(co->stack_mem);
    }   
    free( co );
}

void co_release( stCoRoutine_t *co )
{
    co_free( co );
}

void co_swap(stCoRoutine_t* curr, stCoRoutine_t* pending_co);

/*
 * input: 
 *     co: 要执行的协程
 *
 * 对要执行的协程，如果还没有start，make ctx并置start标记位，
 * 然后在环境的协程调度栈CallStack中push该协程到栈顶，并co_swap执行该协程
 */
void co_resume( stCoRoutine_t *co )
{
	stCoRoutineEnv_t *env = co->env;
	stCoRoutine_t *lpCurrRoutine = env->pCallStack[ env->iCallStackSize - 1 ];
	if( !co->cStart )
	{
		coctx_make( &co->ctx, (coctx_pfn_t)CoRoutineFunc, co, 0 );
		co->cStart = 1;  // 开始运行前置start
	}
	env->pCallStack[ env->iCallStackSize++ ] = co;

    // 首次运行co时，co.ctx->regs[kESP]指向栈底，co.ctx->regs[kEip]指向pfn，其他为0(coctx_make)
    // 如果是第一个co，则lpCurrRoutine为NULL的那个co，且cStart==0, ctx->regs[kESP]==0, ctx->regs[kEIP]==0 (没有经coctx_make初始化)
	co_swap( lpCurrRoutine, co );
}

/*
 * 把当前co弹出栈，调度下一个co
 */
void co_yield_env( stCoRoutineEnv_t *env )
{
	
	stCoRoutine_t *last = env->pCallStack[ env->iCallStackSize - 2 ];
	stCoRoutine_t *curr = env->pCallStack[ env->iCallStackSize - 1 ];

	env->iCallStackSize--;

	co_swap( curr, last);
}

void co_yield_ct()
{

	co_yield_env( co_get_curr_thread_env() );
}

void co_yield( stCoRoutine_t *co )
{
	co_yield_env( co->env );
}

void save_stack_buffer(stCoRoutine_t* occupy_co)
{
	///copy out
	stStackMem_t* stack_mem = occupy_co->stack_mem;
	int len = stack_mem->stack_bp - occupy_co->stack_sp;

	if (occupy_co->save_buffer)
	{
		free(occupy_co->save_buffer), occupy_co->save_buffer = NULL;
	}

	occupy_co->save_buffer = (char*)malloc(len); //malloc buf;
	occupy_co->save_size = len;

	memcpy(occupy_co->save_buffer, occupy_co->stack_sp, len);
}

/*
 * 做上下文切换，调用汇编函数，把当前上下文保存到curr_ctx，并恢复pending_co_ctx的上下文
 */
void co_swap(stCoRoutine_t* curr, stCoRoutine_t* pending_co)
{
 	stCoRoutineEnv_t* env = co_get_curr_thread_env();

	// get curr stack sp
	char c;
	curr->stack_sp= &c;  // ?????

	if (!pending_co->cIsShareStack)
	{
        // 如果为NULL则不是share stack
		env->pending_co = NULL;
		env->occupy_co = NULL;
	}
	else 
	{
		env->pending_co = pending_co;
		// get last occupy co on the same stack mem
		stCoRoutine_t* occupy_co = pending_co->stack_mem->occupy_co;
		// set pending co to occupy thest stack mem;
		pending_co->stack_mem->occupy_co = pending_co;

		env->occupy_co = occupy_co;
		if (occupy_co && occupy_co != pending_co)
		{
			save_stack_buffer(occupy_co);
		}
	}

	// swap context
	coctx_swap(&(curr->ctx), &(pending_co->ctx) );

	// stack buffer may be overwrite, so get again; -- 意思就是切到另一个协程栈上了，前面的变量全都没有了
	stCoRoutineEnv_t* curr_env = co_get_curr_thread_env();
	stCoRoutine_t* update_occupy_co =  curr_env->occupy_co;
	stCoRoutine_t* update_pending_co = curr_env->pending_co;
	
	if (update_occupy_co && update_pending_co && update_occupy_co != update_pending_co)
	{
        // 如果不为NULL则是share stack
		// resume stack buffer
		if (update_pending_co->save_buffer && update_pending_co->save_size > 0)
		{
			memcpy(update_pending_co->stack_sp, update_pending_co->save_buffer, update_pending_co->save_size);
		}
	}
}



/*
 * co_poll_inner时使用，对poll相关信息的包装
 * 设置了OnPollProcessEvent
 */
//int poll(struct pollfd fds[], nfds_t nfds, int timeout);
// { fd, events, revents }
struct stPollItem_t ;
struct stPoll_t : public stTimeoutItem_t 
{
	struct pollfd *fds;
	nfds_t nfds; // typedef unsigned long int nfds_t;

	stPollItem_t *pPollItems;  // 每个pollfd对应一个PollItem

	int iAllEventDetach;  // 用来对stPoll_t做仅有的一次从timeout link remove的标记

	int iEpollFd;

	int iRaiseCnt;  // 表示poll_t中有事件的item数, poll接口语义的返回
};

/*
 * co_poll_inner时使用
 * poll的单个fd的包装
 * 设置了OnPollPreparePfn
 */
struct stPollItem_t : public stTimeoutItem_t
{
	struct pollfd *pSelf;  // poll对应的pollfd
	stPoll_t *pPoll;       // 总控的stPoll_t

	struct epoll_event stEvent;  // pollfd的真正事件循环实现是epoll，
                                 // 里面的data.ptr指向这个stPollItem_t(co_event_loop使用),
                                 // co_epoll_ctl时使用这个epoll_event，result不在这里而是在co_epoll_res
};

/*
 *   EPOLLPRI 		POLLPRI    // There is urgent data to read.  
 *   EPOLLMSG 		POLLMSG
 *
 *   				POLLREMOVE
 *   				POLLRDHUP
 *   				POLLNVAL
 *
 * */
static uint32_t PollEvent2Epoll( short events )
{
	uint32_t e = 0;	
	if( events & POLLIN ) 	e |= EPOLLIN;
	if( events & POLLOUT )  e |= EPOLLOUT;
	if( events & POLLHUP ) 	e |= EPOLLHUP;
	if( events & POLLERR )	e |= EPOLLERR;
	if( events & POLLRDNORM ) e |= EPOLLRDNORM;
	if( events & POLLWRNORM ) e |= EPOLLWRNORM;
	return e;
}

static short EpollEvent2Poll( uint32_t events )
{
	short e = 0;	
	if( events & EPOLLIN ) 	e |= POLLIN;
	if( events & EPOLLOUT ) e |= POLLOUT;
	if( events & EPOLLHUP ) e |= POLLHUP;
	if( events & EPOLLERR ) e |= POLLERR;
	if( events & EPOLLRDNORM ) e |= POLLRDNORM;
	if( events & EPOLLWRNORM ) e |= POLLWRNORM;
	return e;
}

static stCoRoutineEnv_t* g_arrCoEnvPerThread[ 204800 ] = { 0 };

/*
 * 初始化当前线程的env
 * 为什么当前协程pfn=NULL: 因为当切换另一个协程的时候，当前的所有寄存器的值如ip会保存在ctx，
 *                         创建协程指定pfn是入口，当前上下文已经运行到中间了
 */
void co_init_curr_thread_env()
{
    // 1. 从全局变量取出当前线程对应的env
	pid_t pid = GetPid();	
	g_arrCoEnvPerThread[ pid ] = (stCoRoutineEnv_t*)calloc( 1, sizeof(stCoRoutineEnv_t) );
	stCoRoutineEnv_t *env = g_arrCoEnvPerThread[ pid ];

    // 2. 创建此env第一个协程，但pfn是NULL, 没有ctx，有栈, pvEnv也是NULL
	env->iCallStackSize = 0;
	struct stCoRoutine_t *self = co_create_env( env, NULL, NULL, NULL );
	self->cIsMain = 1;

    // 3. 初始化env的其他信息
	env->pending_co = NULL;
	env->occupy_co = NULL;

	coctx_init( &self->ctx );  // ctx清零
	env->pCallStack[ env->iCallStackSize++ ] = self;  // 把这个pfn=NULL的协程放在栈底，表示当前

    // 4. Epoll初始化
	stCoEpoll_t *ev = AllocEpoll();  // 给这个env创建epoll, 包括epollfd，时间轮
	SetEpoll( env, ev );  // env->pEpoll = ev;
}

/*
 * 获取当前线程的env
 */
stCoRoutineEnv_t* co_get_curr_thread_env()
{
	return g_arrCoEnvPerThread[ GetPid() ];
}

/*
 * 运行对应的协程
 */
void OnPollProcessEvent( stTimeoutItem_t * ap )
{
	stCoRoutine_t *co = (stCoRoutine_t*)ap->pArg;
	co_resume( co );
}

/*
 * co_event_loop epoll_wait有事件后的预处理
 *      1. 把事件记到item里
 *      2. 记录stPoll_t的raisecnt
 *      3. 仅做一次stPoll_t从timeout link remove并add到active
 * @input ap: timeout item (也是poll item)
 * @input e: epoll_event 存储的是epoll的结果
 */
void OnPollPreparePfn( stTimeoutItem_t * ap, struct epoll_event &e, stTimeoutItemLink_t *active )
{
	stPollItem_t *lp = (stPollItem_t *)ap;
	lp->pSelf->revents = EpollEvent2Poll( e.events );  // 把epoll结果事件转为poll的并存进poll item


	stPoll_t *pPoll = lp->pPoll;
	pPoll->iRaiseCnt++;  // raisecnt 表示poll_t中有事件的item数

	if( !pPoll->iAllEventDetach )  // 如果为0（默认就是0）
	{
        // 这里的意思是: 如果poll item有一个有事件，就把stPoll_t从timeout link remove掉
        // 因为这时候stPoll_t可能还没有超时，但是已经需要处理
        // 这个动作只做一次
        //
        // 如果为0，置1，保证只处理一次
        // 把stPoll_t从timeout link remove掉
        // 并加到active
		pPoll->iAllEventDetach = 1;

		RemoveFromLink<stTimeoutItem_t, stTimeoutItemLink_t>( pPoll );

		AddTail( active, pPoll );
	}
}


/*
 * @input pfn&&arg: 在最后要调度的函数及其参数，可以为空
 */
void co_eventloop( stCoEpoll_t *ctx, pfn_co_eventloop_t pfn, void *arg )
{
	if( !ctx->result )
	{
		ctx->result =  co_epoll_res_alloc( stCoEpoll_t::_EPOLL_SIZE );
	}
	co_epoll_res *result = ctx->result;


	for(;;)
	{
        // epoll_wait 取出结果，或者timeout，时间轮走一格1ms
		int ret = co_epoll_wait( ctx->iEpollFd, result, stCoEpoll_t::_EPOLL_SIZE, 1 );  // timeout 1 ms

		stTimeoutItemLink_t *active = (ctx->pstActiveList); // active可能已经有数据，比如co_cond_signal

		stTimeoutItemLink_t *timeout = (ctx->pstTimeoutList); // timeout则是到这时候才开始处理
		memset( timeout, 0, sizeof(stTimeoutItemLink_t) );

		for(int i=0;i<ret;i++)
		{
			stTimeoutItem_t *item = (stTimeoutItem_t*)result->events[i].data.ptr;  // 约定epoll_events.data.ptr存着item
			if( item->pfnPrepare )
			{
                // 有prepare运行 (比如多个item只对应一个timeout item，则在这里把timeout item放到active即可 (OnPollPreparePfn)
				item->pfnPrepare( item, result->events[i], active );
			}
			else
			{
                // 没有prepare放到active
				AddTail( active, item );
			}
		}


        // 把超时的时间轮stTimeout_t中的link全取出来放到timeout link
		unsigned long long now = GetTickMS();
		TakeAllTimeout( ctx->pTimeout, now, timeout );

        // 把timeout中的item全部置timeout flag
		stTimeoutItem_t *lp = timeout->head;
		while( lp )
		{
			//printf("raise timeout %p\n",lp);
			lp->bTimeout = true;
			lp = lp->pNext;
		}

        // 把timeout合并到active
		Join<stTimeoutItem_t, stTimeoutItemLink_t>( active, timeout );

        // 处理每一个active link timeout item
		lp = active->head;
		while( lp )
		{
			PopHead<stTimeoutItem_t, stTimeoutItemLink_t>( active );

            // 1. 如果是timeout，需要重新加回stTimeout_t时间轮下等下一次超时
            if (lp->bTimeout && now < lp->ullExpireTime) 
			{
				int ret = AddTimeout(ctx->pTimeout, lp, now);
				if (!ret) 
				{
					lp->bTimeout = false;
					lp = active->head;
					continue;
				}
			}

            // 2. 如果是有事件的item，调用process函数
			if( lp->pfnProcess )
			{
				lp->pfnProcess( lp );
			}

			lp = active->head;
		}

        // 最后运行一下用户指定的周期性函数
		if( pfn )
		{
			if( -1 == pfn( arg ) )
			{
				break;
			}
		}

	}
}

void OnCoroutineEvent( stTimeoutItem_t * ap )
{
	stCoRoutine_t *co = (stCoRoutine_t*)ap->pArg;
	co_resume( co );
}


/*
 * 动态分配协程环境epoll，包括epollfd，时间轮
 */
stCoEpoll_t *AllocEpoll()
{
	stCoEpoll_t *ctx = (stCoEpoll_t*)calloc( 1, sizeof(stCoEpoll_t) );

	ctx->iEpollFd = co_epoll_create( stCoEpoll_t::_EPOLL_SIZE );  // epoll_create, 10240

	ctx->pTimeout = AllocTimeout( 60 * 1000 ); // 时间轮的管理初始化 60s * 1000ms/s
	
	ctx->pstActiveList = (stTimeoutItemLink_t*)calloc( 1, sizeof(stTimeoutItemLink_t) );
	ctx->pstTimeoutList = (stTimeoutItemLink_t*)calloc( 1, sizeof(stTimeoutItemLink_t) );


	return ctx;
}

void FreeEpoll( stCoEpoll_t *ctx )
{
	if( ctx )
	{
		free( ctx->pstActiveList );
		free( ctx->pstTimeoutList );
		FreeTimeout( ctx->pTimeout );
		co_epoll_res_free( ctx->result );
	}
	free( ctx );
}

/*
 * 获取当前co（也是栈顶的co）
 */
stCoRoutine_t *GetCurrCo( stCoRoutineEnv_t *env )
{
	return env->pCallStack[ env->iCallStackSize - 1 ];
}

stCoRoutine_t *GetCurrThreadCo( )
{
	stCoRoutineEnv_t *env = co_get_curr_thread_env();
	if( !env ) return 0;
	return GetCurrCo(env);
}


/*
 * @input pollfunc: 可能为NULL
 * @input timout: 如果timeout==0立即调用pollfunc
 */
typedef int (*poll_pfn_t)(struct pollfd fds[], nfds_t nfds, int timeout);
int co_poll_inner( stCoEpoll_t *ctx, struct pollfd fds[], nfds_t nfds, int timeout, poll_pfn_t pollfunc)
{
    if (timeout == 0)
	{
		return pollfunc(fds, nfds, timeout);
	}
	if (timeout < 0)
	{
		timeout = INT_MAX;
	}
	int epfd = ctx->iEpollFd;
	stCoRoutine_t* self = co_self();

	//1.struct change
    // 分配stPoll_t初始化: epfd, pollfds, pfnProcess(stTimeoutItem_t), PollItems分配内存但未有数据
	stPoll_t& arg = *((stPoll_t*)malloc(sizeof(stPoll_t)));
	memset( &arg, 0, sizeof(arg) );

	arg.iEpollFd = epfd;
	arg.fds = (pollfd*)calloc(nfds, sizeof(pollfd));
	arg.nfds = nfds;

	stPollItem_t arr[2];
	if( nfds < sizeof(arr) / sizeof(arr[0]) && !self->cIsShareStack)
	{
        // 如果poll items很少就直接用栈变量，减少alloc的开销
		arg.pPollItems = arr;
	}	
	else
	{
		arg.pPollItems = (stPollItem_t*)malloc( nfds * sizeof( stPollItem_t ) );
	}
	memset( arg.pPollItems, 0, nfds * sizeof(stPollItem_t) );

	arg.pfnProcess = OnPollProcessEvent;
	arg.pArg = GetCurrCo( co_get_curr_thread_env() );
	
	
	//2. add epoll
    // 分配stPoll_t中的stPollItem_t的数据：pollfd, epoll_event, pfnPrepare(stTimeoutItem_t)
    // 虽然是poll的参数，背后还是使用epoll，stPollItem_t作为epoll_event的数据
    // Add到对应的epoll
	for(nfds_t i=0;i<nfds;i++)
	{
		arg.pPollItems[i].pSelf = arg.fds + i;
		arg.pPollItems[i].pPoll = &arg;

		arg.pPollItems[i].pfnPrepare = OnPollPreparePfn;
		struct epoll_event &ev = arg.pPollItems[i].stEvent;

		if( fds[i].fd > -1 )
		{
			ev.data.ptr = arg.pPollItems + i;  // co_event_loop的约定，data.ptr指向item
			ev.events = PollEvent2Epoll( fds[i].events );

			int ret = co_epoll_ctl( epfd, EPOLL_CTL_ADD, fds[i].fd, &ev );
			if (ret < 0 && errno == EPERM && nfds == 1 && pollfunc != NULL)
			{
				if( arg.pPollItems != arr )
				{
					free( arg.pPollItems );
					arg.pPollItems = NULL;
				}
				free(arg.fds);
				free(&arg);
				return pollfunc(fds, nfds, timeout);
			}
		}
		//if fail, the timeout would work
	}

	//3.add timeout
    // 设置stPoll_t 中的expired_time(stTimeoutItem_t)
	unsigned long long now = GetTickMS();
	arg.ullExpireTime = now + timeout;
	int ret = AddTimeout( ctx->pTimeout, &arg, now );  // 只是把stPoll_t放到了时间轮中，没有把item都放一遍
	int iRaiseCnt = 0;
	if( ret != 0 )
	{
		co_log_err("CO_ERR: AddTimeout ret %d now %lld timeout %d arg.ullExpireTime %lld",
				ret, now, timeout, arg.ullExpireTime);
		errno = EINVAL;
		iRaiseCnt = -1;

	}
    else
	{
        // 弹出当前协程，调度前一个
        // 会不会main co也调用这个导致core????
        //   => 不会??，因为只有hook syscall才会跑到这里，需要每个协程函数显式hook，main没有显式hook
		co_yield_env( co_get_curr_thread_env() );
		iRaiseCnt = arg.iRaiseCnt;  // 在OnPollPreparePfn会被设置
	}

    {
        // 事件发生后
        // co_event_loop 调用prepare函数，即OnPollPreparePfn处理iRaiseCnt与把event写到pollfd
        //               然后调用process函数切回来这个协程

		//clear epoll status and memory
        // 把stPoll_t从timout link移出
		RemoveFromLink<stTimeoutItem_t, stTimeoutItemLink_t>( &arg );

		for(nfds_t i = 0;i < nfds;i++)
		{
			int fd = fds[i].fd;
			if( fd > -1 )
			{
                // 把items的事件监控全部删掉，符合poll接口语义: 一次poll有事件就可以返回了
				co_epoll_ctl( epfd, EPOLL_CTL_DEL, fd,&arg.pPollItems[i].stEvent );
			}
            // 把poll item的事件copy到结果返回
			fds[i].revents = arg.fds[i].revents;
		}

		if( arg.pPollItems != arr )
		{
			free( arg.pPollItems );
			arg.pPollItems = NULL;
		}

		free(arg.fds);
		free(&arg);
	}

	return iRaiseCnt;
}

int	co_poll( stCoEpoll_t *ctx, struct pollfd fds[], nfds_t nfds, int timeout_ms )
{
	return co_poll_inner(ctx, fds, nfds, timeout_ms, NULL);
}

/*
 * 把协程环境Epoll赋给一个协程环境数据结构
 * input: stCoEpoll_t
 * output: stCoRoutineEnv_t
 */
void SetEpoll( stCoRoutineEnv_t *env, stCoEpoll_t *ev )
{
	env->pEpoll = ev;
}

stCoEpoll_t* co_get_epoll_ct()
{
	if( !co_get_curr_thread_env() )
	{
		co_init_curr_thread_env();
	}
	return co_get_curr_thread_env()->pEpoll;
}

struct stHookPThreadSpec_t
{
	stCoRoutine_t *co;
	void *value;

	enum 
	{
		size = 1024
	};
};

void *co_getspecific(pthread_key_t key)
{
	stCoRoutine_t *co = GetCurrThreadCo();
	if( !co || co->cIsMain )
	{
		return pthread_getspecific( key );
	}
	return co->aSpec[ key ].value;
}

int co_setspecific(pthread_key_t key, const void *value)
{
	stCoRoutine_t *co = GetCurrThreadCo();
	if( !co || co->cIsMain )
	{
		return pthread_setspecific( key, value );
	}
	co->aSpec[ key ].value = (void*)value;
	return 0;
}

void co_disable_hook_sys()
{
	stCoRoutine_t *co = GetCurrThreadCo();
	if( co )
	{
		co->cEnableSysHook = 0;
	}
}

bool co_is_enable_sys_hook()
{
	stCoRoutine_t *co = GetCurrThreadCo();
	return ( co && co->cEnableSysHook );
}

stCoRoutine_t *co_self()
{
	return GetCurrThreadCo();
}

//co cond
struct stCoCond_t;

struct stCoCondItem_t 
{
	stCoCondItem_t *pPrev;
	stCoCondItem_t *pNext;
	stCoCond_t *pLink;

	stTimeoutItem_t timeout;
};

struct stCoCond_t
{
	stCoCondItem_t *head;
	stCoCondItem_t *tail;
};

static void OnSignalProcessEvent( stTimeoutItem_t * ap )
{
	stCoRoutine_t *co = (stCoRoutine_t*)ap->pArg;
	co_resume( co );
}

stCoCondItem_t *co_cond_pop( stCoCond_t *link );

/*
 * 从link取出head的cond，并把cond timeout放到epoll的active list
 */
int co_cond_signal( stCoCond_t *si )
{
    // 从link取出head的cond
	stCoCondItem_t * sp = co_cond_pop( si );
	if( !sp ) 
	{
		return 0;
	}

    // 由于cond已经被触发了所以cond内用于超时的timeout要从timeout link中remove
	RemoveFromLink<stTimeoutItem_t, stTimeoutItemLink_t>( &sp->timeout );

    // 并把cond timeout放到epoll的active list
	AddTail( co_get_curr_thread_env()->pEpoll->pstActiveList, &sp->timeout );

	return 0;
}

int co_cond_broadcast( stCoCond_t *si )
{
	for(;;)
	{
		stCoCondItem_t * sp = co_cond_pop( si );
		if( !sp ) return 0;

		RemoveFromLink<stTimeoutItem_t, stTimeoutItemLink_t>( &sp->timeout );

		AddTail( co_get_curr_thread_env()->pEpoll->pstActiveList,&sp->timeout );
	}

	return 0;
}

/*
 * 生成一个cond并加到link里，超时时间为ms通过conditem的timeout item利用co_eventloop时间轮触发
 */
int co_cond_timedwait( stCoCond_t *link, int ms )
{
    // alloc
	stCoCondItem_t* psi = (stCoCondItem_t*)calloc(1, sizeof(stCoCondItem_t));

    // 设置回调函数: 继续运行当前co
	psi->timeout.pArg = GetCurrThreadCo();
	psi->timeout.pfnProcess = OnSignalProcessEvent;

    // 设置timeout，timeout就会从下面co_yield_ct继续运行
	if( ms > 0 )
	{
		unsigned long long now = GetTickMS();
		psi->timeout.ullExpireTime = now + ms;

		int ret = AddTimeout( co_get_curr_thread_env()->pEpoll->pTimeout, &psi->timeout, now );
		if( ret != 0 )
		{
			free(psi);
			return ret;
		}
	}

    // 把CondItem加到Cond的link里，被cond触发就会从下面co_yield_ct继续运行
	AddTail(link, psi);

    // 切出去
	co_yield_ct();
    // 超时/被cond触发, 切回来

	RemoveFromLink<stCoCondItem_t, stCoCond_t>( psi );
	free(psi);

	return 0;
}

stCoCond_t *co_cond_alloc()
{
	return (stCoCond_t*)calloc( 1, sizeof(stCoCond_t) );
}

int co_cond_free( stCoCond_t * cc )
{
	free( cc );
	return 0;
}

// 从link取出head的cond
stCoCondItem_t *co_cond_pop( stCoCond_t *link )
{
	stCoCondItem_t *p = link->head;
	if( p )
	{
		PopHead<stCoCondItem_t, stCoCond_t>( link );
	}
	return p;
}

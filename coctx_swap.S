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

.globl coctx_swap
#if !defined( __APPLE__ ) && !defined( __FreeBSD__ )
.type  coctx_swap, @function
#endif
coctx_swap:

#if defined(__i386__)
    //======================
    // 1. 调用这个函数时，栈的状态
    // 高地址
    // | pending_ctx ptr |
    // | current_ctx ptr | <- eax，按正常非协程调用，caller准备完参数，调用call指令以后，会push ret_addr然后ip指向新函数; 
    //                             callee运行完清完寄存器ret前，esp指向ret_addr，调用ret，会pop ret_addr且ip跳回原来的指令，
    //                             跳回caller以后，这个位置就是esp指向的位置，因此这个位置也是协程上下文切换时要保存的esp
    // | ret addr        | <- esp (刚进入这个函数时指向这里)
    // 低地址

    // 希望这个函数返回后的状态：
    // 高地址
    // | pending_ctx ptr |
    // | current_ctx ptr | <- esp
    // 低地址
    //======================


    // 2. 把esp指向current_ctx内部
    // eax不会需要先保存吗？为什么？ =>  By convention, registers %eax, %edx, and %ecx are classified as caller-save registers. <<CSAPP>>
	leal 4(%esp), %eax      // eax = 4 + esp，见上面标记的位置
	movl 4(%esp), %esp      // esp = Memory(4 + esp) = current_ctx
	leal 32(%esp), %esp     // esp = current_ctx的regs[8] + sizeof(*)，为后面push各寄存器状态做准备，push的时候esp=esp-4再赋内容，所以这里偏移了sizeof(void*)


    // 3. 保存当前寄存器进current_ctx 
    //    如果是第一次运行，current_ctx就是main，首次初始化，且保存的esp指向最初的进程栈
    //    如果是current_ctx是其他协程，保存的esp指向对应的协程栈)
	pushl %eax     // eax保存着1提到的，返回后希望esp指向的位置
	pushl %ebp
	pushl %esi
	pushl %edi
	pushl %edx
	pushl %ecx
	pushl %ebx
	pushl -4(%eax)
    // current coctx_t:
    // 低地址
    // | regs[0]: ret                     | <- esp (push后)
    // | regs[1]: ebx                     |
    // | regs[2]: ecx                     |
    // | regs[3]: edx                     |
    // | regs[4]: edi                     |
    // | regs[5]: esi                     |
    // | regs[6]: ebp                     |
    // | regs[7]: eax(即1里面返回后的esp) |
    // | ss_size                          | <- esp (push前)
    // | ss_sp                            |
    // 高地址


    // 4. 把esp指向pending_ctx内部, esp = &pending_ctx
	movl 4(%eax), %esp


    // 5. 从pending_ctx恢复寄存器状态
    //    如果是此协程第一次运行，co.ctx->regs[kESP]指向co自身栈底，co.ctx->regs[kEip]指向pfn，其他为0(coctx_make)
    //    如果是pending_ctx曾经运行过，那么就是取出前面push的状态

    // 协程栈
    // 高地址
    // | param->s2: s |
    // | param->s1: s1|
    // |              |  <- ctx->regs[kESP]
    // 低地址

    // pending coctx_t:
    // 低地址
    // | regs[0]: ret      | <- esp (pop前)
    // | regs[1]: ebx      |
    // | regs[2]: ecx      |
    // | regs[3]: edx      |
    // | regs[4]: edi      |
    // | regs[5]: esi      |
    // | regs[6]: ebp      |
    // | regs[7]: eax(esp) |
    // | ss_size           |
    // | ss_sp             |
    // 高地址
	popl %eax  // ret func addr, eax = ret_addr, 对于首次运行的co来说存的是pfn
	popl %ebx  
	popl %ecx
	popl %edx
	popl %edi
	popl %esi
	popl %ebp
	popl %esp  // 对于首次运行的co来说是上面的没有存数据的位置，对于main来说是前面讨论的最初的进程栈的位置


    // 6. 设置ret addr到栈顶，为ret跳转做准备
	pushl %eax // set ret func addr    // 高地址
    // | param->s2: s            |
    // | param->s1: s1           |
    // |                         |
    // | ret addr of pending_ctx | <- esp
    // 低地址

    // 7. ret
	xorl %eax, %eax  // clear eax
	ret              // pop && eip = ret_addr

#elif defined(__x86_64__)
	leaq 8(%rsp),%rax
	leaq 112(%rdi),%rsp
	pushq %rax
	pushq %rbx
	pushq %rcx
	pushq %rdx

	pushq -8(%rax) //ret func addr

	pushq %rsi
	pushq %rdi
	pushq %rbp
	pushq %r8
	pushq %r9
	pushq %r12
	pushq %r13
	pushq %r14
	pushq %r15
	
	movq %rsi, %rsp
	popq %r15
	popq %r14
	popq %r13
	popq %r12
	popq %r9
	popq %r8
	popq %rbp
	popq %rdi
	popq %rsi
	popq %rax //ret func addr
	popq %rdx
	popq %rcx
	popq %rbx
	popq %rsp
	pushq %rax
	
	xorl %eax, %eax
	ret
#endif

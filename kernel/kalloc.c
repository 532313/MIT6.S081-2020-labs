// 物理内存分配器，用于用户进程、内核栈、页表页和管道缓冲区
// 分配完整的4096字节页
#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

// 释放指定范围内的物理内存页
void freerange(void *pa_start, void *pa_end);

// 内核结束后的第一个地址，由kernel.ld链接脚本定义
extern char end[];

// 内存页链表节点结构，用于组织空闲内存页
struct run
{
    struct run *next; // 指向下一个空闲页节点
};

// 为每个CPU维护的内存管理结构数组
// 每个元素包含该CPU的专属锁和空闲页列表
struct
{
    struct spinlock lock; // 保护当前CPU空闲列表的自旋锁
    struct run *freelist; // 空闲页链表的头指针
} kmem[NCPU];             // NCPU是系统支持的最大CPU数量

// 每个CPU对应的锁名称数组，均以"kmem"开头（符合实验要求）
char *lockname[] = {
    "kmem_cpu_0", // CPU0的锁名称
    "kmem_cpu_1", // CPU1的锁名称
    "kmem_cpu_2", // CPU2的锁名称
    "kmem_cpu_3", // CPU3的锁名称
    "kmem_cpu_4", // CPU4的锁名称
    "kmem_cpu_5", // CPU5的锁名称
    "kmem_cpu_6", // CPU6的锁名称
    "kmem_cpu_7", // CPU7的锁名称
};

// 初始化内存分配器
void kinit()
{
    // 初始化每个CPU的锁
    for (int i = 0; i < NCPU; ++i)
    {
        // 为每个CPU的锁指定名称并初始化
        initlock(&kmem[i].lock, lockname[i]);
    }
    // 释放从内核结束地址到物理内存上限的所有内存页
    freerange(end, (void *)PHYSTOP);
}

// 释放[pa_start, pa_end)范围内的所有物理内存页
void freerange(void *pa_start, void *pa_end)
{
    char *p;
    // 将起始地址向上对齐到页边界（确保分配完整页）
    p = (char *)PGROUNDUP((uint64)pa_start);
    // 遍历范围内的所有页，逐个释放
    for (; p + PGSIZE <= (char *)pa_end; p += PGSIZE)
        kfree(p);
}

// 释放pa指向的物理内存页
// 该页通常是之前通过kalloc()分配的（初始化时除外，见kinit）
void kfree(void *pa)
{
    struct run *r;

    // 合法性检查：确保释放的是有效内存页
    // 1. 地址必须按页大小(PGSIZE)对齐
    // 2. 地址必须在内核结束地址(end)之后
    // 3. 地址必须在物理内存上限(PHYSTOP)之前
    if (((uint64)pa % PGSIZE) != 0 || (char *)pa < end || (uint64)pa >= PHYSTOP)
        panic("kfree"); // 检查失败则触发内核崩溃

    // 填充垃圾数据以检测悬空引用（使用已释放内存的错误）
    memset(pa, 1, PGSIZE);

    // 将物理地址转换为空闲页链表节点
    r = (struct run *)pa;

    // 关闭当前CPU的中断，确保后续操作安全
    // 原因：cpuid()必须在中断关闭时调用，避免CPU切换导致ID错误
    push_off();

    // 获取当前CPU的编号
    int cpu = cpuid();
    // 获取当前CPU的内存锁，独占访问空闲列表
    acquire(&kmem[cpu].lock);
    // 将释放的页插入到空闲列表头部（头插法）
    r->next = kmem[cpu].freelist; // 新节点指向原列表头部
    kmem[cpu].freelist = r;       // 列表头部更新为新节点
    // 释放当前CPU的内存锁
    release(&kmem[cpu].lock);

    // 恢复中断状态
    pop_off();
}

// 分配一个4096字节的物理内存页
// 返回内核可用的指针，分配失败则返回0
void *
kalloc(void)
{
    // 关闭当前CPU的中断，确保获取CPU编号和操作列表时安全
    push_off();

    // 获取当前CPU的编号
    int cpu = cpuid();
    // 用于存储要分配的空闲页节点
    struct run *r;

    // 获取当前CPU的内存锁，准备访问本地空闲列表
    acquire(&kmem[cpu].lock);
    // 尝试从本地空闲列表获取页
    r = kmem[cpu].freelist;
    if (r)
        // 若获取成功，更新空闲列表（移除已分配的节点）
        kmem[cpu].freelist = r->next;
    else
    { ////////////////////////////////////////////////////////////////////////////////////
        // 本地空闲列表为空，尝试从其他CPU窃取一页
        int antid; // 其他CPU的编号
        // 遍历所有CPU
        for (antid = 0; antid < NCPU; ++antid)
        {
            if (antid == cpu) // 跳过当前CPU
                continue;
            // 获取目标CPU的内存锁
            acquire(&kmem[antid].lock);
            // 尝试从目标CPU的空闲列表获取页
            r = kmem[antid].freelist;
            if (r)
            {
                // 若获取成功，更新目标CPU的空闲列表（移除被窃取的节点）
                kmem[antid].freelist = r->next;
                // 释放目标CPU的内存锁
                release(&kmem[antid].lock);
                // 成功窃取一页，退出循环
                break;
            }
            // 目标CPU也无空闲页，释放其内存锁
            release(&kmem[antid].lock);
        }
    } ////////////////////////////////////////////////////////////////////////////////////////
    // 释放当前CPU的内存锁
    release(&kmem[cpu].lock);
    // 恢复中断状态
    pop_off();

    // 若分配到页，填充垃圾数据以避免使用未初始化内存
    if (r)
        memset((char *)r, 5, PGSIZE); // 填充0x55作为标记

    return (void *)r; // 返回分配的页地址（失败时返回NULL）
}

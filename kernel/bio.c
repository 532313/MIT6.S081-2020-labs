// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKET 13
#define HASH(id) (id % NBUCKET)


struct hashbuf
{
    struct buf head;      // 头节点
    struct spinlock lock; // 锁
};

struct
{
    struct buf buf[NBUF];
    struct hashbuf buckets[NBUCKET]; // 散列桶
} bcache;

void binit(void)
{
    struct buf *b;
    char lockname[16]; // 用于生成桶锁的名称

    // 初始化所有哈希桶的锁和链表
    for (int i = 0; i < NBUCKET; ++i)
    {
        // 初始化散列桶的自旋锁（名称格式：bcache_0, bcache_1...）
        snprintf(lockname, sizeof(lockname), "bcache_%d", i);
        initlock(&bcache.buckets[i].lock, lockname); // 初始化锁，名称符合"bcache"开头要求

        // 初始化散列桶的头节点（双向链表为空时，头节点的prev和next都指向自身）
        bcache.buckets[i].head.prev = &bcache.buckets[i].head;
        bcache.buckets[i].head.next = &bcache.buckets[i].head;
    }

    // 将所有缓冲区初始挂载到第0个桶（初始状态集中管理，后续按需迁移）
    for (b = bcache.buf; b < bcache.buf + NBUF; b++)
    {
        // 头插法插入第0个桶的链表（新节点插在head和原第一个节点之间）
        b->next = bcache.buckets[0].head.next; // 新节点的next指向原head的next
        b->prev = &bcache.buckets[0].head;     // 新节点的prev指向head
        initsleeplock(&b->lock, "buffer");     // 初始化缓冲区自身的睡眠锁（保护数据读写）
        bcache.buckets[0].head.next->prev = b; // 原第一个节点的prev指向新节点
        bcache.buckets[0].head.next = b;       // head的next指向新节点（完成插入）
    }
}

static struct buf *bget(uint dev, uint blockno)
{
    struct buf *b; // 用于存储找到的缓冲区

    // 计算目标块（dev, blockno）应归属的桶
    int bid = HASH(blockno);
    // 锁定目标桶（确保查找和后续分配的原子性）
    acquire(&bcache.buckets[bid].lock);

    // 第一步：检查目标块是否已在当前桶中缓存
    for (b = bcache.buckets[bid].head.next; b != &bcache.buckets[bid].head; b = b->next)
    {
        // 匹配条件：设备号和块号都相同（同一磁盘块）
        if (b->dev == dev && b->blockno == blockno)
        {
            // 找到缓存，增加引用计数（标记为“正在被使用”）
            b->refcnt++;

            // 更新时间戳为当前ticks（标记为“最近使用”）
            acquire(&tickslock);
            b->timestamp = ticks;
            release(&tickslock);

            // 释放目标桶的锁（已完成查找）
            release(&bcache.buckets[bid].lock);
            // 获取缓冲区的睡眠锁（返回前确保调用者独占访问数据）
            acquiresleep(&b->lock);
            return b; // 返回找到的缓冲区
        }
    }

    // 第二步：未在目标桶中找到，需要分配一个空闲缓冲区（LRU策略）
    b = 0;           // 用于记录找到的空闲缓冲区
    struct buf *tmp; // 临时变量，遍历桶内缓冲区

    // 从当前桶开始，依次遍历所有桶（避免死锁的顺序：按桶索引递增）
    for (int i = bid, cycle = 0; cycle != NBUCKET; i = (i + 1) % NBUCKET)
    {
        cycle++; // 记录已遍历的桶数量（避免无限循环）

        // 若遍历到非目标桶，需要获取该桶的锁（已持有目标桶的锁，按顺序获取其他桶的锁）
        if (i != bid)
        {
            // 检查是否已持有该桶的锁（避免重复获取导致死锁）
            if (!holding(&bcache.buckets[i].lock))
                acquire(&bcache.buckets[i].lock); // 未持有则获取
            else
                continue; // 已持有则跳过（可能是循环到已处理的桶）
        }

        // 遍历当前桶内的所有缓冲区，查找空闲（refcnt=0）且时间戳最小（最久未使用）的
        for (tmp = bcache.buckets[i].head.next; tmp != &bcache.buckets[i].head; tmp = tmp->next)
        {
            // 条件：空闲（refcnt=0），且是目前找到的最久未使用（timestamp最小）
            if (tmp->refcnt == 0 && (b == 0 || tmp->timestamp < b->timestamp))
                b = tmp; // 记录该缓冲区
        }

        // 若找到符合条件的空闲缓冲区
        if (b)
        {
            // 若缓冲区来自其他桶（i != bid），需要迁移到目标桶
            if (i != bid)
            {
                // 从原桶（i）中移除该缓冲区（双向链表操作）
                b->next->prev = b->prev;          // 原next节点的prev指向原prev节点
                b->prev->next = b->next;          // 原prev节点的next指向原next节点
                release(&bcache.buckets[i].lock); // 释放原桶的锁（已完成移除）

                // 将缓冲区插入目标桶（bid）的头部（头插法）
                b->next = bcache.buckets[bid].head.next; // 新next指向目标桶原第一个节点
                b->prev = &bcache.buckets[bid].head;     // 新prev指向目标桶head
                bcache.buckets[bid].head.next->prev = b; // 目标桶原第一个节点的prev指向b
                bcache.buckets[bid].head.next = b;       // 目标桶head的next指向b（完成插入）
            }

            // 初始化缓冲区元数据（指向新的磁盘块）
            b->dev = dev;         // 设备号
            b->blockno = blockno; // 块号
            b->valid = 0;         // 标记为无效（需要从磁盘读取数据）
            b->refcnt = 1;        // 引用计数设为1（当前进程正在使用）

            // 更新时间戳为当前ticks（标记为“最近使用”）
            acquire(&tickslock);
            b->timestamp = ticks;
            release(&tickslock);

            // 释放目标桶的锁（分配完成）
            release(&bcache.buckets[bid].lock);
            // 获取缓冲区的睡眠锁（确保调用者独占访问）
            acquiresleep(&b->lock);
            return b; // 返回分配的缓冲区
        }
        else
        {
            // 未在当前桶找到空闲缓冲区，释放该桶的锁（继续遍历下一个）
            if (i != bid)
                release(&bcache.buckets[i].lock);
        }
    }

    // 所有桶都遍历完仍未找到空闲缓冲区（缓存满）
    panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

void brelse(struct buf *b)
{
    // 检查是否持有缓冲区的睡眠锁（释放前必须持有，否则panic）
    if (!holdingsleep(&b->lock))
        panic("brelse");

    // 计算当前缓冲区所属的桶（按blockno哈希）
    int bid = HASH(b->blockno);

    // 释放缓冲区的睡眠锁（允许其他进程获取该缓冲区的数据访问权）
    releasesleep(&b->lock);

    // 锁定所属的桶（保护refcnt和timestamp的修改）
    acquire(&bcache.buckets[bid].lock);
    // 减少引用计数（表示当前进程不再使用该缓冲区）
    b->refcnt--;

    // 更新时间戳为当前ticks（标记为“最近使用”，用于LRU判断）
    acquire(&tickslock); // 锁定ticks（系统时钟可能被多CPU修改）
    b->timestamp = ticks;
    release(&tickslock);

    // 释放桶锁（完成元数据修改）
    release(&bcache.buckets[bid].lock);
}

void bpin(struct buf *b)
{
    // 计算缓冲区所属的桶
    int bid = HASH(b->blockno);
    // 锁定该桶（保护refcnt修改）
    acquire(&bcache.buckets[bid].lock);
    // 增加引用计数（表示有进程需要长期持有该缓冲区）
    b->refcnt++;
    // 释放桶锁
    release(&bcache.buckets[bid].lock);
}
void bunpin(struct buf *b)
{
    // 计算缓冲区所属的桶
    int bid = HASH(b->blockno);
    // 锁定该桶（保护refcnt修改）
    acquire(&bcache.buckets[bid].lock);
    // 减少引用计数
    b->refcnt--;
    // 释放桶锁
    release(&bcache.buckets[bid].lock);
}
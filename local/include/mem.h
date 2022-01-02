#if !defined(MEM_H)
#define MEM_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <stdint.h>
#include <pthread.h>


// (mostly) thread-safe library for allocating and recycling memory 
//    blocks of specific sizes
// allocated memory is on cache-line boundaries (ie, 64-byte)
// a pre- and post- buffer is added to each allocation to help detect
//    memory under/overflow
//
// NOTE: if memory is allocated using the default pool, it must be 
//    freed in the thread that it was created, as there's a different
//    default pool for each thread
//    
// NOTE: this is not efficient for small allocations


// boundary code based on assumption that cache line is 64 bytes
// if not, either disable check (optimization issue) or fix code
//    so it supports correct cache size
#if defined(INTEL)
#define CACHE_LINE_SIZE    64
#elif defined(ARM)
#define CACHE_LINE_SIZE    32
#endif   // INTEL | ARM


// returns cache line size for this CPU
uint32_t get_cache_line_size(void);


// boundry is 16 bytes
// boundry is length of cache line. 
struct alloc_boundary {
   uint32_t  size;      // number of bytes allocated
   uint32_t  in_use;
   uint32_t  null_val[14];
};

struct memory_pool {
   struct alloc_boundary **cache;
   void **allocation_list;
   uint32_t avail;
   uint32_t cap;
   uint32_t num_allocations;
   uint32_t allocation_cap;
   pthread_mutex_t  mutex;
};
typedef struct memory_pool memory_pool_type;

struct memory_pool * create_memory_pool(void);

// destruction is assumed to be handled by a single thread and no
//    additional operations on a memory pool by other threads.
//    behavior is undefined if one thread operates on memory while
//    another thread destroys the pool
void delete_memory_pool(struct memory_pool *pool);
void delete_memory_pool_quietly(struct memory_pool *pool);

// this is an advisory function giving the approx # of blocks that are
//    cached. as it's only an approximation, it's not been made thread
//    safe
uint32_t num_unfreed_blocks(struct memory_pool *pool);

// thread safe -- allocate and recycle blocks of memory
void * cache_malloc(struct memory_pool *pool, size_t num_bytes);
int32_t cache_free(struct memory_pool *pool, void *mem);

// thread-specific general memory pool
struct memory_pool * get_default_memory_pool(void);

void debug_print_boundaries(
      /* in     */ const void *cache_mem);

#endif   // MEM_H


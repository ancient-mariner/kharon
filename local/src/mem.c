#include "pinet.h"
#include "mem.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#if defined(TEST_MEM)
#define NDEBUG
#endif   // TEST_MEM
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#if defined(INTEL)
#define  PLATFORM_64 1
#elif defined(RPI)
#define  PLATFORM_32 1
#elif defined(RPI4)
#define  PLATFORM_64 1
#endif   // INTEL | RPI

// TODO make sure alignments are always to cache line boundaries

// pad buffer header and tail to help detect buffer overflow errors
// use arbitrary values
#define NULL_0_PATTERN 0x01230123
#define NULL_1_PATTERN 0x12341234
#define NULL_2_PATTERN 0x23452345
#define NULL_3_PATTERN 0x34563456
#define NULL_4_PATTERN 0x45674567
#define NULL_5_PATTERN 0x56785678
#define NULL_6_PATTERN 0x67896789
#define NULL_7_PATTERN 0x789a789a
#define NULL_8_PATTERN 0x89ab89ab
#define NULL_9_PATTERN 0x9abc9abc
#define NULL_10_PATTERN 0xabcdabcd
#define NULL_11_PATTERN 0xbcdebcde
#define NULL_12_PATTERN 0xcdefcdef
#define NULL_13_PATTERN 0xdef0def0
#define NULL_14_PATTERN 0xef01ef01
#define NULL_15_PATTERN 0xf012f012

// cache line size varies between platforms. it was used as the size
//    for the "fence" around allocated memory, so make that explicit
#define ALLOC_FENCE_SIZE   64

//uint32_t get_cache_line_size()
//{  
//#if defined(INTEL)
//   return (uint32_t) sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
//#elif defined(RPI)
//   // this code revised to handle case where cache line info not available
//   // once it is available, logic needs to be revisited
//   assert(sysconf(_SC_LEVEL1_DCACHE_LINESIZE) == 0);
//   return 32u; // unclear if it's 32 or 64 bytes, so go w/ the latter
//#endif   // INTEL | RPI
//} 


// returns number rounded up to next multiple of 64
static size_t next_64(size_t num_bytes)
{
   return (0xffffffc0 & (num_bytes + 63));
}


////////////////////////////////////////////////////////////////////////

static __thread struct memory_pool * s_pool = NULL;

struct memory_pool *get_default_memory_pool()
{
   if (s_pool == NULL)
      s_pool = create_memory_pool();
   return s_pool;
}

// prints out values for header and footer zones for specified memory
// (memory assumed to be allocated from cache)
void debug_print_boundaries(
      /* in     */ const void *cache_mem)
{
   const uint32_t line = ALLOC_FENCE_SIZE;
   const uint32_t NUM_INT32 = line / sizeof(uint32_t);
   //
   const uint8_t *base = (const uint8_t *) cache_mem;
   const uint8_t *head = base - line;
   const uint32_t *head32 = (const uint32_t *) head;
   printf("Header\n------\n");
   for (uint32_t i=0; i<NUM_INT32; i+=2) {
      printf("  0x%012lx\t0x%08x\t0x%08x\n", 
            (long unsigned int) &head32[i], head32[i], head32[i+1]);
   }
   printf("Trailer\n-------\n");
   const uint32_t jump = (uint32_t) next_64(head32[0]);
   const uint8_t *foot = base + jump;
   const uint32_t *foot32 = (const uint32_t *) foot;
   for (uint32_t i=0; i<NUM_INT32; i+=2) {
      printf("  0x%012lx\t0x%08x\t0x%08x\n", 
            (long unsigned int) &foot32[i], foot32[i], foot32[i+1]);
      //printf("%08x  \t0x%08x\t\t0x%08x\n", &foot[i], foot[i], foot[i+1]);
   }
}

// fingerprint is hopefully unique pattern that is copied into
//    space immediately before and after allocation. establishes
//    a boundary to detect over- and under-write errors
static const uint32_t FINGERPRINT[ALLOC_FENCE_SIZE/4] = { 
   NULL_0_PATTERN, NULL_1_PATTERN, NULL_2_PATTERN, NULL_3_PATTERN,
   NULL_4_PATTERN, NULL_5_PATTERN, NULL_6_PATTERN, NULL_7_PATTERN,
   NULL_8_PATTERN, NULL_9_PATTERN, NULL_10_PATTERN, NULL_11_PATTERN,
   NULL_12_PATTERN, NULL_13_PATTERN, NULL_14_PATTERN, NULL_15_PATTERN
};

// creates new memory pool
struct memory_pool *create_memory_pool()
{
   if (sizeof(FINGERPRINT) != ALLOC_FENCE_SIZE) {
      fprintf(stderr, "Internal error -- mem.c based on allocations being "
            "on cache-line boundaries\n");
      exit(1);
   }
   struct memory_pool *pool;
   pool = malloc(sizeof(struct memory_pool));
   uint32_t sz = 32;
   pool->cap = sz;
   pool->cache = malloc(sz * sizeof(struct alloc_boundary*));
   pool->avail = 0;
   pool->num_allocations = 0;
   pool->allocation_cap = sz;
   pool->allocation_list = malloc(sz * sizeof(void*));
   pthread_mutex_init(&pool->mutex, NULL);
   return pool;
}

void delete_memory_pool_quietly(struct memory_pool *pool)
{
   // free all allocations
   for (uint32_t i=0; i<pool->num_allocations; i++) {
      free(pool->allocation_list[i]);
   }
   // free administrative storage
   free(pool->allocation_list);
   free(pool->cache);
   pthread_mutex_destroy(&pool->mutex);
   free(pool);
}

void delete_memory_pool(struct memory_pool *pool)
{
   uint32_t n = num_unfreed_blocks(pool);
   if (n != 0) {
      fprintf(stderr, "WARNING: deleting memory pool with %d outstanding allocation%s.\n", n, n==1?"":"s");
   }
   delete_memory_pool_quietly(pool);
}

// initializes header and footer to expected values, and body to NULL
static void initialize_block(void * raw, uint32_t size)
{
   // get pointers to for and aft headers, and central body, then set content
   struct alloc_boundary *pre = (struct alloc_boundary*) raw;
   assert(pre != NULL);
   struct alloc_boundary *center = &pre[1];
   uint8_t *center8 = (uint8_t *) center;
   uint8_t *post8 = center8 + size;
   struct alloc_boundary *post = (struct alloc_boundary *) post8;
   // compilation of unit test indicates that 'pre' can be NULL. this
   //    isn't terribly important as test will crash if it is, and it
   //    can be investigated. get rid of compiler warnings by adding
   //    an explicit check when in test mode
   assert(pre != NULL);
#if defined(TEST_MEM)
   if (pre != NULL) {
#endif   // TEST_MEM
   memcpy(pre, &FINGERPRINT, sizeof(struct alloc_boundary));
   pre->size = size;
   pre->in_use = 0;
#if defined(TEST_MEM)
   }
#endif   // TEST_MEM
   size_t sz_64 = next_64(size);
   memset(center, 0, sz_64);
   memcpy(post, &FINGERPRINT, sizeof(struct alloc_boundary));
}

// internal function -- allocates block of memory
// memory immediately before and after returned block is initialized
//    to 'fingerprint' value to detect over and underflow errors
static void * cache_malloc_block(struct memory_pool *pool, size_t num_bytes)
{
   // round num_bytes up to next multiple of 16 (default to SSE-friendly)
   // add 32 bytes of padding (16 pre and 16 post)
   size_t sz_64 = next_64(num_bytes);
   size_t sz = sz_64 + 2 * ALLOC_FENCE_SIZE;
   // allocate memory and initialize boundaries
   // gcc automatically allocates on 16-byte boundaries on 64-bit, 
   //    but only 8-byte on 32-bit. to maintain 32-bit support
   //    but that's irrelevant if we're allocating to cache-line boundaries
   uint8_t * mem = NULL;
   errno = 0;
   posix_memalign((void**) &mem, ALLOC_FENCE_SIZE, sz);
   if ((mem == NULL) || (errno != 0)) {
      fprintf(stderr, "Failed to allocate memory (%s)\n", strerror(errno));
      hard_exit(__FILE__, __LINE__);
   }
   initialize_block(mem, (uint32_t) sz_64);
   // store a record of this allocation
   pthread_mutex_lock(&pool->mutex); // lock access to pool first
   if (pool->num_allocations >= pool->allocation_cap) {
      pool->allocation_cap *= 2;
      pool->allocation_list = (void**) realloc(pool->allocation_list, 
            pool->allocation_cap * sizeof(void*));
   }
   pool->allocation_list[pool->num_allocations++] = mem;
   pthread_mutex_unlock(&pool->mutex);  // changes made -- unlock
   // done
   return mem;
}

// NOTE: this is an advisory function that is not thread safe
uint32_t num_unfreed_blocks(struct memory_pool *pool)
{
   return pool->num_allocations - pool->avail;
}

void * cache_malloc(struct memory_pool *pool, size_t num_bytes)
{
   // look for pre-allocated block of this size. if it exists, return it.
   // otherwise create new
   size_t sz_64 = next_64(num_bytes);
   struct alloc_boundary *blk = NULL;
   // lock pool while searching its data structure
   pthread_mutex_lock(&pool->mutex);
   for (uint32_t i=0; i<pool->avail; i++) {
      if (pool->cache[i]->size == sz_64) {
         blk = pool->cache[i];
         if (i < pool->avail-1) {
            pool->cache[i] = pool->cache[pool->avail-1];
         }
         pool->avail--;
         break;
      }
   }
   pthread_mutex_unlock(&pool->mutex);
   if (blk == NULL) {
      blk = cache_malloc_block(pool, sz_64);
   }
   blk->in_use = 1;
   return &blk[1];
   //return &((uint8_t*) blk)[16];
}

int32_t cache_free(struct memory_pool *pool, void *mem)
{
   // make sure pointer is to cache block
   struct alloc_boundary *center = (struct alloc_boundary*) mem;
   struct alloc_boundary *pre = &center[-1];
   uint8_t *center8 = (uint8_t *) center;
   uint8_t *post8 = center8 + pre->size;
   struct alloc_boundary *post = (struct alloc_boundary *) post8;
   // compare unused area of header to fingerprint to catch
   //    underflow error
   if (memcmp(&pre->null_val[0], &FINGERPRINT[2], ALLOC_FENCE_SIZE-8) != 0) {
   //if ((pre->null_2 != NULL_2_PATTERN) || (pre->null_3 != NULL_3_PATTERN)) {
      fprintf(stderr, "Freeing object not recognized as being from cache\n");
      fprintf(stderr, "Possible buffer underwrite problem\n");
      debug_print_boundaries(mem);
      assert(1 == 0);
      goto err;
   }
   if (pre->in_use == 0) {
      fprintf(stderr, "Double-deletion of object, or memory corruption\n");
      debug_print_boundaries(mem);
      assert(1 == 0);
      goto err;
   }
   // check for fingerprint value immediately after allocated space
   if (memcmp(post, &FINGERPRINT, ALLOC_FENCE_SIZE) != 0) {
      fprintf(stderr, "Freeing object not recognized as being from cache\n");
      fprintf(stderr, "Possible buffer overwrite problem\n");
      debug_print_boundaries(mem);
      assert(1 == 0);
      goto err;
   }
   pre->in_use = 0;
   // return to cache
   pthread_mutex_lock(&pool->mutex); // protect modification of pool
   if (pool->avail >= pool->cap) {
      pool->cap *= 2;
      pool->cache = (struct alloc_boundary**) realloc(pool->cache, pool->cap);
   }
   pool->cache[pool->avail++] = pre;
   pthread_mutex_unlock(&pool->mutex);  // leave critical section
   return 0;
err:
   // this should be a hard error as memory corruptiong is a BAD thing
   // TODO make this a fatal error
   return -1;
}

#if defined(TEST_MEM)

uint32_t test_cache_line(void);
uint32_t test_alloc(void);
uint32_t test_under(void);
uint32_t test_over(void);
uint32_t test_layout(void);

uint32_t test_cache_line(void)
{
   printf("test_cacha_line()\n");
   uint32_t errs = 0;
   uint32_t sz = ALLOC_FENCE_SIZE;
   if (sz != 64) {
      printf("Expected malloc fence size of 64, found %d\n", sz);
      errs++;
   }
   return errs;
}

uint32_t test_alloc(void)
{
   uint32_t errs = 0;
   printf("test_alloc()\n");
   struct memory_pool *pool = get_default_memory_pool();
   const uint32_t CNT = 4;
   void * data[CNT];
   for (uint32_t j=0; j<10; j++) {
      for (uint32_t i=0; i<CNT; i++) {
         data[i] = cache_malloc(pool, 128);
#if defined(PLATFORM_64)
         if (((uint64_t) data[i]) & 0x0000003f) {
            fprintf(stderr, "Allocation %d on incorrect boundary\n", i);
            errs++;
         }
#elif defined(PLATFORM_32)
         if (((uint32_t) data[i]) & 0x003f) {
            fprintf(stderr, "Allocation %d on incorrect boundary\n", i);
            errs++;
         }
#else
#error "undefined platform"
#endif   // PLATFORM_64 | PLATFORM_32
      }
      for (uint32_t i=0; i<CNT; i++) {
         cache_free(pool, data[i]);
      }
   }
   if (pool->avail != CNT) {
      fprintf(stderr, "Freed %d but %d available\n", CNT, pool->avail);
      errs++;
   }
   return errs;
}

uint32_t test_under(void)
{
   uint32_t errs = 0;
   printf("test_under()\n");
   struct memory_pool *pool = get_default_memory_pool();
   uint32_t *data = (uint32_t *) cache_malloc(pool, 16 * sizeof(uint32_t));
   data[-1] = 0;
   if (cache_free(pool, data) == 0) {
      errs++;
      fprintf(stderr, "Error -- expected buffer underrun error "
            "didn't happen\n");
   }
   return errs;
}

uint32_t test_over(void)
{
   uint32_t errs = 0;
   printf("test_over()\n");
   struct memory_pool *pool = get_default_memory_pool();
   uint32_t *data = (uint32_t *) cache_malloc(pool, 16 * sizeof(uint32_t));
   data[16] = 0;
   if (cache_free(pool, data) == 0) {
      errs++;
      fprintf(stderr, "Error -- expected buffer overrun error "
            "didn't happen\n");
   }
   return errs;
}

uint32_t test_layout(void)
{
   uint32_t errs = 0;
   printf("test_layout()\n");
   const uint32_t NUM_BYTES = 104;
   const uint32_t NUM_BYTES_64 = 128;
   size_t sz64 = next_64(NUM_BYTES);
   if (sz64 != NUM_BYTES_64) {
      fprintf(stderr, "%d does not round up to %d\n", NUM_BYTES, NUM_BYTES_64);
      errs++;
   }
   if (next_64(sz64) != NUM_BYTES_64) {
      fprintf(stderr, "Internal error in next_64()\n");
      errs++;
   }
   struct memory_pool *pool = get_default_memory_pool();
   uint8_t *assigned = (uint8_t *) cache_malloc(pool, NUM_BYTES);
   uint8_t *data = (uint8_t *) assigned - ALLOC_FENCE_SIZE;
   uint32_t *data32 = (uint32_t *) data;
   // make sure header is properly formed
   // make sure body is null
   // make sure trailer is properly formed
   uint32_t expected;
   for (uint32_t i=0; i<16; i++) {
      if (i == 0) 
         expected = 128;
      else if (i == 1)
         expected = 1;
      else
         expected = FINGERPRINT[i];
      uint32_t val = data32[i];
      if (expected != val) {
         fprintf(stderr, "Header entry %d incorrect. Expected 0x%08x, "
               "found 0x%08x\n", i, expected, val);
         errs++;
      }
   }
   // body is all zero
   for (uint32_t i=0; i<sz64/4; i++) {
      expected = 0;
      uint32_t val = data32[i + 16];
      if (expected != val) {
         fprintf(stderr, "Body entry %d incorrect. Expected 0x%08x, "
               "found 0x%08x\n", i, expected, val);
         errs++;
      }
   }
   // trailer is all fingerprint
   for (uint32_t i=0; i<16; i++) {
      expected = FINGERPRINT[i];
      uint32_t val = data32[i + 16 + sz64/4];
      if (expected != val) {
         fprintf(stderr, "Trailer entry %d incorrect. Expected 0x%08x, "
               "found 0x%08x\n", i, expected, val);
         errs++;
      }
   }
//for (uint32_t i=0; i<NUM_INT32; i+=2) {
//   printf("%08x  \t0x%08x\t\t0x%08x\n", &data32[i], data32[i], data32[i+1]);
//}
   if (cache_free(pool, assigned) != 0) {
      errs++;
      fprintf(stderr, "Error -- Error freeing allocation to cache\n");
   }
   return errs;
}

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   uint32_t errs = 0;
   errs += test_alloc();
   errs += test_cache_line();
   errs += test_layout();
   errs += test_over();
   errs += test_under();
   //
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
   }
   return (int) errs;
}

#endif   // TEST_MEM

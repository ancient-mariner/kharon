#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "timekeeper.h"

#include "core_modules/support/frame_heap.h"

#define MAX_FRAME_HEAP_ALLOC        48
#define MAX_FRAME_HEAP_AVAILABLE   36

// linked list based memory management for self-compacting frame list
// image frames are stored with increasing inter-frame intervals for
//    older images. for example, the first several images are stored
//    sequentially, wherafter for the next several images every other
//    image is stored, then every 4th, then every 8th, etc.
// this logarithmic approach allows a long history to be stored relative 
//    to the memory footprint
//
// list alteration is done by a single process, while other processes
//    have read-only access to the list. the list is thread-safe for
//    reading processes
//
// a single process modifies the lists, adding and triming nodes. multiple
//    processes may read from the list while it's being modified. when
//    a process wants the frame list, the present head of the list is
//    provided to that process. it iterates through the frames in
//    the list at its own rate.
// when new elements are added, these are added to the head of the
//    list. because the consuming process already has a head, and as
//    the list is singly-linked, it doesn't know about the newer
//    head and it can continue to read the list. multiple processes
//    can read the same list this way, as none of the consumers modify
//    the list
// when elements are removed, the removal is an atomic operation (i.e.,
//    the .next field is updated to point to a new frame, which
//    occurs in a single cycle). if the .next page is read before the
//    change, the to-be-deleted page will be read and processed. if
//    .next is read after the change occurs, the subsequent page will be
//    read
// deleted pages are put into a queue where they remain for several
//    frame updates (e.g., 2 seconds). the deleted page's .next field 
//    will remain unchanged,
//    as will image data. this allows any process using a deleted
//    frame to continue processing it. when processing is completed,
//    it continues traversing the list, continuing with the .next
//    element
//
// page deletion occurs after every other page allocation. every
//    other allocation the 4th element in the list is deleted.
//    every 4th allocation, the 8th element is also deleted, etc.
// when the maximum page allocation occurs, the list is forcibly trimmed
//    and the last element is deleted, pushed into the queue. a
//    page allocation thus never fails
//
// the heap manages 2 lists -- one for the active frames, stored as
//    .frames, and a list for elements in the queue, managed through 
//    .available_head and .available_tail (and ._next in the frame 
//    page itself)
//    
//    


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


static frame_page_heap_type * create_heap(void)
{
   // memory management w/ system
   frame_page_heap_type *heap = malloc(sizeof *heap);
   heap->heap = calloc(MAX_FRAME_HEAP_ALLOC, sizeof *heap->heap);
   heap->available = MAX_FRAME_HEAP_AVAILABLE;
   // create linked list of pages in the pending queue
   heap->available_head = heap->heap;
   for (uint32_t i=0; i<MAX_FRAME_HEAP_ALLOC-1; i++) {
      heap->heap[i]._next = &heap->heap[i+1];
   }
   // deleted objects are pushed to the tail
   // allocated objects are pulled from the head
   heap->available_tail = &heap->available_head[MAX_FRAME_HEAP_ALLOC-1];
   heap->available_tail->content = MAX_FRAME_HEAP_ALLOC - 1;
   //
   heap->frames = NULL;
   //
   return heap;
}


static void free_page(frame_page_heap_type *heap, frame_page_type *page)
{
   heap->available_tail->_next = page;
   page->_next = NULL;
   heap->available_tail = page;
   // don't modify any of the payload content, (i.e., .next,
   //    .t and .frame)
   // increment number of available frames
   heap->available++;
}


static frame_page_type * allocate_page(frame_page_heap_type *heap)
{
   frame_page_type *page = NULL;
   if (heap->available == 0) {
//printf("%.3f PANORAMA force evict\n", now());
      // if no more pages are available, forcibly evict the
      //    last allocated page
      // find the last frame
      page = heap->frames;
      frame_page_type *prev = NULL;
      while (page->next) {
         prev = page;
         page = page->next;
      }
      if (c_assert(prev != NULL)) {
         log_err(get_kernel_log(), "Internal error allocating frame page");
         hard_exit(__func__, __LINE__);
      }
      // terminate the list
      prev->next = NULL;
      // send the page to the available queue
      free_page(heap, page);
   }
   // now there's content available. grab the next page from the head
   //    of the list. 
   if (heap->available > 0) {
      page = heap->available_head;
      heap->available_head = page->_next;
      heap->available--;
      page->_next = NULL;
      ////////////
      page->next = NULL;
      // TODO frame points to data that's allocated to this page.
      //    allocate that data
      page->t = 0.0;
      page->content = 0;
   }
   // don't add allocated page to the frame list yet. wait until
   //    the page is ready for consumption, and let the master
   //    process add it to the list using add_to_frames()
   // return the page to the calling process
   return page;
}

// 
static void add_to_frames(frame_page_heap_type *heap, frame_page_type *page)
{
   page->next = heap->frames;
   heap->frames = page;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


//void print_list(frame_page_heap_type *heap)
//{
//   uint32_t cnt = 0;
//   uint32_t start, stop;
//   frame_page_type *page = heap->frames;
//   while(page) {
//      if (cnt++ == 0)
//         start = page->content;
//      stop = page->content;
//      printf("%d ", page->content);
//      page = page->next;
//   }
//   printf("  (%d, %d. range: %d)\n", cnt, heap->available, start-stop);
//}


// delete the 4th element in the list and return the page that
//    was after it
static frame_page_type * delete_fourth(
      /* in out */       frame_page_heap_type *heap, 
      /* in out */       frame_page_type *page
      )
{
   if (page == NULL)
      return NULL;
   uint32_t ctr = 0;
   frame_page_type *prev = page;
   while (page) {
      if (++ctr == 4) {
         prev->next = page->next;
         free_page(heap, page);
         break;
      }
      prev = page;
      page = page->next;
   }
   return prev->next;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//
//int main(int argc, char **argv)
//{
//   frame_page_heap_type *heap = create_heap();
//   for (uint32_t i=0; i<4096; i++) {
//      {
//         frame_page_type *page = allocate_page(heap);
//         // do processing
//         page->content = i;
//         // add page to frame list
//         add_to_frames(heap, page);
//      }
//      {
//         // trim older elements of list
//         uint32_t val = i;
//         frame_page_type *head = heap->frames;
//         while (val & 1) {
//            head = delete_fourth(heap, head);
//            val >>= 1;
//         }
//      }
////printf("%d\n", i);
////      if (i > 4000)
////         print_list(heap);
//   }
//   return 0;
//}
//

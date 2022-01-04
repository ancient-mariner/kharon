/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#if !defined(FRAME_HEAP_H)
#define FRAME_HEAP_H

// for description of paging logic, see frame_heap.c

struct panorama_output;
struct pan_color_grid;

// data page, storing image frame and frame time
struct frame_page {
   // allocation control
   // _next is used by heap when it's unallocated (ie, in the pending queue)
   struct frame_page *_next;
   /////////////////////////////////////////////////////////////////////
   // payload
   struct frame_page *next;
   double t;
   // this is a pointer into panorama's void_queue
   struct panorama_output *frame;
//   // recent history of colors seen at each region of world view
//   // this is a pointer into panorama's color grid heap
//   struct pan_color_grid *color_grid;
   // for testing and debugging
   uint32_t content;
};
typedef struct frame_page frame_page_type;

// control structure for frame pages
struct frame_page_heap {
   // objects pulled off the head, pushed onto the tail
   frame_page_type *available_head;
   frame_page_type *available_tail;
   uint32_t available;
   // raw memory storage of frame pages
   // the allocation size is not modified during runtime, so no size
   //    information need be kept
   frame_page_type *heap;
   /////////////////////////////////////////////////////////////////////
   // list of allocated frames
   frame_page_type *frames;
};
typedef struct frame_page_heap frame_page_heap_type;


#endif   // FRAME_HEAP_H

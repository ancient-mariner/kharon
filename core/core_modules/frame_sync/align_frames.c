
// the goal here is to generate a set of camera frames that produces
//    a full panorama, or as close to that as possible. frames must
//    be captured w/in a certain window to be considered temporally
//    overlapping

////////////////////////////////////////////////////////////////////////
// memory management

static frame_node_type* allocate_frame_node(
      /* in out */       frame_sync_type* sync
      )
{
   if (++sync->num_allocated_nodes >= FRAME_NODE_HEAP_SIZE) {
      log_err(sync->log, "All frame nodes are allocated. This should "
            "be impossible");
      hard_exit(__FILE__, __LINE__);
   }
   frame_node_type* node = sync->frame_node_list_head;
   sync->frame_node_list_head = node->next;
   sync->frame_node_list_head->prev = NULL;
   node->next = NULL;
   node->prev = NULL;
   node->t = 0.0;
   node->frame = NULL;
   node->cam_num = 0;
   return node;
}


static void free_frame_node(
      /* in out */       frame_sync_type* sync,
      /* in out */       frame_node_type* node
      )
{
   // num_allocated_nodes is unsigned, so <0 is very large number
   if (--sync->num_allocated_nodes > FRAME_NODE_HEAP_SIZE) {
      log_err(sync->log, "More frame nodes are freed than exist");
      hard_exit(__FILE__, __LINE__);
   }
   memset(node, 0, sizeof *node);
   node->next = sync->frame_node_list_head;
   sync->frame_node_list_head->prev = node;
   sync->frame_node_list_head = node;
}

// memory management
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//

void print_frames(
      /* in     */       frame_sync_type* sync
      );
void print_frames(
      /* in     */       frame_sync_type* sync
      )
{
   frame_node_type* node = sync->active_frame_list_head;
   uint32_t cnt = 0;
   printf("Frame list------\n");
   while(node) {
assert(cnt < FRAME_NODE_HEAP_SIZE);
      printf("  %d  \t%.3f  (%d)\n", cnt++, node->t, node->node_num);
      node = node->next;
   }
}

// search for next time where there is a full set of temporally
//    overlapping frames. if no full overlap, returns -1.0
static double find_next_full_set(
      /* in out */       frame_sync_type* sync
      )
{
   frame_node_type* node = sync->active_frame_list_head;
   double set_time = -1.0;
   if (node == NULL) {
      goto end; // no frames -- no overlap
   }
   // rememember front and back nodes of time-overlapping section
   frame_node_type* trailing_node = node;
   frame_node_type* leading_node = node;
   uint32_t count = 1;  // number of nodes that overlap
   node = node->next;
   while (node) {
      // advance head
      leading_node = node;
      count++;
      // advance tail
      while (trailing_node->t < (leading_node->t - FRAME_ALIGN_SECS)) {
         trailing_node = trailing_node->next;
         count--;
      }
      if (count == sync->num_input_cams) {
         // we've a fully overlapping set. get middle time and return it
//         set_time = trailing_node->t + 
//               (leading_node->t - trailing_node->t) / 2.0;
         set_time = (trailing_node->t + leading_node->t) / 2.0;
         goto end;
      }
      assert(count > 0);
      assert(count <= sync->num_input_cams);
      node = node->next;
   }
end:
   return set_time;
}


// search the interval for a time when there are most frames available.
// returns time of overlap and otherwise -1
static double find_next_set(
      /* in out */       frame_sync_type* sync,
      /* in     */ const double ival_start_sec,
      /* in     */ const double ival_end_sec
      )
{
//printf("* Check next set  %.3f-%.3f\n", ival_start_sec, ival_end_sec);
//print_frames(sync);
   frame_node_type* node = sync->active_frame_list_head;
   double best_set_time = -1.0;
   if (node == NULL) {
      goto end; // no frames -- no overlap
   }
   // rememember front and back nodes of time-overlapping section
   frame_node_type* trailing_node = NULL;
   frame_node_type* leading_node = NULL;
   // count is number of nodes that overlap. this starts at 1 because 
   //    we're starting w/ 1 node
   // use float for score 
   uint32_t count = 0;
   uint32_t best_count = 0;
   best_set_time = -1.0;
   while (node) {
      if (node->t < ival_start_sec) {
//printf("   %.3f is too early\n", node->t);
         // node is before interval start. advance to next node
         node = node->next;
         continue;
      } else if (node->t > ival_end_sec) {
//printf("   %.3f is too late\n", node->t);
         // node is beyond interval end. we're done
         break;
      } else if (trailing_node == NULL) {
//printf("   setting trailing node at %.3f\n", node->t);
         trailing_node = node;
      }
      // advance head
      leading_node = node;
      count++;
//printf("*   ival %.3f - %.3f  count %d\n", trailing_node->t, leading_node->t, count);
      // advance tail
      while (trailing_node->t < (leading_node->t - FRAME_ALIGN_SECS)) {
         trailing_node = trailing_node->next;
         count--;
      }
      if (count > best_count) {
         best_count = count;
         best_set_time = (trailing_node->t + leading_node->t) / 2.0;
      }
      assert(count > 0);
      assert(count <= sync->num_input_cams);
      node = node->next;
   }
end:
   return best_set_time;
}

// support functions
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// external interface

// adds frame to sorted (by time) linked list of frames, with earliest
//    frame at head of list
static void add_frame_to_list(
      /* in out */       frame_sync_type* sync,
      /* in     */       frame_time_type *frame,
      /* in     */       uint8_t cam_num
      )
{
   // create new frame node
   frame_node_type* new_frame = allocate_frame_node(sync);
   new_frame->cam_num = cam_num;
   new_frame->t = frame->t;
   new_frame->frame = frame->frame;
   // find a place to store node in frame node list. list is sorted with
   //    earliest time at list head
   frame_node_type* node = sync->active_frame_list_head;
   if (node == NULL) {
      // list is empty. put node at head of list
      sync->active_frame_list_head = new_frame;
      
   } else {
//printf("*  frame list head at t=%.3f\n", node->t);
      while (node) {
         if (new_frame->t < node->t) {
            // frame is earlier than this node. insert it into list here
//printf("*    adding %.3f before %.3f\n", new_frame->t, node->t);
            new_frame->next = node;
            new_frame->prev = node->prev;
            if (node->prev == NULL) {
               // node is at head of list
               sync->active_frame_list_head = new_frame;
            } else {
               node->prev->next = new_frame;
            }
            node->prev = new_frame;
            break;
         } else if (node->next == NULL) {
//printf("*    adding %.3f after %.3f\n", new_frame->t, node->t);
            // reached end of list. add frame here
            node->next = new_frame;
            new_frame->prev = node;
            break;
         }
         node = node->next;
      }
   }
//print_frames(sync);
}


// removes all frames from frame list that are older than t
static void purge_old_frames(
      /* in out */       frame_sync_type* sync,
      /* in     */ const double t
      )
{
//printf("* Purging frames before %.3f\n", t);
   frame_node_type* node = sync->active_frame_list_head;
   while (node) {
      if (node->t > t) {
         // node is after past cutoff time, so are all of its children.
         //    nothing more to do
         break;
      }
      // this node head of list and it's too old. purge from list. reset 
      //    list head to next node
      sync->active_frame_list_head = node->next;
      if (node->next) {
         node->next->prev = NULL;
      }
      // delete this node and advance to next
      frame_node_type* corpse = node;
      node = node->next;
      free_frame_node(sync, corpse);
   }
   sync->last_sync_time = t;
}


// a new frame has arrived. add it to the list of frames and look for
//    a full set to publish. if that's not available and too much
//    time has elapsed since the first frame, then publish the next best
//    set
// returns >0.0 if there's content to be published and <0.0 otherwise
static double check_for_frame_set(
      /* in out */       frame_sync_type* sync,
      /* in     */ const double frame_time
      )
{
//printf("* Check for frame set at %.3f  (%.3f)\n", frame_time, sync->last_sync_time);
   // look for full set of frames
   double publish_time = -1.0;
   double dt_sec = frame_time - sync->last_sync_time;
   if (dt_sec > STREAM_DUMP_INTERVAL_SEC) {
//printf("*   stream dump\n");
      // too much time has elapsed between this frame and the previous
      //    sync time. ignore unpublished data and reset clock to now.
      sync->last_sync_time = frame_time - CAMERA_FRAME_INTERVAL_SEC;
      purge_old_frames(sync, sync->last_sync_time);
   } else if (dt_sec > MISSED_FRAME_INTERVAL_SEC) {
//printf("*   missed frame\n");
      // if we might have missed a frame set, look for it
      // search window here should be large enough so that there
      //    is no 'blind' time that frames can occur but aren't considered
      //    for being included in a set (to account for pathological async
      //    edge cases). slightly larger than the frame capture interval 
      //    should be fine
      double target_sec = sync->last_sync_time + CAMERA_FRAME_INTERVAL_SEC;
      double ival_start_sec = target_sec - 0.51 * CAMERA_FRAME_INTERVAL_SEC;
      double ival_end_sec = target_sec + 0.51 * CAMERA_FRAME_INTERVAL_SEC;
      publish_time = find_next_set(sync, ival_start_sec, ival_end_sec);
      if (publish_time > 0.0) {
         // found a set to publish
         sync->last_sync_time = publish_time;
      } else {
         // didn't find anything. advance time and keep looking
         sync->last_sync_time += CAMERA_FRAME_INTERVAL_SEC;
         purge_old_frames(sync, sync->last_sync_time);
      }
   } else {
//printf("*   check full\n");
      // we're no longer overdue for a frame. check to see if a full set
      //    has been delivered
      publish_time = find_next_full_set(sync);
   }
   return publish_time;
}


// write next frame set to output
// there must be one or more frames near 't' 
// returns number of frames in set
static uint32_t build_frame_set(
      /* in out */       frame_sync_type* sync,
      /*    out */       frame_sync_output_type* out,
      /* in     */ const double t
      )
{
   // look for frames that are part of this set
   frame_node_type* node = sync->active_frame_list_head;
   memset(out, 0, sizeof *out);
   uint32_t count = 0;
   while (node) {
      if (node->t <= (t + FRAME_ALIGN_SECS/2.0)) {
         if (node->t >= (t - FRAME_ALIGN_SECS/2.0)) {
            if (out->frames[node->cam_num] == NULL) {
               out->frames[node->cam_num] = node->frame;
               count++;
            } else {
               // duplicate frames. this is an internal error but
               //    not fatal. log it and move on
               log_err(sync->log, "Duplicate frames from one cam in a "
                     "frame set. Cam %d, t=%.3f", node->cam_num, node->t);
            }
         }
      } else {
         // this node is after frame window -- no possible match for this or
         //    later times
         break;
      }
      node = node->next;
   }
   if (count == 0) {
      // this shouldn't happen -- frame set should only be published
      //    when it's known to exist. this is internal error but for
      //    now just log it and auto-recover
      log_err(sync->log, "Attempted to publish null frame set");
   }
   return count;
}

// cycle through all producers and return time of earliest available
//    frame
// if none available, return -1
static int32_t get_next_earliest_frame(
      /* in out */       datap_desc_type *self,
      /*    out */       frame_time_type *frame
      )
{
   int32_t early_idx = -1;
   frame->t = 1.0e30;
   const uint32_t n_producers = self->num_attached_producers;
   for (uint32_t i=0; i<n_producers; i++) {
      const producer_record_type *pr = &self->producer_list[i];
      const datap_desc_type *producer = pr->producer;
      if (pr->consumed_elements < producer->elements_produced) {
         // data available from this producer -- evaluate when
         const uint32_t idx = (uint32_t) 
               (pr->consumed_elements % producer->queue_length);
         const double t = producer->ts[idx];
         if (t < frame->t) {
            frame->frame = 
                  (optical_up_output_type *) dp_get_object_at(producer, idx); 
            frame->t = t;
            early_idx = (int32_t) i;
         }
      }
   }
   // pop this frame from the producer so we don't process it again
   if (early_idx >= 0) {
      self->producer_list[early_idx].consumed_elements++;
   }
   return early_idx;
} 


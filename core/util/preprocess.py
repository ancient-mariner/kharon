#!/usr/bin/env python3
import sys
import os
import random

#OUTPUT_DIR = "/tmp/ass_check/"
# input files
IN_TRACKER_LOG0    = "log_tracker-0"
IN_TRACKER_LOG1    = "log_tracker-1"
IN_ASSOCIATOR_LOG  = "log_associator"
# input files
OUT_TRACKER_LOG0    = "chk_tracker-0.txt"
OUT_TRACKER_LOG1    = "chk_tracker-1.txt"
OUT_ASSOCIATOR_LOG  = "chk_associator.txt"

#if not os.path.exists(OUTPUT_DIR):
#   os.makedirs(OUTPUT_DIR)

# pull log string from log entry
def get_content(line_filter, line):
   toks = line.split()
   if toks[0] != line_filter:
      return None
   offset = line.find(': ')
   return line[2+offset:]

########################################################################
# color determination

# select an RGB combination using target ID and instance num as seed
# r,g,b selected so sum is normalized to 1.2 (255)
# returns values as string "r,g,b"
def select_color(target_id_str):
   toks = target_id_str.split('-')
   target_id = int(toks[0])
   target_instance = int(toks[1])
   random.seed(11 * target_id + 17 * target_instance)
   r = random.random() + 0.001
   g = random.random() + 0.001
   b = random.random() + 0.001
   tot = 0.75 * (r + g + b)
   r = 255.0 * r / tot
   g = 255.0 * g / tot
   b = 255.0 * b / tot
   r = int(min(r, 255.0))
   g = int(min(g, 255.0))
   b = int(min(b, 255.0))
   return "%d,%d,%d" % (r, g, b)


########################################################################
# associator parsing

ASS_OPEN = 1
ASS_DATA = 2

def process_ass_log(in_name, out_name):
   state = ASS_OPEN
   targets = []
   timestamp = -1.0
   with open(in_name, 'r') as fin:
      with open(out_name, 'w') as fout:
         for line in fin:
            if state == ASS_OPEN:
               # look for info line starting with 'Publish '
               if len(line.strip()) == 0:
                  continue
               content = get_content('INFO', line)
               if content is not None:
                  toks = content.strip().split()
                  if toks[0] == 'Publish':
                     timestamp = toks[1]
                     state = ASS_DATA
            elif state == ASS_DATA:
               # keep copying while line starts w/ 'OUT'
               if len(line.strip()) == 0:
                  continue
               content = get_content('INFO', line)
               if content is not None:
#                  toks = content.split()
                  if content.strip().startswith('OUT'):
                     targets.append(content[4:].strip())
                  elif content.strip().startswith('End publish'):
                     state = ASS_OPEN
#                  if toks[0] == 'OUT':
#                     targets.append(content[4:].strip())
#                  else:
#                     state = ASS_OPEN
            #
            if state == ASS_OPEN and len(targets) > 0:
               # end of target list reached. write to output
               fout.write('%s %d\n' % (timestamp, len(targets)))
               for tgt in targets:
                  # parse out target ID to get color and include that
                  target_id = tgt.split()[0]
                  color = select_color(target_id)
                  fout.write('%s %s\n' % (color, tgt))
               targets = []

# associator parsing
########################################################################
########################################################################
# tracker parsing

TRK_OPEN = 1
TRK_DATA = 2

def process_track_log(in_name, out_name):
   state = TRK_OPEN
   targets = []
   timestamp = -1.0
   with open(in_name, 'r') as fin:
      with open(out_name, 'w') as fout:
         for line in fin:
            if state == TRK_OPEN:
               # look for info line starting with 'Gaze center'. get
               #     timestamp from end of that string and change
               #     state
               if len(line.strip()) == 0:
                  continue
               content = get_content('INFO', line)
               if content is not None:
                  if content.startswith('Gaze center'):
                     toks = content.strip().split()
                     timestamp = toks[-1]
                     state = TRK_DATA
            elif state == TRK_DATA:
               # keep copying while line starts w/ 'publish target'
               if len(line.strip()) == 0:
                  continue
               content = get_content('INFO', line)
               if content is not None:
                  if content.startswith('publish target'):
                     targets.append(content[15:].strip())
#                  elif content.startswith('hide target'):
#                     # non-published target. ignore
#                     #targets.append(content[15:].strip())
#                     pass
                  elif content.startswith('Publishing '):
                     # look for explicit end, in case auxiliary log data
                     #     is embedded
                     state = TRK_OPEN
#                  else:
#                     state = TRK_OPEN
            #
            if state == TRK_OPEN and len(targets) > 0:
               # end of target list reached. write to output
               fout.write('%s %d\n' % (timestamp, len(targets)))
               for tgt in targets:
                  # parse out target ID to get color and include that
                  target_id = tgt.split()[0]
                  color = select_color(target_id)
                  fout.write('%s %s\n' % (color, tgt))
               targets = []

########################################################################



if len(sys.argv) != 2:
   print("Usage: %s <log dir>" % sys.argv[0]);
   sys.exit(1)

log_dir = sys.argv[1]
if not log_dir.endswith('/'):
   log_dir += '/'

process_ass_log(log_dir + IN_ASSOCIATOR_LOG, log_dir + OUT_ASSOCIATOR_LOG)
process_track_log(log_dir + IN_TRACKER_LOG0, log_dir + OUT_TRACKER_LOG0)
process_track_log(log_dir + IN_TRACKER_LOG1, log_dir + OUT_TRACKER_LOG1)


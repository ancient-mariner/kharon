#!/bin/bash 

# script to build depth map folder. this wrapper should be used instead of
#     using the individual utilities in the 'create' folder

# error handling in subscripts is pretty poor, with segfaults sometimes
#     occuring on bad input params. this should be fixed


###############
# configuration

gebco_dir=/opt/kharon/mapping/2020_gebco_ascii/

#noaa_cat=test.txt
noaa_cat=big_text_files/files_noaa.txt

shoreline_data=big_text_files/box_f.txt
#shoreline_data=test_shore.txt

# turn on/off subscripts by setting to 1/0
# these can all be set to '1' to create a map in one go, but it can
#  be helpful to activate only one at a time until one's confident that
#  everything's working OK (note that create and init_level1 must be
#  run together)
create_mapdir=0      # creates a fresh map directory, deleting existing
init_level1_gebco=0  # creates new level-1 map based on gebco
init_level2_gebco=0  # creates new level-2 maps (15sec) based on jebco
init_level3_noaa=0   # creates new level-3 maps (5sec) based on noaa
update_level3_shoreline=1   # update level-3 maps (5sec) w/ shoreline data

########################################################################
########################################################################
# content below should not need to be modified

# declare -e separately, in case script is launched by 'bash <script>'
# we want to exit if any subprocess fails
set -e

if (( $# != 1 )); then
   echo "Usage: $0 <map dir>"
   exit 1
fi
map_dir=$1

if (( $create_mapdir == 1 )); then
   echo "Creating map directory '$map_dir'"
   rm -f $map_dir/world.map1
   rm -f $map_dir/15sec/*0/*
   rm -f $map_dir/5sec/*0/*
   # create base dirs
   mkdir -p $map_dir
   # create subdirs for level2 maps, so thousands aren't in one directory
   for i in {0..170..10}
   do
      mkdir -p $map_dir/15sec/$i
      mkdir -p $map_dir/5sec/$i
   done
fi

if (( $init_level1_gebco == 1 )); then
   create/init_map1_gebco $map_dir $gebco_dir
fi


if (( $init_level2_gebco == 1 )); then
   create/load_gebco_map2 $map_dir $gebco_dir
fi

if (( $init_level3_noaa == 1 )); then
   create/read_noaa $map_dir $noaa_cat
fi

if (( $update_level3_shoreline == 1 )); then
   create/horeline $map_dir $shoreline_data
fi



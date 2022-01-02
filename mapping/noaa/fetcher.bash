#!/bin/bash

# this script is to automatically download NOAA's .xyz depth files

# script takes an NOS manifest file and downloads the files listed 
#     there using wget
# files are stored in specified output directory
# files are not re-downloaded if they're in the output directory

# name of file is hard-coded, as is output directory. this should be run
#     infrequently, and the script will have to be evaluated to re-learn
#     its contents by the time it needs to be rerun. it's easier to change
#     this value manually than to automate things, given its infrequent use

# given the size of the NOS file, and the fact that it contains several
#     maps that probably don't need to be downloaded, it's probably a good
#     idea to break the manifest file up into smaller pieces (eg, a couple
#     thousand files each)

#file="nos_products_manifest.2020-11-02.xyz"
file="nos_4.xyz"
dir="xyz_4"
lines=$(sed {s/\,\ [0-9a-z]*//} $file)
for line in $lines 
do
   fname=$(basename $line)
   echo "Checking $fname"
   if [ ! -f $dir/$fname ]; then
      wget $line
      mv $fname $dir/
   fi
done


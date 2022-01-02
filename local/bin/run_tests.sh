#!/bin/bash
rc=0
rm -f _log.txt
rm -f _log.err
for file in test_*
do
   ./"$file" >> _log.txt 2>> _log.err
   rc=$(($rc + $?));
done

if [ $rc -gt 0 ]
then
   echo Encountered $rc errors. Check _log.txt and _log.err
else
   echo All tests passed
fi


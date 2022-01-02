#!/bin/bash
./wipe_ass.sh
./preprocess.py $1
./check_associator $1

#!/bin/bash

# Define the directory to search
DIR="build/"

# Find the newest CSV file in the directory
newest_csv=$(find "$DIR" -type f -name "*.csv" -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)

echo $newest_csv
scp $newest_csv kyle@192.168.0.231:/home/kyle


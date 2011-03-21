#!/bin/bash

rm *.check

for file in *.[ch]
do
/opt/STM/linux_spear-1310/scripts/checkpatch.pl -f "$file" > "$file".check
done

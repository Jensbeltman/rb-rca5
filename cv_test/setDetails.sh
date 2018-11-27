#!/bin/bash

folders=$(ls -1 datasets/ | grep images* | wc -l)

echo $folders > datasets/details.txt

for i in $(eval echo {1..$folders})
do
    echo $(ls -1 datasets/images$i/ | grep test_img* | wc -l) >> datasets/details.txt
done

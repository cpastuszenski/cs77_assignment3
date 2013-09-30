#!/bin/bash

function check_error {
    if [ "$?" -ne "0" ]; then
        echo "An error occurred!"
        exit 1
    fi
}

echo "test01: Animated Transformations"
./view -i -t 2.5 scenes/test01.json
check_error

echo "test02: Skinned Mesh"
./view -i -t 2.5 scenes/test02.json
check_error

echo "test03: Particles"
./view -i -t 2.5 scenes/test03.json
check_error

echo "test04: Cloth"
./view -i -t 10 scenes/test04.json
check_error

echo "test05: Rain (Beware, this chugs)"
./view -i -t 10 scenes/test05.json
check_error

echo "test06: Snow"
./view -i -t 10 scenes/test06.json
check_error

echo "All completed successfully!"

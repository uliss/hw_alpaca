#!/bin/bash

cat abc.h | grep -v '#' | grep '"' |
    grep -v // | gsed 's/[",\\]//g' |
    gsed 's/ //g' | gsed 's/^/0b/' |
    gsed '1~2s/$/ |/' | gsed '0~2s/$/ << 4, /g' |
    gsed '0~6s/4,/4\n},/' | gsed '1~7s/^/{\n/' > abc_comp.h

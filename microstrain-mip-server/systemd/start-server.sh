#!/bin/bash

args=( $1 )
echo "Dev name: ${args[0]}"
echo "Short serial: ${args[1]}"
PORT=5`echo ${args[1]} | tr -d -c [:digit:]  | tail -c 4`
/usr/local/bin/MicrostrainMipServer -d ${args[0]} -p $PORT -h 0.0.0.0
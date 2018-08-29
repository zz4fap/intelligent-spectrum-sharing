#!/bin/bash

origin=$(pwd)
cd communicator
protoc --python_out=./python --cpp_out=./cpp interf.proto
cd ${origin}

#protoc --python_out=. interf.proto


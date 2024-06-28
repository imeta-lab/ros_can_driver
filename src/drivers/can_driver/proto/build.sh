#!/bin/bash

chmod 777 $0

bash_path=$(realpath $0)
proto_path=$(dirname $bash_path)

pushd ${proto_path} > /dev/null;
    proto_srcs=$(find ~+ -type f -name "*.proto")
popd > /dev/null;

protoc --cpp_out=${proto_path} --proto_path=${proto_path}  ${proto_srcs}
#!/usr/bin/zsh

config_file=./champsim_config.json

./config.sh $config_file
make -j200


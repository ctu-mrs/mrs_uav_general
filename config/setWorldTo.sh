#!/bin/bash
ln -sf $1 world_current.yaml

current_world_is=` ls -l world_current.yaml  | grep -Po '(?<=->\s)\w+' `
echo "current_world IS SET TO ""$current_world_is"
  


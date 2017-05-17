#!/bin/bash
ln -sf $1 arena_current.yaml

current_arena_is=` ls -l arena_current.yaml  | grep -Po '(?<=->\s)\w+' `
echo "current_arena IS SET TO ""$current_arena_is"
  


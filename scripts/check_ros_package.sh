#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Skipping required package check, no list of packages provided!"
    exit 0
fi

if [ ! -f "$1" ]; then
    echo "Skipping required package check, provided file does not exits!"
    exit 0
fi

result=0
RED='\e[31m'
GREEN='\e[32m'
NC='\e[39m' # No Color

while read line; do
  VAR=$(rospack list-names | grep $line)
  echo -e "checking: $line ... \c"
  if [ -z "${VAR}" ]; then
    echo -e "${RED}missing${NC}"
    result=1
  else
    echo -e "${GREEN}found${NC}"
  fi
done < $1

if [[ $result -ne 0 ]]; then
  echo -e "${RED}Some required packages were not found, the simulation will probably not work correctly.${NC}"

  default=n
  { read -t 10 -n 2 -p $'Proceed to run the simulation? [y/n] (default: '"$default"$')' resp || resp=$default ; }
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then
    exit 0
  else
    exit 1
  fi
fi

exit 0

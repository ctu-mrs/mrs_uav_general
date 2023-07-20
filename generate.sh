#!/bin/bash

bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro noetic

fakeroot debian/rules binary

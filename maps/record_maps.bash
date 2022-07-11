#!/bin/bash
source ~/remote_husky.sh
read -p "Enter map name: " mapName
rosrun map_server map_saver -f $mapName map:=/rtabmap/grid_map
echo "Press enter once launch file is killed"
while [ true ] ; do
read -s -N 1 -t 1 key
if [[ $key == $'\x0a' ]];
then
cp ~/.ros/rtabmap.db $mapName.db
exit 0;
fi
done


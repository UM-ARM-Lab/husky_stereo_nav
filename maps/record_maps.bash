#!/bin/bash
read -p "Enter map name: " mapName
if [ -f $mapName.db ];
then
echo "This map already exists. Are you sure you want to overwrite it? y/n"
while [ true ] ; do
read -s -N 1 -t 1 key
if [[ $key == $'y' ]];
then
rosrun map_server map_saver -f $mapName map:=/rtabmap/grid_map
echo "Map file has been overwritten. Press enter once launch file is killed"
break
elif [[ $key != $'y' && $key != '' ]];
then
echo "Map file was not overwritten."
exit 0;
fi
done
else
rosrun map_server map_saver -f $mapName map:=/rtabmap/grid_map
echo "Press enter once launch file is killed"
fi
while [ true ] ; do
read -s -N 1 -t 1 key
if [[ $key == $'\x0a' ]];
then
cp ~/.ros/rtabmap.db $mapName.db
exit 0;
fi
done

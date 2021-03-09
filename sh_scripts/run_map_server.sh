. /home/rodion/.bashrc
ros2 run nav2_map_server map_saver_cli -f /home/rodion/rodion_nav/map
cp /home/rodion/rodion_nav/map.pgm /home/rodion/rodion_nav/map_filter.pgm
cp /home/rodion/rodion_nav/map.yaml /home/rodion/rodion_nav/map_filter.yaml


#!/bin/bash
# Lower the simulated sensor rates so Gazebo runs closer to real time on a slow PC / VDI.
# Makes a copy of the robot description packages in ~/tb4_fast_overlay (no sudo, /opt is untouched).
# Undo: rm -rf ~/tb4_fast_overlay and open a new terminal.
#
# usage:  bash fast_sim_overlay.sh        (once)
#         source ~/tb4_fast_overlay/setup.bash   (in the simulator terminal, after the other source lines)

OV=$HOME/tb4_fast_overlay
LIDAR_HZ=10
CAMERA_HZ=1
SMALL_HZ=5

mkdir -p $OV/share/ament_index/resource_index/packages
for p in turtlebot4_description irobot_create_description; do
  if [ ! -d /opt/ros/humble/share/$p ]; then echo "missing /opt/ros/humble/share/$p"; exit 1; fi
  rm -rf $OV/share/$p
  cp -r /opt/ros/humble/share/$p $OV/share/$p
  touch $OV/share/ament_index/resource_index/packages/$p
done

set_rate () {  # file  rate   (covers <update_rate>N</update_rate>, update_rate="N",
               #               update_rate:=N macro defaults and xacro:property name="update_rate")
  sed -i -E "s#(<update_rate>)[0-9.]+(</update_rate>)#\1$2\2#g; \
             s#(update_rate=\")[0-9.]+(\")#\1$2\2#g; \
             s#(update_rate:=)[0-9.]+#\1$2#g; \
             s#(name=\"update_rate\"[[:space:]]+value=\")[0-9.]+(\")#\1$2\2#g" "$1"
}

for f in $(grep -rlE "update_rate" $OV/share --include=*.xacro --include=*.urdf --include=*.sdf); do
  name=$(basename $f)
  case $name in
    *lidar*|*rplidar*)            set_rate $f $LIDAR_HZ ;;
    *oakd*|*camera*)              set_rate $f $CAMERA_HZ ;;
    *cliff*|*ir_intensity*|*ir_opcode*) set_rate $f $SMALL_HZ ;;
  esac
done

cat > $OV/setup.bash <<EOF
export AMENT_PREFIX_PATH=$OV:\$AMENT_PREFIX_PATH
export IGN_GAZEBO_RESOURCE_PATH=$OV/share:\$IGN_GAZEBO_RESOURCE_PATH
EOF

echo "sensor rates now:"
grep -rnE "update_rate" $OV/share --include=*.xacro --include=*.urdf --include=*.sdf | sed "s#$OV/share/##"
echo
echo "done. in the simulator terminal run:  source $OV/setup.bash"

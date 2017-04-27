#!/bin/bash
# edit CPUSHDATA_BASE=$HOME/cpushdata to your push data directory

thisFile=$_
if [ $BASH ] 
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_cpush_base()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export CPUSH_BASE=$(dirname $configParentDir);;
    "build") export CPUSH_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: CPUSH environment file is stored in unrecognized location: $thisFile";;
  esac
  export CPUSHDATA_BASE=$CPUSH_BASE/../cpushdata
  export PATH=$PATH:$CPUSH_BASE/software/build/bin
}

setup_cpush()
{
  export PATH=$PATH:$CPUSH_BASE/software/build/bin
  export PATH=$PATH:$HOME/software/libbot/build/bin  # for lcm and libbot install
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$CPUSH_BASE/software/build/lib:$CPUSH_BASE/software/build/lib64:$LD_LIBRARY_PATH
  export GUROBI_HOME="$HOME/software/gurobi702/linux64"
  export PATH="${PATH}:${GUROBI_HOME}/bin"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
  #~ export LD_LIBRARY_PATH=$HOME/software/gurobi651/linux64/lib:$LD_LIBRARY_PATH  # for gurobi
  
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$CPUSH_BASE/software/build/share/java/lcmtypes_cpush_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$CPUSH_BASE/software/build/share/java/drake.jar:$CPUSH_BASE/software/build/share/java/bot2-lcmgl.jar
  export PKG_CONFIG_PATH=$CPUSH_BASE/software/build/lib/pkgconfig:$CPUSH_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH
  export GRB_LICENSE_FILE=$HOME/software/gurobi651/gurobi.lic

  # python path
  export PYTHONPATH=$PYTHONPATH:$CPUSH_BASE/software/build/lib/python2.7/site-packages:$CPUSH_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
  
  export PATH=$PATH:$HOME/software/ffmpeg-2.4.2-64bit-static # for ffmpeg software
  
  export ROSLAUNCH_SSH_UNKNOWN=1
}

set_ros()
{
  if [ -f $CPUSH_BASE/catkin_ws/devel/setup.bash ]; then
    source $CPUSH_BASE/catkin_ws/devel/setup.bash
  else
    source /opt/ros/kinetic/setup.bash
  fi
  export ROS_PACKAGE_PATH=$HOME/cpush/ros_ws/:$ROS_PACKAGE_PATH
}

# some useful commands
alias cdcpush='cd $CPUSH_BASE'
alias cdcpushdata='cd $CPUSHDATA_BASE'

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $CPUSH_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot2_SetSpeed 1600 180'
alias faster='rosservice call /robot2_SetSpeed 200 50'
alias fast='rosservice call /robot2_SetSpeed 100 30'
alias slow='rosservice call /robot2_SetSpeed 50 15'

alias gohome='rosservice call robot2_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 90, j6: 0}"'

alias teleop='rosrun teleop teleop'
alias pythoncpush='ipython -i -c "run $CPUSH_BASE/catkin_ws/src/cpush_config/python/pythoncpush.py"'

alias pman='bot-procman-sheriff -l $CPUSH_BASE/software/config/cpush.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias getjoint='rosservice call -- robot2_GetJoints'
alias getcart='rosservice call -- robot2_GetCartesian'
alias setjoint='rosservice call -- robot2_SetJoints'
alias setcart='rosservice call -- robot2_SetCartesian'
alias setspeed='rosservice call /robot2_SetSpeed'
alias zeroft='rosservice call zero'

alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'


alias catmake='cd $CPUSH_BASE/catkin_ws; catkin_make; cd -;'

ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE
   
   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

set_cpush_base
setup_cpush
set_ros
set_bash


exec "$@"

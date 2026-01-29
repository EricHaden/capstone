# Source ROS 2 workspace - works in both bash and zsh.
# From ros2_ws run:  source ./setup_workspace.sh

if [ -n "$ZSH_VERSION" ]; then
  _dir="$(cd "$(dirname "${(%):-%x}")" && pwd)"
else
  _dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

if [ -n "$ZSH_VERSION" ] && [ -f "$_dir/install/setup.zsh" ]; then
  . "$_dir/install/setup.zsh"
elif [ -f "$_dir/install/setup.bash" ]; then
  . "$_dir/install/setup.bash"
else
  echo "No install/setup found. Run: colcon build" 1>&2
  unset _dir
  return 1 2>/dev/null || exit 1
fi
unset _dir

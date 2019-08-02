#! /bin/sh
#
# Update the <robot>_maps/config symlink to point to the map that is
# currently being used.

set -e

MAPS_LINK=$HOME/.pal/maps

# Ensure we are not creating a recursive symlink
# This is important since often <robot>_maps/config may be used as path.
abspath() {
  (cd "${1%/*}" &>/dev/null && echo "$(pwd)/${1##*/}")
}

# Check parameters
if [ $# -lt 1 ]; then
  echo "Usage: $0 <map path> [ROS args...]"
  echo "Updates the symlink $HOME/.pal/maps to the given path."
  exit 1
fi

# Ensure target directory exists
if [ ! -d "$1" ]; then
  echo "Warning: Target path $1 doesn't exist: try cp_maps_to_home.sh"
  PKG=`echo $1 | awk '{ n=split($0, s, "/"); for (i=1; i<=n; i++) { if (index(s[i], "_maps") != 0) { print s[i] } } }'`
  rosrun pal_navigation_sm cp_maps_to_home.sh $PKG
  if [ $? -ne 0 ]; then
    echo "Error: Failed to copy maps from $PKG to $HOME/.pal."
    exit 3
  fi
fi

# Create a symlink $HOME/.pal/maps pointing to $HOME/.pal/<current_robot>_maps
if [ -e "$MAPS_LINK" ]; then
  if [ ! -h "$MAPS_LINK" ]; then
    echo "Error: Path is not a symlink: $MAPS_LINK"
    exit 2
  fi

  if [ "`abspath $MAPS_LINK`" != "`abspath $S1`" ]; then
    echo "Warning: link is not pointing to $1, updating"
    unlink "$MAPS_LINK"
  fi
fi

# Create the new maps symlink!
ln -s "$1" "$MAPS_LINK"

echo "Done."

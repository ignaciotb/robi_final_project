#! /bin/sh
#
# Copies the pose to $HOME/.pal; creates the target folder if it doesn't exist.

TARGET=$HOME/.pal

# Ensure target directory exists
if [ ! -d "$TARGET" ]; then
  echo "Warning: Target path $TARGET doesn't exist. Creating it."
  mkdir -p $TARGET
  if [ $? -ne 0 ]; then
    echo "Error: Target path $TARGET couldn't be created."
    exit 3
  fi
fi

# Copy pose target folder
cp `rospack find pal_navigation_sm`/config/pose.yaml $TARGET
echo "Done."

#! /bin/sh
#
# Copies the maps from a given pkg (robot) to $HOME/.pal; creates the target
# folder if it doesn't exist.

TARGET=$HOME/.pal

# Check parameters
if [ $# -lt 1 ]; then
  echo "Usage: $0 <maps pkg>"
  echo "Copies the maps from the pkg to $TARGET/<maps pkg>."
  exit 1
fi
PKG=$1

# Find pkg path
SOURCE=`rospack find -q $PKG`
if [ "$SOURCE" = "" ]; then
  echo "Error: Couldn't find package $PKG path."
  exit 1
fi

# Ensure source directory exists
if [ ! -d "$SOURCE/configurations" ]; then
  echo "Error: Source path $SOURCE/configurations doesn't exist."
  exit 3
fi
if [ ! -L "$SOURCE/config" ]; then
  echo "Error: Source path $SOURCE/config doesn't exist."
  exit 3
fi

# Ensure target directory exists
TARGET=$TARGET/$PKG
if [ ! -d "$TARGET" ]; then
  echo "Warning: Target path $TARGET doesn't exist. Creating it."
  mkdir -p $TARGET
  if [ $? -ne 0 ]; then
    echo "Error: Target path $TARGET couldn't be created."
    exit 3
  fi
fi

# Copy maps from source into target folder
cp -r $SOURCE/configurations $TARGET
cp -d $SOURCE/config $TARGET
echo "Done."

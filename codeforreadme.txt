

sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
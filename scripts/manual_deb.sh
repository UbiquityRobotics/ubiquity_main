################
# Read:
#  http://answers.ros.org/question/173804/generate-deb-from-ros-package/
# Browse:
#  https://wiki.debian.org/BuildingTutorial
#  http://answers.ros.org/question/11315/creating-private-deb-packages-for-distribution/
################
# Install required software:
sudo apt-get install -y build-essential fakeroot devscripts equivs
sudo apt-get install -y python-bloom gdebi-core
################
# Make sure that there is no `debian` directory
rm -rf debian
################################################################
# Run bloom to generate `debian` directory.  The `rosdebian`
# argument will install files into /ros/indigo/...  In theory,
# replacing `rosdebian` with `debian` will install into `/usr`,
# but it does not seem to work that well:
bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
################
# Deal with dependencies:
sudo mk-build-deps -i -r
################
# Now build the package:
fakeroot debian/rules clean
fakeroot debian/rules binary
################

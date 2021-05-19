rm -rf /opt/realsense
sudo rm /etc/ld.so.conf.d/realsense.conf
sudo ldconfig
sudo rm /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && udevadm trigger
# --> remove the line "export realsense2_DIR=/opt/realsense/lib/cmake/realsense2" from ~/.bashrc

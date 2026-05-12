sudo minicom -D /dev/ttyUSB0

ip link set end0 up
dhclient -v end0

cat /sys/class/fpga_bridge/*/state

scp rock_lee.rbf root@192.168.80.198:~/fpga-binaries

mount /dev/mmcblk0p1 fat/

cp fpga-binaries/blink.rbf fat/soc_system.rbf

reboot

cp fat/socfpga_cyclone5_de0_nano_soc_orig.dtb fat/socfpga_cyclone5_de0_nano_soc.dtb

=================================================
====Tools Usage
=================================================
python3 -m venv venv
source venv/bin/activate

pip install matplotlib numpy pandas scipy seaborn
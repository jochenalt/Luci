#!/bin/bash

function ensure-being-root {
   w=`whoami`
   if [ "$w" = "root" ]
   then
   		return 0
   else
        echo ""
		echo "I need to be root to do this."
   	    return  1
   fi
}

# video checking
videodev=/dev/video0
echo -n "check webcam "
if [ -e "$videodev" ] 
then
	if [ -c "$videodev" ] 
	then
		echo "ok."
	else
		echo "device $videodev is no character device"
	fi
else
	echo -n "device $videodev does not exist. Try reset usb."
	ensure-being-root
	if [ $? -eq 0 ]
	then 
		echo 0 > /sys/bus/usb/devices/1-3.1/authorized
		echo 1 > /sys/bus/usb/devices/1-3.1/authorized

       ./usbreset /dev/bus/usb/001/002
       ./usbreset /dev/bus/usb/001/003
	fi
	
fi


# i2c checking
echo -n "check i2c "
i2cdevpath="/dev/i2c-4"
i2cdevno=4
i2caddr="04"
i2cdev=`i2cdetect -y -r $i2cdevno | grep $i2caddr`
if [ $? -eq 1 ]
then
  echo
  echo "i2c device with address $i2caddr on $i2cdevpath not found. Check electronics." 
else
  echo "ok".
fi


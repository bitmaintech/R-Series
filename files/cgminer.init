#!/bin/sh /etc/rc.common
#set -x
START=99

APP=cgminer
PID_FILE=/var/run/$APP.pid


start() {
	if [ !  -d "/sys/class/gpio/gpio27" ];then
		echo 27 > /sys/class/gpio/export
	fi
        echo out > /sys/class/gpio/gpio27/direction
        echo 0 > /sys/class/gpio/gpio27/value
        sleep 1
        echo 1 > /sys/class/gpio/gpio27/value
	local _pool1url
	local _pool1user

	local _pool2url
	local _pool2user

	local _pool3url
	local _pool3user
	local _freq
	
	config_load cgminer

	config_get _aa default api_allow
	config_get _freq default freq
	config_get _pool1url default pool1url
	config_get _pool1user default pool1user
	config_get _pool1pw default pool1pw
	config_get _pool2url default pool2url
	config_get _pool2user default pool2user
	config_get _pool2pw default pool2pw
	config_get _pool3url default pool3url
	config_get _pool3user default pool3user
	config_get _pool3pw default pool3pw
	
	if [ "$_pool1url" != "" -a "$_pool1user" != "" -a "$_pool1pw" != "" ]; then
	    POOL1="-o $_pool1url -O $_pool1user:$_pool1pw"
	fi
	if [ "$_pool2url" != "" -a "$_pool2user" != "" -a "$_pool2pw" != "" ]; then
	    POOL2="-o $_pool2url -O $_pool2user:$_pool2pw"
	fi
	if [ "$_pool3url" != "" -a "$_pool3user" != "" -a "$_pool3pw" != "" ]; then
	    POOL3="-o $_pool3url -O $_pool3user:$_pool3pw"
	fi

	source /etc/init.d/usr_bak

#	_mac=`/sbin/ifconfig wlan0  | sed -n '/HWaddr/ s/^.*HWaddr *//pg' | sed 's/://g'| sed 's/ //g'`
#	_mac=$(cat /sys/devices/platform/ar933x_wmac/ieee80211/phy0/macaddress | sed 's/://g'| tr -d '\n')

    #TO=$(( 12690 / $_cf ))
	#_cf = 25 *((($_regv >> 7)&0x7f) + 1)/(((($_regv >> 2) & 0x1f) + 1) * (0x1 << (&_regv & 0x3)))
	#AOPTIONS=" --icarus-timing 2 -G -D"
	#AOPTIONS="--bitmain-options 115200:32:8:$_freq"
	AOPTIONS="--antrouter-options 115200:$_freq"
	#PARAMS=" --lowmem $AOPTIONS $POOL1 $POOL2 $POOL3 $_pb --api-allow $_aa --api-listen"
	#PARAMS="-D $AOPTIONS $POOL1 $POOL2 $POOL3 $_pb $_ow $_bec --api-listen --api-network"
	PARAMS="$AOPTIONS $POOL1 $POOL2 $POOL3 --api-listen --antrouter-vil 1 --api-allow $_aa  --version-file /usr/bin/compile_time --queue 50"
	$APP --lowmem --bitmain-options 115200:$_freq -q >/dev/null 2>&1

	sleep 1
	#when ntpd fail	can delet #
	touch /tmp/cgminer-ntpd-done
	cnt=0
	if [ ! -f /tmp/cgminer-ntpd-done ]; then
		while [ "$NTPD_RET" != "0" ]; do
			ntpd -d -n -q -N \
			    -p 0.openwrt.pool.ntp.org \
			    -p 1.openwrt.pool.ntp.org \
			    -p 2.openwrt.pool.ntp.org \
			    -p 3.openwrt.pool.ntp.org
				cnt=$(($cnt+1))
				if [ $cnt -gt 0 ];then
					echo cnt
					break
				fi
			NTPD_RET=$?
		done

		touch /tmp/cgminer-ntpd-done
	fi
	echo $PARAMS
	start-stop-daemon -S -x $APP -p $PID_FILE -m -b -- $PARAMS
	#start-stop-daemon -S -x $APP -p $PID_FILE -m -- $PARAMS
}

stop() {
	start-stop-daemon -K -n $APP -p $PID_FILE -s TERM
#	start-stop-daemon -K -n udevd -s TERM
}

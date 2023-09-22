#!/bin/bash
# ネットワークの設定
rm -f /run/dbus/pid
rm -f /run/avahi-daemon//pid
dbus-daemon --system
# avahi-daemonをバックグラウンドで起動
avahi-daemon &

# terminatorを起動
terminator -T "$1"

# コンテナを動作させ続けるための待機
tail -f /dev/null

#! /bin/bash

echo "Enter the 'About' submenu of the Robot Controller, and connect to the WiFi Direct network it specifies."
read -p "Press any key to continue... " -n1 -s
echo ""
echo ""

echo "Setting up..."
cd ~/Library/Android/sdk/platform-tools
./adb kill-server
./adb tcpip 5555

./adb connect 192.168.49.1:5555

echo "Done! :)"
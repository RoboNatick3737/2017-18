#! /bin/bash

echo "First, connect the phone to the USB."
read -p "Press any key to continue... " -n1 -s
echo ""
echo ""

echo "Enter the 'About' submenu of the Robot Controller, and connect to the WiFi Direct network it specifies."
read -p "Press any key to continue... " -n1 -s
echo ""
echo ""

echo "Setting up..."
cd ~/Library/Android/sdk/platform-tools
./adb kill-server
./adb tcpip 5555

./adb connect 192.168.49.1:5555

echo "Now please remove the USB cable from the phone."
read -p "Press any key to continue... " -n1 -s
echo ""
echo ""

./adb connect 192.168.49.1:5555

echo "Done! :)"
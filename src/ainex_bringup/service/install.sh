#!/bin/bash
sudo cp *.service /etc/systemd/system/
cd /etc/systemd/system/
sudo systemctl enable expand_rootfs.service start_app_node.service
sudo systemctl daemon-reload
echo "finish "

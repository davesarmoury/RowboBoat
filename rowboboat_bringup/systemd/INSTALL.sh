sudo cp rowboboat_can_activate.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/rowboboat_can_activate.sh

sudo cp rowboboat_can.service /etc/systemd/system
sudo systemctl enable rowboboat_can.service

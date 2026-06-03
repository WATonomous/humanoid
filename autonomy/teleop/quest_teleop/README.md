# Find ip for the machine running the node
ifconfig | grep "inet " | grep -v 127.0.0.1

# cd
cd ~/wato/humanoid/autonomy/teleop/quest_teleop/certs

# Generate a new private key
openssl genrsa -out key.pem 2048

# Generate a self-signed certificate valid for the new IP
openssl req -new -x509 -key key.pem -out cert.pem -days 365 \
  -subj "/CN=YOUR_IP"
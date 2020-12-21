#!/bin/bash

set -e
set -x

# do not run as root!
# this is just an example, modify as necessary

## rsync the data from an existing host, except for the data:
rsync -avz ntsoi@10.5.161.169:sim_ws $HOME/ --exclude data --exclude video

## copy over the src directory
rsync -avz ntsoi@10.5.161.169:src $HOME/

## copy over lets encrypt, keeping permissions the same:
# on the other host:
#   sudo tar cjf /etc/letsencrypt.tgz /etc/letsencrypt
#   sudo chown ntsoi:ntsoi /etc/letsencrypt.tgz 
rsync -avz ntsoi@10.5.161.169:/etc/letsencrypt.tgz $HOME/
tar xjf letsencrypt.tgz
mv etc/letsencrypt /etc/
sudo chown -R root:root /etc/letsencrypt
rm -rf letsencrypt.tgz

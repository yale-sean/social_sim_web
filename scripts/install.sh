#!/bin/bash

set -e
set -x

apt update

# install nginx, redis
apt install -y nginx redis-server

# install docker
curl -L https://gist.githubusercontent.com/nathantsoi/e668e83f8cadfa0b87b67d18cc965bd3/raw/setup_docker.sh | sudo bash


# as root
#sudo su -
export TURBOVNC_VERSION=2.2.5
export VIRTUALGL_VERSION=2.6.4
export LIBJPEG_VERSION=2.0.5
export WEBSOCKIFY_VERSION=0.9.0
export NOVNC_VERSION=1.2.0

apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gcc \
        libc6-dev \
        libglu1 \
        libsm6 \
        libxv1 \
        libxtst6 \
        make \
        python \
        python-setuptools \
        python-numpy \
        python-imageio \
        x11-xkb-utils \
        xauth \
        xfonts-base \
        xkb-data \
        xfce4 \
        thunar \
        supervisor \
        awscli

# Install turbovnc
curl -fsSL -O https://svwh.dl.sourceforge.net/project/turbovnc/${TURBOVNC_VERSION}/turbovnc_${TURBOVNC_VERSION}_amd64.deb \
        -O https://svwh.dl.sourceforge.net/project/libjpeg-turbo/${LIBJPEG_VERSION}/libjpeg-turbo-official_${LIBJPEG_VERSION}_amd64.deb \
        -O https://svwh.dl.sourceforge.net/project/virtualgl/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb \
        -O https://svwh.dl.sourceforge.net/project/virtualgl/${VIRTUALGL_VERSION}/virtualgl32_${VIRTUALGL_VERSION}_amd64.deb && \
    dpkg -i *.deb && \
    rm -f /tmp/*.deb && \
    sed -i 's/$host:/unix:/g' /opt/TurboVNC/bin/vncserver

# Install novnc
curl -fsSL https://github.com/novnc/noVNC/archive/v${NOVNC_VERSION}.tar.gz | tar -xzf - -C /opt && \
    curl -fsSL https://github.com/novnc/websockify/archive/v${WEBSOCKIFY_VERSION}.tar.gz | tar -xzf - -C /opt && \
    mv /opt/noVNC-${NOVNC_VERSION} /opt/noVNC && \
    mv /opt/websockify-${WEBSOCKIFY_VERSION} /opt/websockify && \
    ln -s /opt/noVNC/vnc_lite.html /opt/noVNC/index.html && \
    cd /opt/websockify && make && mkdir -p lib && mv rebind.so lib/

echo 'no-remote-connections\n\
no-httpd\n\
no-x11-tcp-connections\n\
no-pam-sessions\n\
permitted-security-types = TLSNone,X509None,None\
' > /etc/turbovncserver-security.conf

sudo ~/sim_ws/docker/ros/bootstrap.sh melodic
sudo ~/sim_ws/docker/ros/packages.sh melodic

sudo apt install -y python3-pip

# if this causes a segfault, `rm -rf ~/.local` then run again:
pip3 install wheel
cd ~/sim_ws/yale/social_sim_web/ && pip3 install -r requirements.txt

## rsync the data from an existing host, except for the data:
rsync -avz ntsoi@10.5.161.169:sim_ws $HOME/ --exclude data

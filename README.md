# Social Sim Web

Web wrapper for the social sim web interface

## Host setup

### Clone an existing instance

rsync the data as in (but use your own username/ip):

```
./scripts/clone.sh
```

### If you do not have an instance to clone

checkout the repositories

```
~/sim_ws
~/sim_ws/src/social_sim_ros
~/src/yale/social_sim_web
~/src/yale/social_sim_unity
```

### Build the docker container

```
cd ~/sim_ws
yarn build
yarn up
(ctrl-c)
```

### Installation

Setup the host (install all vnc stuff and ros) with:

```
sudo ./scripts/install.sh
```

To get vgl running with noVNC, first install the ubuntu desktop:

```
sudo apt update
sudo apt install -y tasksel
sudo tasksel install ubuntu-desktop
```

Then configure noVNC:

```
sudo init 3
sudo rmmod nvidia_drm
sudo rmmod nvidia_modeset
sudo rmmod nvidia_uvm
sudo rmmod nvidia
# https://cdn.rawgit.com/VirtualGL/virtualgl/2.6.4/doc/index.html#hd006
sudo /opt/VirtualGL/bin/vglserver_config -config +s +f +t
export DISPLAY=:0
sudo /usr/bin/vglgenkey
xauth merge /etc/opt/VirtualGL/vgl_xauth_key
sudo cp /etc/xdg/xfce4/panel/default.xml /etc/xdg/xfce4/xfconf/xfce-perchannel-xml/xfce4-panel.xml
kill %1
```

Build the environment with

```
source /opt/ros/melodic/setup.bash
cd ~/sim_ws
catkin_make
```

Reboot and you can connect to the VM using TurboVNC at [ip]:5901

Every time a new connection is made, an open port will be found this will be run with the new port:

```
# start a vnc server on a given display with
export PORT=20
export DISPLAY=:${PORT}
export ROS_BRIDGE_PORT=90${PORT}
export ROS_MASTER_URI=http://127.0.0.1:113${PORT}
/opt/websockify/run 59${PORT} --web=/opt/noVNC --wrap-mode=ignore -- /opt/TurboVNC/bin/vncserver $DISPLAY -vgl -securitytypes TLSNone,X509None,None -noxstartup
# start a ros instance with
source ~/sim_ws/devel/setup.bash
roslaunch social_sim_ros jackal_demo_slam_random_dest.launch port:=${ROS_BRIDGE_PORT}
# then start the game
export ROS_BRIDGE_URI=ws://127.0.0.1:90${PORT}
# optional, to show debug messages
#export UNITY_DEBUG=1
vglrun ~/sim_ws/social_sim_unity/Build/SurveyGame.x86_64 -scene Scenes/AgentControlLabScene
```


## Run

You'll need redis

```
sudo apt install redis-server
```

Due to virtualenv conflicts, this must be run with system python

install dependencies with `pip install -r requirements.txt` _or_:

```
pip install flask
pip install uwsgi
pip install flask-redis
pip install celery
```

Run the development server:

```
bin/serve
```

Then open a url like: [http://10.5.161.169:5000/?&scene=lab&avatar=f1&person\_position=SpawnLocation\_Storage1&robot\_position=SpawnLocation\_Storage2&id=1](http://10.5.161.169:5000/?&scene=lab&avatar=f1&person_position=SpawnLocation_Storage1&robot_position=SpawnLocation_Storage2&id=1)

Where `10.5.161.169` can be `localhost` to test on your local computer


## nginx configuration (for production)

Install nginx:

```
sudo apt update
sudo apt install -y nginx
```

Add the application current user to the `www-data` group. This user is currently hard-coded to `ntsoi`.

Make sure the `www-data` user is part of the `docker` group by editing `/etc/group` and ensuring the `docker:...` line includes `www-data`:

`docker:x:999:ntsoi,www-data`

```
usermod -a -G www-data ntsoi
```

Give `www-data` permission to the app:

```
sudo chown -R ntsoi:www-data ~/sim_ws
sudo chmod -R g+rw ~/sim_ws
```

Configure nginx

```
sudo mv /etc/nginx/nginx.conf /etc/nginx/nginx.conf.bk
sudo rm /etc/nginx/sites-enabled/default
sudo mkdir /var/log/ssw && sudo chown -R ntsoi:www-data /var/log/ssw
sudo ln -sf $HOME/src/yale/social_sim_web/configs/nginx/nginx.conf /etc/nginx/nginx.conf
sudo ln -sf $HOME/src/yale/social_sim_web/configs/nginx/ssw.conf /etc/nginx/sites-enabled/ssw.conf
```

add the ssw service

```
sudo ln -sf $HOME/src/yale/social_sim_web/configs/ssw.service /etc/systemd/system/ssw.service
sudo ln -sf $HOME/src/yale/social_sim_web/configs/celery.service /etc/systemd/system/celery.service
```

start and enable the services:

flask:

```
sudo systemctl start ssw
sudo systemctl enable ssw
```

celery background tasks:

```
sudo systemctl start celery
sudo systemctl enable celery
```

check the status to make sure it came up ok, you should see the text "active (running)" in green:

```
sudo systemctl status ssw
```


### Getting and SSL cert

We'll use "Lets encrypt" and Certbot

```
sudo apt install -y python-certbot-nginx
```

```
sudo certbot --nginx -d survey.interactive-machines.com
```

### Setup a crontab to expire old sessions

Run from the project root

Copy `configs/expire.crontab` into `crontab -e` or:

```
(2>/dev/null crontab -l ; cat configs/expire.crontab) | crontab -
```

### Data

Move data to s3:

```
# install awscli once
sudo apt install -y awscli
```

Attach the appropriate role to the instance in the aws console, per https://aws.amazon.com/premiumsupport/knowledge-center/ec2-instance-access-s3-bucket/, then run a copy command. e.g.:

```
aws s3 cp --recursive ~/sim_ws/data_10_24_2020_1 s3://sean-icra2021/pre-survey/data_10_24_2020_1
```

Or even better, do a sync:
```
aws s3 sync ~/sim_ws/data s3://sean-icra2021/pilot2/data
```

### Session Management

To kill a specific port, visit from the vpn: https://10.5.161.169/kill?port=20


### Gotchas

Sometimes the `docker stop ...` command doesn't work, which makes it so that old processes aren't kill after the time limit and we run out of ports.

### Notes

Logging is in `/var/log/nginx/*.log` and `/var/log/ssw/*.log` and `~/sim_ws/data/*/*.log`

The three services you care about are `nginx` (the proxy), `ssw` (the flask app), and `celery` (the background task manager).


### Interactive study data

Interactive session data is stored in `~/sim_ws/data` and the file `server.csv` has the data to tie a specific request to a participant id


### Video study


Once an interactive session is complete, videos for the session can be rendered with:

```
curl -k https://10.5.161.169/render?vid=4b2a34eebe1ddc0001aa35d9
```

where `vid=[participant id]`

this will both render images in all bags for the participant to videos and add the session to the available sessions to be watched


- Available sessions can be check by running (in a `redis-cli`)

```
smembers video_ids
```

- Current user ids assigned to watch a specific interactive users videos (the vid) can be seen with (in a `redis-cli`):

```
keys vid-for-*
```

or to view a specific video participant's assigned id:

```
keys vid-for-[video participant id]
```

which will return the interactive user's id


check in the `~/sim_ws/video` folder for any empty subfolders: `find . -type d -empty` this will tell you if any videos were not able to be generated

for example:

```
find . -type d -empty
./5f5b0f1be585c32184a4cf35/bird
./5f5b0f1be585c32184a4cf35/llama
./5d1ccc7c7e747d0016183e65/dog
./5f4d4ad0fdcc2b1371ceec98/bird
./5f4d53f96aa639a0d605052e/zebr
```

find the bag file that caused this by running:

```
cat /var/log/ssw/uwsgi.log |grep '5f5b0f1be585c32184a4cf35/bird/agent.mp4'
```

Then a url can be embedded in video study, where field 8 is an animal name:

```
var url = "https://survey.interactive-machines.com/rendered?id=${e://Field/id}&animal=${lm://Field/8}"
```

- The logs tying the user id that watched the video of a specific user that generated them is in the folder `~/sim_ws/video/watch.csv`

to get a mapping between video participant IDS and interactive participant IDs:

```
cat watch.csv | awk -F',' '{print $1, $2}'|uniq
```


#### PAUSE the prolific study!

Be sure to expand the study to the number of required slots, wait for the slots to fill, then PAUSE the study, so that if someone returns their study before completion, you can add re-queue the interactive ID before another participant joins or you will run out of available interactive ids for the video participants to review!

Re-queue the interactive ID by searching for the video participant's ID in watch.csv, which correlates the interactive ID to the video participant ID.

Then run:

```
curl -k https://10.5.161.169/render?vid=4b2a34eebe1ddc0001aa35d9
```

Where `4b2a34eebe1ddc0001aa35d9` is the interactive participant ID to re-queue.



### Development Setup

Install with [pipenv](https://pypi.org/project/pipenv/) and [pyenv](https://github.com/pyenv/pyenv#basic-github-checkout):

e.g. for Ubuntu 18.04:

```
# install pipenv
sudo apt install -y python3-pip
python3 -m pip install --user pipenv
echo 'export PATH="${HOME}/.local/bin:$PATH"' >> ~/.bash_profile

# install pyenv
sudo apt-get install -y make build-essential libssl-dev zlib1g-dev libbz2-dev \
libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev \
xz-utils tk-dev libffi-dev liblzma-dev python-openssl git
git clone https://github.com/pyenv/pyenv.git ~/.pyenv
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bash_profile
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bash_profile

# update env
source ~/.bash_profile
```

Then install dependencies from the project root:

```
pipenv install
```

Run with

```
pipenv run bin/serve
```

#### Debugging with rivz

```
export PORT=20
export DISPLAY=:${PORT}
export ROS_MASTER_URI=http://127.0.0.1:113${PORT}
source ~/sim_ws/devel/setup.bash
xfce4-session &
vglrun rviz -d $(rospack find social_sim_ros)/config/kuri_move.rviz &
```

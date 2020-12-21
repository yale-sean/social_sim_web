#!/usr/bin/env python3

import os
import hashlib
import random
import shlex
import socket
import time
import datetime
import csv
import random
import json
import glob

# port and datetime now object the port was last used (initialized via freeport)
#processes = {}
REDIS_URL = "redis://127.0.0.1:6379/0"

PORT_START = 20
MAX_INSTANCES = 10

# 5 min
TIME_LIMIT_MIN = 5
TIME_LIMIT_SEC = 60*TIME_LIMIT_MIN

DATA_DIR = os.path.expanduser('~/sim_ws/data')
VIDEO_DIR = os.path.expanduser('~/sim_ws/video')
CSV_LOG_FILE = os.path.join(DATA_DIR, 'server.csv')
VIDEO_CSV_LOG_FILE = os.path.join(VIDEO_DIR, 'watch.csv')

SCENES = {
    'lab': 'Scenes/AgentControlLabScene',
    'warehouse': 'Scenes/AgentControlSmallWarehouseScene'
}
AVATARS = {
    'f1': 'Female_Adult_01',
    'f1': 'Female_Adult_02',
    'f1': 'Female_Adult_03',
    'f1': 'Female_Adult_04',
    'm1': 'Male_Adult_01',
    'm1': 'Male_Adult_02',
    'm1': 'Male_Adult_03',
    'm1': 'Male_Adult_04'
}

lockfile_suffixes = [
    '_vnc',
    '_uni'
]

from flask import Flask, request, redirect, render_template, send_file
from flask_redis import FlaskRedis
from celery import Celery

def make_celery(app):
    celery = Celery(app.import_name, backend=app.config['CELERY_RESULT_BACKEND'],
                    broker=app.config['CELERY_BROKER_URL'])
    celery.conf.update(app.config)
    TaskBase = celery.Task
    class ContextTask(TaskBase):
        abstract = True
        def __call__(self, *args, **kwargs):
            with app.app_context():
                return TaskBase.__call__(self, *args, **kwargs)
    celery.Task = ContextTask
    return celery

#app
app = Flask(__name__, template_folder='../templates')
# redis
redis_client = FlaskRedis(app, decode_responses=True)
# celery
app.config.update(
    CELERY_BROKER_URL='redis://localhost:6379/1',
    CELERY_RESULT_BACKEND='redis://localhost:6379/1'
)
celery = make_celery(app)

import logging
logFormatStr = '[%(asctime)s] p%(process)s {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s'
logging.basicConfig(format = logFormatStr, filename = "log/global.log", level=logging.DEBUG)
formatter = logging.Formatter(logFormatStr,'%m-%d %H:%M:%S')
fileHandler = logging.FileHandler("log/server.log")
fileHandler.setLevel(logging.DEBUG)
fileHandler.setFormatter(formatter)
streamHandler = logging.StreamHandler()
streamHandler.setLevel(logging.DEBUG)
streamHandler.setFormatter(formatter)
app.logger.addHandler(fileHandler)
app.logger.addHandler(streamHandler)
app.logger.info("Logging is set up.")

from urllib.parse import urlparse

def killport(port):
    ''' kill processes associated with a given port '''
    # kill the ros docker container
    cmd = f"/usr/bin/docker ps -f name=sim_{port}_ros" + "|/usr/bin/awk '{print $1}'|/usr/bin/tail -n1"
    app.logger.info(cmd)
    container_id = os.popen(cmd).read().strip()
    app.logger.info(f"cmd: '{cmd}', id: '{container_id}'")
    if container_id and container_id != 'CONTAINER':
        # blocks up to 10s, but with 1s, it seems to hang?
        docker_stop_flock = f"/tmp/docker_stop"
        cmd = f"/usr/bin/flock -w 15 {docker_stop_flock} /usr/bin/docker stop --time 4 {container_id}"
        app.logger.info(cmd)
        container_id = os.system(cmd)
    # kill game and vnc
    for suffix in lockfile_suffixes:
        flockf = f"/tmp/sim_{port}"+suffix
        app.logger.info(f"checking {flockf}")
        if os.path.exists(flockf):
            cmd = f"/bin/fuser -k {flockf}; /bin/rm {flockf}"
            app.logger.info(cmd)
            os.system(cmd)
    xlock = f"/tmp/.X{port}-lock"
    xunix= f"/tmp/.X11-unix/X{port}"
    try:
        os.unlink(xlock)
    except FileNotFoundError:
        pass
    try:
        os.unlink(xunix)
    except FileNotFoundError:
        pass
    # last to unlock the port once all files are cleared
    redis_del_port(port)


def redis_get_port_by_token(token):
    ''' lookup port and timeout by token '''
    s = redis_client.get("token6-{}".format(token))
    if not s: 
        return None, None, None
    d = json.loads(s)
    return d['port'], d['id'], d['token6']

def redis_get_port_by_id(id):
    ''' lookup port and timeout by user id '''
    s = redis_client.get("user_id-{}".format(id))
    if not s: 
        return None, None, None
    d = json.loads(s)
    return d['port'], d['id'], d['token6']

def redis_get_port_by_port(port):
    ''' lookup port and timeout by user port '''
    s = redis_client.get("port-{}".format(port))
    if not s: 
        return None, None, None
    d = json.loads(s)
    return d['port'], d['id'], d['token6']

def redis_set_port(port, id, token6):
    ''' assign a port to a user id '''
    j = json.dumps({'port': port, 'id': id, 'token6': token6})
    pipe = redis_client.pipeline()
    pipe.set("port-{}".format(port), j, ex=TIME_LIMIT_SEC)
    pipe.set("user_id-{}".format(id), j, ex=TIME_LIMIT_SEC)
    pipe.set("token6-{}".format(token6), j, ex=TIME_LIMIT_SEC)
    pipe.execute()

def redis_del_port(port):
    port, id, token = redis_get_port_by_port(port)
    redis_client.delete("port-{}".format(port),
                        "user_id-{}".format(id),
                        "token6-{}".format(token))

def freeport(id, token6):
    ''' find a free port for a given user id '''
    port = None
    # only allow 1 processes per id
    port, user_id, _ = redis_get_port_by_id(id)
    if port:
        app.logger.info("killing port '{}' for user '{}'".format(port, user_id))
    else:
        # look for unused ports
        for i in range(PORT_START, PORT_START+MAX_INSTANCES):
            p, _, _ = redis_get_port_by_port(i)
            if not p:
                app.logger.info("found free port '{}' for user '{}'".format(i, id))
                port = i
                break

    # re-use port by updating timestamp
    if port is not None:
        killport(port)
        redis_set_port(port, id, token6)

    # port if found, otherwise, none
    return port

def escape(request):
    # note: these params _MUST_ be validated and escaped to avoid shell injection
    # required
    id = shlex.quote(request.args.get('id'))
    scene = shlex.quote(request.args.get('scene'))
    avatar = shlex.quote(request.args.get('avatar'))
    robot_position = shlex.quote(request.args.get('robot_position'))
    person_position = shlex.quote(request.args.get('person_position'))
    show_token = shlex.quote(request.args.get('t1'))
    show_token_timeout = shlex.quote(request.args.get('t2'))

    # optional
    force_reload = request.args.get('force_reload')
    video = shlex.quote(request.args.get('video'))
    video_token = shlex.quote(request.args.get('video_token'))
    if video_token == "''":
        video_token = ''
    intro = False
    if request.args.get('intro') == '1':
        intro = True
    avatar_seed = ''
    if request.args.get('avatar_seed') is not None and request.args.get('avatar_seed').isdigit():
        avatar_seed = int(request.args.get('avatar_seed'))

    # all params must be included
    if id == "''" or scene == "''" or avatar == "''" or robot_position == "''" or person_position == "''":
        app.logger.warn(f"Not all params. id: {id}, scene: {scene}, avatar: {avatar}")
        return (False, 'Invalid')

    app.logger.info(f"All params. id: {id}, scene: {scene}, avatar: {avatar}")

    # TODO: pre-validate and ensure a matching ID
    if not scene in SCENES.keys():
        app.logger.warn(f"Scene: {scene} is not in {SCENES}")
        return (False, 'Invalid Scene')
    ros_scene = scene
    if ros_scene == 'warehouse':
        ros_scene = 'warehouse_small'
    unity_scene = SCENES[scene]
    if not avatar in AVATARS.keys():
        app.logger.warn(f"Avatar: {avatar} is not in {AVATARS}")
        return (False, 'Invalid Avatar')
    unity_avatar = AVATARS[avatar]

    seconds = time.time()
    #hashable = '-'.join([id, scene, avatar, robot_position, person_position, str(seconds)])
    hashable = '-'.join([id, scene, avatar, robot_position, person_position, str(intro), str(avatar_seed)])
    token = hashlib.sha1(hashable.encode("UTF-8")).hexdigest()
    token6 = token[:6]
    datapath = f"$HOME/sim_ws/data/{token6}"

    keys = ['timestamp', 'path', 'id', 'scene', 'ros_scene', 'unity_scene', 'avatar', 'unity_avatar', 'robot_position', 'person_position', 'intro', 'avatar_seed', 'token', 'token6', 'show_token', 'show_token_timeout', 'datapath', 'video', 'video_token', 'force_reload']
    vals = [seconds, request.path, id, scene, ros_scene, unity_scene, avatar, unity_avatar, robot_position, person_position, intro, avatar_seed, token, token6, show_token, show_token_timeout, datapath, video, video_token, force_reload]
    

    write_header = True
    try:
        write_header = not (os.path.exists(CSV_LOG_FILE) and os.path.getsize(CSV_LOG_FILE) > 0)
    except FileNotFoundError:
        pass
    with open(CSV_LOG_FILE, 'a+') as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(keys)
        w.writerow(vals)

    return (True, dict(zip(keys,vals)))


@app.route('/video')
def video():
    ''' creates a video on the fly for a given user or video token6 '''
    result, ctx = escape(request)
    # validate args
    if not result:
        return ctx

    # this will shutdown the whole interactive session
    killport(ctx['id'])

    # choose the token
    token = ctx['video_token']
    if ctx['video_token'] == '':
        token = ctx['token6']
    ctx['video_token'] = token

    # stream video
    if ctx['video'] in ['agent', 'player']:
        video_exists = False

        data_path = os.path.join(DATA_DIR, token)
        video_path = os.path.join(data_path, f"{ctx['video']}.mp4")
        try:
            video_exists = os.path.exists(video_path) and os.path.getsize(video_path) > 0
        except FileNotFoundError:
            pass
        if not video_exists:
            cmd = f"/bin/bash -c 'export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin && source $HOME/sim_ws/devel/setup.bash && $HOME/sim_ws/src/social_sim_ros/social_sim_ros/scripts/make_video.sh {token} {ctx['video']} >> {data_path}/video.log 2>&1'"
            app.logger.info(f"making videos for {token}: {cmd}")
            os.system(cmd)
        app.logger.info(f"sending: {video_path}, exists: {video_exists}")
        return send_file(video_path, as_attachment=True)

    # render template
    return render_template('video.html', **ctx)


def redis_add_video_user_id(vid):
    ''' takes a video user id (vid) and adds them to the available videos to watch'''
    redis_client.sadd('video_ids', str(vid))

def redis_get_vid_for_user(id):
    ''' takes a user id (id) and and returns a video id for the videos they should watch or none if there are no videos to watch'''
    assigned_key = f'vid-for-{id}'
    assigned = redis_client.get(assigned_key)
    if assigned:
        return assigned
    # pop then set to ensure only 1 user gets this vid
    vid = redis_client.spop('video_ids')
    if not vid:
        return None
    redis_client.set(assigned_key, vid)
    return vid


def get_tokens_and_animals(id):
    ''' returns a dict of {token: animal}
       where
         token is token6
         animal is just the animal name in show_token'''
    tokens = {}
    with open(CSV_LOG_FILE, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['id'] == str(id):
                animal = row['show_token'].split('-')[-1]
                tokens[row['token6']] = animal
    if len(tokens.keys()) < 1:
        return None
    return tokens


@app.route('/render')
def render():
    ''' renders a series of videos for a given user id
        takes the id of the user we want to generate videos for
    '''
    if not management(request):
        return '404'

    vid = request.args.get('vid')
    if not vid or vid == '':
        return 'Invalid, please specify which participant id to render with the vid param'

    video = 'agent'
    rendered = ['rendered:']
    exists = ['rendered:']
    token_to_animal = get_tokens_and_animals(vid)
    if not token_to_animal:
        return 'Not found'
    for token, animal in token_to_animal.items():
        datapath = os.path.join(DATA_DIR, token)
        bagpath = os.path.join(datapath, '*.bag')
        animalpath = os.path.join(VIDEO_DIR, vid, animal)
        os.makedirs(animalpath, exist_ok=True)
        videopath = os.path.join(animalpath, f"{video}.mp4")
        video_exists = False
        try:
            video_exists = os.path.exists(videopath) and os.path.getsize(videopath) > 0
        except FileNotFoundError:
            pass
        if not video_exists:
            flockf = f"/tmp/makevideo-{vid}-{animal}-{video}"
            cmd = f"/usr/bin/flock -w 1 {flockf} /bin/bash -c 'export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin && source $HOME/sim_ws/devel/setup.bash && rosrun bag2video bag2video.py '{bagpath}' /thirdpersonview/compressed --outfile '{videopath}' --height 480 --width 640 --stop_topic /actor_goal_status >> {datapath}/video.log 2>&1'"
            app.logger.info(cmd)
            os.system(cmd)
            rendered.append(videopath)
        else:
            exists.append(videopath)

    if len(token_to_animal) != 7:
        return f'Incorrect number of videos: {token_to_animal}'

    # queue the video id for watching
    redis_add_video_user_id(vid)

    return '<br/>'.join(rendered) + '<br/><br/>' + '<br/>'.join(exists)

@app.route('/rendered')
def rendered():
    ''' shows a series of videos for a new userid '''

    animal = request.args.get('animal')
    if not animal or animal == '' or animal not in ['bird','cat','dog','llama','panda','zebra']:
        return 'Invalid animal'

    # get the user id for the series of videos we want to show
    # these must be pre-populated in redis by running '/render?id=[PROLIFIID]'
    # this way we only have each person watch the set of videos for another person once
    id = request.args.get('id')
    if not id or id == '':
        return 'Invalid id'
    vid = redis_get_vid_for_user(id)

    # when set to 'agent', returns the video
    video = request.args.get('video')

    # log
    keys = ['id', 'vid', 'animal', 'video']
    vals = [id, vid, animal, video]
    os.makedirs(VIDEO_DIR, exist_ok=True)
    write_header = True
    try:
        write_header = not (os.path.exists(VIDEO_CSV_LOG_FILE) and os.path.getsize(VIDEO_CSV_LOG_FILE) > 0)
    except FileNotFoundError:
        pass
    with open(VIDEO_CSV_LOG_FILE, 'a+') as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(keys)
        w.writerow(vals)

    if not vid:
        return 'No more videos'

    if video in ['agent']:
        # to watch a specific video, just override vid=, e.g.:
        # https://survey.interactive-machines.com/rendered?id=nathan2&animal=dog&video=agent&vid=5ea5dd6605b5170963934a65
        vid_override = request.args.get('vid')
        if vid_override:
            vid = vid_override
        video_path = os.path.join(VIDEO_DIR, vid, animal, f"{video}.mp4")
        video_exists = False
        try:
            video_exists = os.path.exists(video_path) and os.path.getsize(video_path) > 0
        except FileNotFoundError:
            pass
        if not video_exists:
            return 'Not found'

        return send_file(video_path)

    ctx = {
        'id': id,
        'animal': animal,
        'video': 'agent'
    }

    return render_template('rendered.html', **ctx)


def to_vnc(request, web_port):
    web_port = int(web_port)
    # redirect
    o = urlparse(request.base_url)
    host = o.hostname
    # w/o nginx
    if request.environ.get('REMOTE_PORT') == '5000':
        return redirect(f"https://{host}:{web_port}")
    # w/ nginx
    else:
        return redirect(f"https://{host}/game/{web_port}/game.html?path=game%2f{web_port}=true")

@celery.task(bind=True, soft_time_limit=TIME_LIMIT_SEC)
def boot(self, ctx=None, request_url=None):
    port = freeport(ctx['id'], ctx['token6'])
    web_port = f"59{port}"
    ros_bridge_port = f"90{port}"
    ros_bridge_uri = f"ws://127.0.0.1:{ros_bridge_port}"

    datapath = os.path.expandvars(ctx['datapath'])
    os.makedirs(datapath, exist_ok=True)
    app.logger.info(f"created {datapath}")

    with open(os.path.join(datapath, 'link'), 'w') as f:
        f.write(request_url)

    # vnc server
    vnc_flock = f"/tmp/sim_{port}_vnc"
    vnc_cmd = f"export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin && /usr/bin/flock {vnc_flock} /opt/websockify/run {web_port} --web=/opt/noVNC --wrap-mode=ignore -- /opt/TurboVNC/bin/vncserver :{port} -vgl -securitytypes TLSNone,X509None,None -noxstartup >> {ctx['datapath']}/vnc.log 2>&1 &"
    app.logger.info(f"vnc_cmd: {vnc_cmd}")
    os.system(vnc_cmd)
    self.update_state(state='PROGRESS', meta={'current': 10, 'total': 100, 'status': "booted vnc for '{}' on '{}'".format(ctx['id'], web_port)})

    time.sleep(2)
    
    # ros 
    # To run this container, first run `cd ~/sim_ws && yarn build` then hit ctrl-c
    ros_cmd = f'''/usr/bin/docker run -p 90{port}:90{port} -p 113{port}:113{port} -v $HOME/sim_ws:$HOME/sim_ws --name sim_{port}_ros --rm --user $USER -i ros-shell:social-sim "/bin/bash -c 'source $HOME/sim_ws/devel/setup.bash && export DISPLAY=:{port} && export ROS_MASTER_URI=http://127.0.0.1:113{port} && export ROS_HOSTNAME=localhost && ROS_IP=127.0.0.1 && roslaunch social_sim_ros kuri_survey.launch port:=90{port} datapath:={ctx['datapath']} duration:={TIME_LIMIT_MIN}m scene:={ctx['ros_scene']}'" >> {ctx['datapath']}/ros.log 2>&1 &'''
    app.logger.info(f"ros_cmd: {ros_cmd}")
    os.system(ros_cmd)
    self.update_state(state='PROGRESS', meta={'current': 20, 'total': 100, 'status': "booted ros for '{}' on '{}'".format(ctx['id'], web_port)})

    # wait for the rosbridge to come up
    time.sleep(3)
    os.system(f"while ! echo exit | /bin/nc localhost 90{port}; do /bin/sleep 2; done")
    time.sleep(3)

    # game
    # To run this container, first run `cd ~/sim_ws && yarn game-build` then hit ctrl-c
    uni_flock = f"/tmp/sim_{port}_uni"
    show_intro = '0'
    if ctx['intro'] is True:
        show_intro = '1'
    avatar_seed = ''
    if ctx['avatar_seed'] != '':
        avatar_seed = f"-avatar_seed {ctx['avatar_seed']}"
    uni_cmd = f"""export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin && export DISPLAY=:{port} && export ROS_BRIDGE_URI={ros_bridge_uri} && export INTRO={show_intro} && /usr/bin/flock {uni_flock} /usr/bin/vglrun $HOME/sim_ws/social_sim_unity/Build/SurveyGame.x86_64 -scene {ctx['unity_scene']} -avatar {ctx['unity_avatar']} -token {ctx['show_token']} -token_timeout {ctx['show_token_timeout']} -person_position "{ctx['person_position']}" -robot_position {ctx['robot_position']} {avatar_seed} >> {ctx['datapath']}/unity.log 2>&1 &"""
    app.logger.info(f"uni_cmd: {uni_cmd}")
    os.system(uni_cmd)
    self.update_state(state='PROGRESS', meta={'current': 30, 'total': 100, 'status': "booted game for '{}' on '{}'".format(ctx['id'], web_port)})

@app.route('/')
def scene_manager():
    result, ctx = escape(request)
    if not result:
        return ctx

    # handle reloads of the page, send the user to the correct port if the request is already running
    running_port, user_id, _ = redis_get_port_by_token(ctx['token6'])
    if ctx['force_reload'] != '1' and running_port is not None:
        app.logger.info(f"redirecting existing session: '{ctx}' on port '{running_port}' for user '{user_id}'")
        web_port = f"59{running_port}"
        return to_vnc(request, web_port)

    datapath = os.path.expandvars(ctx['datapath'])
    if os.path.exists(datapath):
        app.logger.info(f"already complete: {ctx}")
        return 'Already complete'

    # get the next free port
    port = freeport(ctx['id'], ctx['token6'])
    web_port = f"59{port}"

    if port is None:
        app.logger.info(f"no ports {ctx}")
        return 'No ports'
    app.logger.info(f"using port {port}")

    # boot in the background and let the browser start to load the vnc interface
    # request expires in 15 seconds (is marked revoked if not started w/ in 15 seconds)
    task = boot.apply_async(kwargs={'ctx': ctx, 'request_url': request.url}, expires=15)

    # wait for vnc to come up
    os.system(f"while ! echo exit | /bin/nc localhost 59{port}; do /bin/sleep 1; done")

    return to_vnc(request, web_port)

def management(request):
    ''' only allow access to routes with this method from an internal ip '''
    o = urlparse(request.base_url)
    host = o.hostname
    if host in ['localhost', '127.0.0.1']:
        return True
    if host.startswith('10.5.161.'):
        return True
    return False

@app.route('/expire')
def expire_ports():
    ''' expire unused ports '''
    if not management(request):
        return '404'
    # for each vnc lock file
    expired = ['expired:']
    locks = list(glob.iglob("/tmp/sim_*_vnc"))
    app.logger.info(f"checking: {locks}")
    for f in locks:
        port = f.split('/sim_')[-1].split('_vnc')[0]
        app.logger.info(f"checking port '{port}'")
        # if it's not in redis, kill it
        p, _, _ = redis_get_port_by_port(port)
        if not p:
            app.logger.info(f"killing port '{port}'")
            killport(port)
            expired.append(port)
    return '<br/>'.join(expired)

@app.route('/kill')
def kill_port():
    ''' kill all processes '''
    if not management(request):
        return '404'
    port = request.args.get('port')
    if port == 'all':
        for port in range(PORT_START, PORT_START+MAX_INSTANCES):
            killport(port)
        return f'killed all'
    if str(int(port)) == str(port):
        killport(int(port))
        return f'killed {int(port)}'
    return 'na'

# WSGI entrypoint
if __name__ == "__main__":
    app.run()

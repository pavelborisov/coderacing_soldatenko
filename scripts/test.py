from __future__ import print_function
import os
import subprocess
import time

maps = ['default', 'map01', 'map02', 'map03', 'map04', 'map05', 'map06', 'map07', 'map08',
 'map09', 'map10', 'map11', 'map12', 'map13', 'map14', 'map15', 'map16', 'map17', 'map18',
 'map19', 'map20', 'map21', '_fdoke', '_tyamgin', '_ud1']
#maps = ['default', 'map01']
players = [
    ['current', '..\\CodeRacing\\ReleaseNoLogging\\cpp-cgdk_vs12.exe'],
    #['previous', '..\\ReleaseNoLogging\\cpp-cgdk_vs12.exe']
]
local_runner_path = '..\\local-runner\\local-runner.jar'

properties_template = """\
render-to-screen=false
render-to-screen-sync=false
render-to-screen-size=1680x1050
render-to-screen-tick=
results-file={4}
log-file={5}
team-size=2
player-count=2
p1-type=Local
p2-type={3}
p3-type=Empty
p4-type=Empty
p1-name={1}
p2-name={2}
p3-name=
p4-name=
swap-car-types=false
disable-car-collision=false
map={0}
base-adapter-port=31101
seed=0
plugins-directory=..\\local-runner\\plugins
"""

def create_properties(mapname, p1name, p2name, p2type, result_filename, log_filename):
    filename = '{}_{}_{}.properties'.format(mapname, p1name, p2name, p2type)
    f = open(filename, 'w')
    f.write(properties_template.format(mapname, p1name, p2name, p2type, result_filename, log_filename))
    f.close()
    return filename

def analyze_log(log_filename):
    f = open(log_filename, 'r')
    finished_ticks = [-1, -1, -1, -1]
    player_scores = [-1, -1]
    player_crashed = [-1, -1]
    car_id_map = {}
    for line in f.readlines():
        data = eval(line.replace('false', 'False').replace('true', 'True'))
        tick = data['tick']
        for car in data['cars']:
            if 'playerId' in car:
                car_id_map[car['id']] = 2 * int(car['playerId'] - 1) + (0 if car['type'] == 'BUGGY' else 1)
            if 'finishedTrack' in car and car['finishedTrack']:
                car_id = car_id_map[car['id']]
                finished_ticks[car_id] = min(finished_ticks[car_id], tick) if finished_ticks[car_id] > 0 else tick
        for player in data['players']:
            player_id = int(player['id'] - 1)
            if 'score' in player:
                player_scores[player_id] = player['score']
            if 'strategyCrashed' in player and player['strategyCrashed']:
                player_crashed[player_id] = min(player_crashed[player_id], tick) if player_crashed[player_id] > 0 else tick
    f.close()
    return player_scores, finished_ticks, player_crashed

def run_game_single(mapname, p1name, p1exec):
    result_filename = '{}.result'.format(mapname)
    log_filename = '{}.log'.format(mapname)
    properties_filename = create_properties(mapname, p1name, 'empty', 'Empty', result_filename, log_filename)
    print('Map: ', mapname)
    local_runner_p = subprocess.Popen(['javaw', '-jar', local_runner_path, properties_filename], shell=True)
    time.sleep(1)
    p1_p = subprocess.Popen([p1exec, '127.0.0.1', '31101', '0000000000000000'])
    start = time.clock()
    local_runner_p.wait()
    p1_p.wait()
    end = time.clock()
    print('Time: ', end - start)
    player_scores, finished_ticks, player_crashed = analyze_log(log_filename)
    print('Player scores: ', player_scores)
    print('Car finish ticks: ', finished_ticks)
    print('Player crashed ticks: ', player_crashed)
    print('------')

def run_game_duel(mapname, p1name, p1exec, p2name, p2exec):
    result_filename = '{}_{}_{}.result'.format(mapname)
    log_filename = '{}_{}_{}.log'.format(mapname)
    properties_filename = create_properties(mapname, p1name, p2name, 'Local')
    print('Map, P1Name, P2Name: ', mapname, p1name, p2name)
    local_runner_p = subprocess.Popen(['javaw', '-jar', local_runner_path, properties_filename], shell=True)
    time.sleep(1)
    p1_p = subprocess.Popen([p1exec, '127.0.0.1', '31101', '0000000000000000'])
    time.sleep(1)
    p2_p = subprocess.Popen([p2exec, '127.0.0.1', '31102', '0000000000000000'])
    start = time.clock()
    local_runner_p.wait()
    p1_p.wait()
    p2_p.wait()    
    end = time.clock()
    print('Time: ', end - start)
    player_scores, finished_ticks, player_crashed = analyze_log(log_filename)
    print('Player scores: ', player_scores)
    print('Car finish ticks: ', finished_ticks)
    print('Player crashed ticks: ', player_crashed)
    print('------')

def test():
    for mapname in maps:
        if(len(players) == 2):
            run_game_duel(mapname, players[0][0], players[0][1], players[1][0], players[1][1])
            run_game_duel(mapname, players[1][0], players[1][1], players[0][0], players[0][1])
        else:
            run_game_single(mapname, players[0][0], players[0][1])

test()
        
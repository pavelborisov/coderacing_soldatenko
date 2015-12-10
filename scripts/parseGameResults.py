from __future__ import print_function
import os
import numpy

myname = 'SKolotienko'

data = []
for fname in os.listdir("."):
    if not 'gameresults' in fname:
        continue
    f = open(fname, 'r')
    fdata = eval(f.read())
    f.close()
    data += fdata

results = {}
players = []
for winner, loser in data:
    players += [winner]
    players += [loser]
    if not winner in results:
        results[winner] = {}
    if not loser in results[winner]:
        results[winner][loser] = 0
    results[winner][loser] += 1
players = sorted(numpy.unique(players))
#print(players)

f = open('gamestable.csv', 'w')
header = "MyName;EnemyName;TotalGames;Won;Lost;Percentage"
print(header)   
f.write(header + "\n")
for p in players:
    if p == myname:
        continue
    won = 0
    if p in results[myname]:
        won = results[myname][p]
    lost = 0
    if p in results and myname in results[p]:
        lost = results[p][myname]
    total = won + lost
    percentage = "NaN"
    if total > 0:
        percentage = str(int(100 * won / total)) + "%"
    s = myname + ";" + p + ";" + str(total) + ";" + str(won) + ";" + str(lost) + ";" + percentage
    print(s)
    f.write(s + "\n")
print()
f.write("\n")
    
header = ""
for i in range(0, len(players)):
    header += ";"
    header += players[i]
print(header)
f.write(header + "\n")
for i in range(0, len(players)):
    player1 = players[i]
    s = player1
    for j in range(0, len(players)):
        s += ";"
        if i == j:
            s += 'x'
        player2 = players[j]
        if(player1 in results and player2 in results[player1]):
            s += str(results[player1][player2])
    print(s)
    f.write(s + "\n")
f.close()
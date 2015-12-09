function processRow(row) {
    var gametype = row.cells[1].innerHTML.trim();
    if(gametype != "2×2") {
        return;
    }
    var stat = row.cells[6].innerHTML.trim();
    if(stat == "Игра тестируется") {
        return;
    }
    
    var winnerName = row.cells[4].childNodes[3].firstChild.innerHTML;
    var loserName = row.cells[4].childNodes[9].firstChild.innerHTML;
    console.log(winnerName + " > " + loserName);
}

var t = document.getElementsByClassName("gamesTable")[0];
for(var i = 1, row; row = t.rows[i]; i++) {
    processRow(row);
}
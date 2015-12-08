//To use with "Custom JavaScript for websites" extension for Chrome
var myName = "SKolotienko";

function createGame() {
    window.open("http://russianaicup.ru/game/create", "_self")
}

function fillGame() {
    var enemyList = ["Mr.Smile", "ud1", "Antmsu", "Angor"];
    var enemyName = enemyList[Math.floor(Math.random() * enemyList.length)];
    var myPosition = Math.floor(Math.random() * 2) + 1;
    var enemyPosition = 3 - myPosition;
    document.getElementById("participant".concat(myPosition)).value = myName;
    document.getElementById("participant".concat(enemyPosition)).value = enemyName;
}

function submitGame() {
    document.getElementsByClassName("form-horizontal")[0].submit();
}

if (window.location.href == "http://russianaicup.ru/games/creator/".concat(myName)) {
    setTimeout(createGame, 160 * 1000);
} else if (window.location.href == "http://russianaicup.ru/game/create") {
    fillGame();
    setTimeout(submitGame, 1000);
}
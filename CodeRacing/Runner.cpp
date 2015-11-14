#include "Runner.h"

#include <vector>
#include <Windows.h>

#include "MyStrategy.h"

using namespace model;
using namespace std;

static void startServer()
{
	const wchar_t* const localRunnerDirectory = L"..\\local-runner";
	const wchar_t* const localRunnerBat = L"local-runner-debug.bat";
	ShellExecute(NULL, NULL, localRunnerBat, NULL, localRunnerDirectory, SW_HIDE);
	Sleep(500);
}

int main(int argc, char* argv[]) {
    if (argc == 4) {
        Runner runner(argv[1], argv[2], argv[3]);
        runner.run();
    } else {
		startServer();
        Runner runner("127.0.0.1", "31001", "0000000000000000");
        runner.run();
    }
    
    return 0;
}

Runner::Runner(const char* host, const char* port, const char* token)
        : remoteProcessClient(host, atoi(port)), token(token) {
}

void Runner::run() {
    remoteProcessClient.writeTokenMessage(token);
    int teamSize = remoteProcessClient.readTeamSizeMessage();
    remoteProcessClient.writeProtocolVersionMessage();
    Game game = remoteProcessClient.readGameContextMessage();

    vector<Strategy*> strategies;

    for (int strategyIndex = 0; strategyIndex < teamSize; ++strategyIndex) {
        Strategy* strategy = new MyStrategy;
        strategies.push_back(strategy);
    }

    PlayerContext* playerContext;

    while ((playerContext = remoteProcessClient.readPlayerContextMessage()) != NULL) {
        vector<Car> playerCars = playerContext->getCars();
        if ((int) playerCars.size() != teamSize) {
            break;
        }

        vector<Move> moves;

        for (int carIndex = 0; carIndex < teamSize; ++carIndex) {
            Car playerCar = playerCars[carIndex];

            Move move;
            strategies[playerCar.getTeammateIndex()]->move(playerCar, playerContext->getWorld(), game, move);
            moves.push_back(move);
        }

        remoteProcessClient.writeMovesMessage(moves);

        delete playerContext;
    }

    for (int strategyIndex = 0; strategyIndex < teamSize; ++strategyIndex) {
        delete strategies[strategyIndex];
    }
}

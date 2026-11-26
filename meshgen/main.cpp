//
// Main.cpp
//

#include "meshgen/Application.h"
#include "meshgen/HeadlessCli.h"

#include <Windows.h>

int main(int argc, char* argv[])
{
	if (argc > 1)
	{
		const std::string command = argv[1];
		if (IsHeadlessCommand(command))
			return RunHeadlessCli(argc, argv);
	}

	// The exe is a console program so shells (PowerShell included) capture CLI output. When the
	// GUI is started from Explorer, Windows gives it a console of its own; let that one go.
	DWORD consoleProcesses[2];
	if (GetConsoleProcessList(consoleProcesses, 2) == 1)
		FreeConsole();

	std::string startingZone;
	if (argc > 1)
		startingZone = argv[1];

	Application window(startingZone);
	return window.RunMainLoop();
}

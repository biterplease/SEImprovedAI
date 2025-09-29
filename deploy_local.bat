@echo off
rmdir /S /Q "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAIConstructor" 2>NUL
mkdir "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAIConstructor" 2>NUL
mkdir "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAIConstructor\Data" 2>NUL

xcopy "C:\Users\dangu\source\repos\BetterAIConstructor\BetterAIConstructor\modinfo.sbm" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAIConstructor\" /Y /Q 2>NUL
REM xcopy "C:\Users\dangu\source\repos\BetterAI_Constructor\BetterAI_Constructor\*.sbc" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAI_Constructor\" /S /Y /Q 2>NUL
xcopy "C:\Users\dangu\source\repos\BetterAIConstructor\BetterAIConstructor\Data" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\BetterAIConstructor\Data\" /S /Y /Q 2>NUL
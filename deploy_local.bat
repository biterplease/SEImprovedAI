@echo off
rmdir /S /Q "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI" 2>NUL
mkdir "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI" 2>NUL
mkdir "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI\Data" 2>NUL

xcopy "C:\Users\dangu\source\repos\ImprovedAI\ImprovedAI\modinfo.sbm" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI\" /Y /Q 2>NUL
xcopy "C:\Users\dangu\source\repos\ImprovedAI\ImprovedAI\ImprovedAI.ini" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI\" /Y /Q 2>NUL
xcopy "C:\Users\dangu\source\repos\ImprovedAI\ImprovedAI\Data" "C:\Users\dangu\AppData\Roaming\SpaceEngineers\Mods\ImprovedAI\Data\" /S /Y /Q 2>NUL
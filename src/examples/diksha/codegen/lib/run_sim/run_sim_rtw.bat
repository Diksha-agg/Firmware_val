@echo off

set MATLAB=F:\matlab

cd .

if "%1"=="" (F:\matlab\bin\win64\gmake -f run_sim_rtw.mk all) else (F:\matlab\bin\win64\gmake -f run_sim_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make

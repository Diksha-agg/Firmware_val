@echo off
set MATLAB=F:\matlab
set MATLAB_ARCH=win64
set MATLAB_BIN="F:\matlab\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=run_sim_mex
set MEX_NAME=run_sim_mex
set MEX_EXT=.mexw64
call "F:\matlab\sys\lcc64\lcc64\mex\lcc64opts.bat"
echo # Make settings for run_sim > run_sim_mex.mki
echo COMPILER=%COMPILER%>> run_sim_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> run_sim_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> run_sim_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> run_sim_mex.mki
echo LINKER=%LINKER%>> run_sim_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> run_sim_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> run_sim_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> run_sim_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> run_sim_mex.mki
echo BORLAND=%BORLAND%>> run_sim_mex.mki
echo OMPFLAGS= >> run_sim_mex.mki
echo OMPLINKFLAGS= >> run_sim_mex.mki
echo EMC_COMPILER=lcc64>> run_sim_mex.mki
echo EMC_CONFIG=optim>> run_sim_mex.mki
"F:\matlab\bin\win64\gmake" -B -f run_sim_mex.mk

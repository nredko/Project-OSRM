set PATH=%PATH%;x:\libs18\bin;x:\tbb\bin\intel64\vc12
echo disk=m:\temp\stxxl,1000,wincall > test/stxxl.txt
set OSRM_TIMEOUT=200
set STXXLCFG=stxxl.txt
cucumber > testlog.txt
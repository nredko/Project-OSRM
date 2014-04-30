set PATH=C:\Ruby193\bin;r:\libs12\bin;r:\tbb\bin\intel64\vc12
echo disk=m:\temp\stxxl,1000,wincall > test/stxxl.txt
set OSRM_TIMEOUT=200
set STXXLCFG=stxxl.txt
cucumber > testlog.txt
set PATH=%PATH%;r:\libs12\bin
echo "disk=d:\temp\stxxl,1000,wincall" > stxxl.txt
set OSRM_TIMEOUT=200
set STXXLCFG=stxxl.txt
cucumber > testlog.txt
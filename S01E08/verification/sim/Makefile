# all: filelist com_fsdb sim verdi
#---------------------------------------------------------------
#filelist:
#    find ../de ../dv -name "*.v" -o -name "*.sv" > filelist.f
#---------------------------------------------------------------
compile:
	vcs -f ../tb/tb.f \
	-timescale=1ns/1ps \
	-full64 +v2k -sverilog -Mupdate +define+DUMP_FSDB \
	-P ${VERDI_HOME}/share/PLI/VCS/LINUX64/novas.tab ${VERDI_HOME}/share/PLI/VCS/LINUX64/pli.a \
	-debug_acc+all -debug_region+cell+encrypt \
	|tee vcs.log 
#---------------------------------------------------------------
run:
	vcs -f ../tb/tb.f \
	-timescale=1ns/1ps \
	-full64 +v2k -sverilog -Mupdate +define+DUMP_FSDB \
	-R \
	-P ${VERDI_HOME}/share/PLI/VCS/LINUX64/novas.tab ${VERDI_HOME}/share/PLI/VCS/LINUX64/pli.a \
	-debug_acc+all -debug_region+cell+encrypt \
	|tee vcs.log 
#---------------------------------------------------------------
sim:
	./simv |tee sim.log
#---------------------------------------------------------------
#dve:
#	dve -vpd vcdplus.vpd -script dve.tcl 
#---------------------------------------------------------------
verdi:
	verdi -f ../tb/tb.f -ssf sim_out.fsdb -sswr signal.rc &
#---------------------------------------------------------------
clean:
	rm -rf csrc DVEfiles novas* *.log simv* *fsdb* ucli.key *.vpd verdiLog

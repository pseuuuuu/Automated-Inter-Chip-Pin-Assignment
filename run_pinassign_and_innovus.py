#!/usr/bin/env python

import sys
import os

if len(sys.argv) != 7:
	print "Usage: run_pinassign_and_innovus.py <INPUT_DESIGN_NAME> <OUTPUT_DESIGN_NAME> <BENCHMARK_PATH>"
	print "                                              ...  <INPUT_RULE_FILE> <LOG_PATH> <OUTPUT_PATH>"
	print "Assumes input_design exists in OA DB"
	exit(1)

# Parse args
bench_name = sys.argv[1]
output_bench_name = sys.argv[2]
bench_path = sys.argv[3]
input_rule_file = sys.argv[4]
log_path = sys.argv[5]
output_path = sys.argv[6]

macro_info_file = log_path+"/pinassign_"+output_bench_name+".macros"

num_invs_runs = 0

# ===========================================================================================================
# You may customize the flow below to have several iterations over PinAssign + Innovus
	# You must keep the structure of each command below intact in order to maintain the global flow
	# You may loop over the 3-step flow as necessary
	# You are responsible for any Innovus output log file parsing for PinAssign
		# For instance: You would want to parse Innovus Run1 log info for PinAssign Run2
	# Maximum Innovus runs = 5
	# It is NOT necessary to loop over the flow - every iteration will have a significant hit on runtime

# UPDATE: Added PAVerify execution code as Step 2
	# Detailed log saved as PAVerify_<design>_<ruletype>.log
	# Summary log saved as PAVerify_<design>_<ruletype>.summary
		# Single final message from material_v3 split into 3 messages to conform with evaluation metrics:
			# (i)   Min. requirements to run Innovus: connectivity, untouched macro placement, system pins
			# (ii)  Min. macro pin pitch constraint (pmin)
			# (iii) Max. macro pin perturbation constraint (pmax) 

# 1. Run compiled PinAssign OA source code
os.system("./PinAssign "+bench_name+" "+output_bench_name+" "+input_rule_file+ \
		  " "+macro_info_file+" > "+log_path+"/pinassign_"+output_bench_name+".log")

# 2. Run compiled PAVerify OA source code
os.system("echo 'PAVerify:' >> "+log_path+"/runtime_"+output_bench_name+".log")
os.system("time ./PAVerify "+bench_name+" "+output_bench_name+" "+input_rule_file+ \
		  " "+log_path+"/PAVerify_"+output_bench_name+".summary"+ \
		  " > "+log_path+"/PAVerify_"+output_bench_name+".log 2>> " \
		  +log_path+"/runtime_"+output_bench_name+".log" )
os.system("echo '====================' >> "+log_path+"/runtime_"+output_bench_name+".log")

# Check PAVerify output
verify_out = os.popen("grep 'MIN INNOVUS REQUIREMENTS' "+log_path+"/PAVerify_"+output_bench_name+".summary").read()
verify_out = verify_out.split()[3]

if verify_out == "MET":
	print "\nPin Assignment PASSed minimum Innovus requirements. Starting Innovus run..."

	# 3. Set up Innovus script 
	os.chdir("innovus")
	os.system("./clean_invs.sh")
	os.system("./setup_invs_script.py run_innovus.template run_innovus.tcl "+bench_name+ \
			  " "+output_bench_name+" ../"+input_rule_file+ \
			  " ../"+macro_info_file+" ../"+bench_path+" ../"+output_path+" ../"+log_path)
			
	# 4. run Innovus
	num_invs_runs += 1
	if num_invs_runs<=5:
		os.system("echo 'Innovus:' >> ../"+log_path+"/runtime_"+output_bench_name+".log")
		os.system("time innovus -nowin -init run_innovus.tcl -log ../"+log_path+"/innovus_"+output_bench_name \
					+" 2>> ../"+log_path+"/runtime_"+output_bench_name+".log" )
		os.system("echo '====================' >> ../"+log_path+"/runtime_"+output_bench_name+".log")
	else:
		print "Max number of Innovus runs reached!"
	os.chdir("..")

else:
	print "\nPin Assignment FAILed minimum Innovus requirements. Skipping Innovus..."

# ===========================================================================================================

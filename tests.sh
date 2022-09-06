#!/usr/bin/env bash

set -e  # exit on error

# printf "\033c" resets the output
function log { printf "\033c"; echo -e "\033[32m[$BASH_SOURCE] $1\033[0m"; }
function echo_and_run { echo -e "\$ $@" ; read input; "$@" ; read input; }

# always run in script directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

p="-p"
if [[ $* == *--no_plots* ]]; then
    p=
fi

#echo "Please enter ref traj text file path"
#read ref_traj
#echo "Please enter traj text file path"
#read traj
#echo "Please enter results file name"
#read results

ref_traj=$1
traj=$2
out_file=$3

for m in rpe ape
do
	log "compare using rpe full (i.e. translation and rotation)"
	evo_$m tum $ref_traj $traj $p -v -as -r full --save_results ${out_file}_$m.zip --save_plot ${out_file}_${m}_plot --serialize_plot ${out_file}_${m}_splot
	
	log "load results from evo_$m and save plots in pdf and save tables"
    	evo_res ${out_file}_$m.zip --save_plot ${out_file}_$m.pdf --save_table ${out_file}_$m.csv

	log "compare using rpe translation part only"
	evo_$m tum $ref_traj $traj $p -v -as -r trans_part --save_results ${out_file}_${m}_trans.zip --save_plot ${out_file}_${m}_trans_plot
	
	log "load results from evo_$m and save plots in pdf and save tables"
    	evo_res ${out_file}_${m}_trans.zip --save_plot ${out_file}_${m}_trans.pdf --save_table ${out_file}_${m}_trans.csv

	log "compare using rpe rotation(deg) part only"
	evo_$m tum $ref_traj $traj $p -v -as -r angle_deg --save_results ${out_file}_${m}_rot.zip  --save_plot ${out_file}_${m}_rot_plot
	
	log "load results from evo_$m and save plots in pdf and save tables"
    	evo_res ${out_file}_${m}_rot.zip --save_plot ${out_file}_${m}_rot.pdf --save_table ${out_file}_${m}_rot.csv
done

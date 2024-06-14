#!/bin/bash

echo -e "\n############ ALFA Vivado Project Setup ############ \n"

if [ -d "src" ]; then
		rm -r src/*
	else
		mkdir src
fi

cp -r ../../alfa-unit/src/* src/

if [ -z "$1" ]; then
	    echo -e "-> Project Name: Undefined"
	    echo -e "-> Default Name is ALFA "
	    vivado_project_name="ALFA"
else 
	vivado_project_name=$1
	echo -e "-> Project Name: $vivado_project_name"
fi

if [ -z "$2" ]; then
	    echo -e "-> Board: Undefined"
	    echo -e "-> Default Board is ZCU104"
	    vivado_project_board="zcu104"
else 
	vivado_project_board=$2
	echo -e "-> Board: $vivado_project_board"
fi

if [ -d "tcl/$vivado_project_board" ]; then
		echo -e "-> Board: supported"
else
		echo -e "-> Board: not supported"
		echo -e "-> Run this script again with a supported board as second parameter\n"
		echo -e "-> Supported boards:"
		echo -e "-> zcu104"
		exit 1
fi

if [ ! -d "vivado_projects" ]; then
		mkdir vivado_projects
fi    
if [ -d "vivado_projects/$vivado_project_name" ]; then  
        echo -e "-> Rebuilding $vivado_project_name project"
        rm -r vivado_projects/${vivado_project_name}
else
		echo -e "-> Creating $vivado_project_name project"
fi

if [ -d "/opt/Xilinx/Vivado/" ]; then	
    	/opt/Xilinx/Vivado/*/bin/vivado -source tcl/$vivado_project_board/build.tcl -tclargs $vivado_project_name -log "vivado_projects/vivado.log" -journal "vivado_projects/vivado.jou"
    	exit 0
    	
else 
	if [ -d $3 ]; then
		vivado_path_dir=$3
		vivado_path_dir+="/*/bin/vivado -source tcl/$vivado_project_board/build.tcl -tclargs $vivado_project_name -log "vivado_projects/$vivado_project_name/vivado.log" -journal "vivado_projects/$vivado_project_name/vivado.jou""
		${vivado_path_dir}
		exit 0
	
	else
		echo -e "-> Vivado installation folder not found in /opt/Xilinx/Vivado/ neither in $3\n"
		echo -e "-> Run this script again with the full path of Vivado folder as third parameter\n"
		exit 1
	fi
fi

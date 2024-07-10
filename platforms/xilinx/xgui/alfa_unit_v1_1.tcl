
# Loading additional proc with user specified bodies to compute parameter values.
source [file join [file dirname [file dirname [info script]]] gui/ALFAUnit_v1_0.gtcl]

# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  #Adding Group
  set Memory [ipgui::add_group $IPINST -name "Memory" -parent ${Page_0}]
  ipgui::add_param $IPINST -name "INTERFACE_TYPE" -parent ${Memory} -widget comboBox
  ipgui::add_param $IPINST -name "BRAM_DDR" -parent ${Memory} -widget comboBox
  ipgui::add_param $IPINST -name "REPRESENTATION_TYPE" -parent ${Memory} -widget comboBox

  #Adding Group
  set Extension [ipgui::add_group $IPINST -name "Extension" -parent ${Page_0}]
  ipgui::add_param $IPINST -name "EXMU_SUPPORT" -parent ${Extension} -widget comboBox
  ipgui::add_param $IPINST -name "NUMBER_OF_DEBUG_POINTS" -parent ${Extension}
  ipgui::add_param $IPINST -name "NUMBER_OF_USER_DEFINES" -parent ${Extension}
  #Adding Group
  set Extension_Memory [ipgui::add_group $IPINST -name "Extension Memory" -parent ${Extension} -layout horizontal]
  ipgui::add_param $IPINST -name "EXTENSION_MEMORY" -parent ${Extension_Memory}




}

proc update_PARAM_VALUE.EXMU_SUPPORT { PARAM_VALUE.EXMU_SUPPORT PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update EXMU_SUPPORT when any of the dependent parameters in the arguments change
	
	set EXMU_SUPPORT ${PARAM_VALUE.EXMU_SUPPORT}
	set SIU_SUPPORT ${PARAM_VALUE.SIU_SUPPORT}
	set values(SIU_SUPPORT) [get_property value $SIU_SUPPORT]
	if { [gen_USERPARAMETER_EXMU_SUPPORT_ENABLEMENT $values(SIU_SUPPORT)] } {
		set_property enabled true $EXMU_SUPPORT
	} else {
		set_property enabled false $EXMU_SUPPORT
	}
}

proc validate_PARAM_VALUE.EXMU_SUPPORT { PARAM_VALUE.EXMU_SUPPORT } {
	# Procedure called to validate EXMU_SUPPORT
	return true
}

proc update_PARAM_VALUE.OFFSET { PARAM_VALUE.OFFSET PARAM_VALUE.BRAM_DDR } {
	# Procedure called to update OFFSET when any of the dependent parameters in the arguments change
	
	set OFFSET ${PARAM_VALUE.OFFSET}
	set BRAM_DDR ${PARAM_VALUE.BRAM_DDR}
	set values(BRAM_DDR) [get_property value $BRAM_DDR]
	if { [gen_USERPARAMETER_OFFSET_ENABLEMENT $values(BRAM_DDR)] } {
		set_property enabled true $OFFSET
	} else {
		set_property enabled false $OFFSET
	}
}

proc validate_PARAM_VALUE.OFFSET { PARAM_VALUE.OFFSET } {
	# Procedure called to validate OFFSET
	return true
}

proc update_PARAM_VALUE.POINTCLOUD_ID { PARAM_VALUE.POINTCLOUD_ID PARAM_VALUE.BRAM_DDR } {
	# Procedure called to update POINTCLOUD_ID when any of the dependent parameters in the arguments change
	
	set POINTCLOUD_ID ${PARAM_VALUE.POINTCLOUD_ID}
	set BRAM_DDR ${PARAM_VALUE.BRAM_DDR}
	set values(BRAM_DDR) [get_property value $BRAM_DDR]
	if { [gen_USERPARAMETER_POINTCLOUD_ID_ENABLEMENT $values(BRAM_DDR)] } {
		set_property enabled true $POINTCLOUD_ID
	} else {
		set_property enabled false $POINTCLOUD_ID
	}
}

proc validate_PARAM_VALUE.POINTCLOUD_ID { PARAM_VALUE.POINTCLOUD_ID } {
	# Procedure called to validate POINTCLOUD_ID
	return true
}

proc update_PARAM_VALUE.REPRESENTATION_PARAM_1 { PARAM_VALUE.REPRESENTATION_PARAM_1 PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update REPRESENTATION_PARAM_1 when any of the dependent parameters in the arguments change
	
	set REPRESENTATION_PARAM_1 ${PARAM_VALUE.REPRESENTATION_PARAM_1}
	set SIU_SUPPORT ${PARAM_VALUE.SIU_SUPPORT}
	set values(SIU_SUPPORT) [get_property value $SIU_SUPPORT]
	if { [gen_USERPARAMETER_REPRESENTATION_PARAM_1_ENABLEMENT $values(SIU_SUPPORT)] } {
		set_property enabled true $REPRESENTATION_PARAM_1
	} else {
		set_property enabled false $REPRESENTATION_PARAM_1
	}
}

proc validate_PARAM_VALUE.REPRESENTATION_PARAM_1 { PARAM_VALUE.REPRESENTATION_PARAM_1 } {
	# Procedure called to validate REPRESENTATION_PARAM_1
	return true
}

proc update_PARAM_VALUE.REPRESENTATION_PARAM_2 { PARAM_VALUE.REPRESENTATION_PARAM_2 PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update REPRESENTATION_PARAM_2 when any of the dependent parameters in the arguments change
	
	set REPRESENTATION_PARAM_2 ${PARAM_VALUE.REPRESENTATION_PARAM_2}
	set SIU_SUPPORT ${PARAM_VALUE.SIU_SUPPORT}
	set values(SIU_SUPPORT) [get_property value $SIU_SUPPORT]
	if { [gen_USERPARAMETER_REPRESENTATION_PARAM_2_ENABLEMENT $values(SIU_SUPPORT)] } {
		set_property enabled true $REPRESENTATION_PARAM_2
	} else {
		set_property enabled false $REPRESENTATION_PARAM_2
	}
}

proc validate_PARAM_VALUE.REPRESENTATION_PARAM_2 { PARAM_VALUE.REPRESENTATION_PARAM_2 } {
	# Procedure called to validate REPRESENTATION_PARAM_2
	return true
}

proc update_PARAM_VALUE.REPRESENTATION_PARAM_3 { PARAM_VALUE.REPRESENTATION_PARAM_3 PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update REPRESENTATION_PARAM_3 when any of the dependent parameters in the arguments change
	
	set REPRESENTATION_PARAM_3 ${PARAM_VALUE.REPRESENTATION_PARAM_3}
	set SIU_SUPPORT ${PARAM_VALUE.SIU_SUPPORT}
	set values(SIU_SUPPORT) [get_property value $SIU_SUPPORT]
	if { [gen_USERPARAMETER_REPRESENTATION_PARAM_3_ENABLEMENT $values(SIU_SUPPORT)] } {
		set_property enabled true $REPRESENTATION_PARAM_3
	} else {
		set_property enabled false $REPRESENTATION_PARAM_3
	}
}

proc validate_PARAM_VALUE.REPRESENTATION_PARAM_3 { PARAM_VALUE.REPRESENTATION_PARAM_3 } {
	# Procedure called to validate REPRESENTATION_PARAM_3
	return true
}

proc update_PARAM_VALUE.REPRESENTATION_PARAM_4 { PARAM_VALUE.REPRESENTATION_PARAM_4 PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update REPRESENTATION_PARAM_4 when any of the dependent parameters in the arguments change
	
	set REPRESENTATION_PARAM_4 ${PARAM_VALUE.REPRESENTATION_PARAM_4}
	set SIU_SUPPORT ${PARAM_VALUE.SIU_SUPPORT}
	set values(SIU_SUPPORT) [get_property value $SIU_SUPPORT]
	if { [gen_USERPARAMETER_REPRESENTATION_PARAM_4_ENABLEMENT $values(SIU_SUPPORT)] } {
		set_property enabled true $REPRESENTATION_PARAM_4
	} else {
		set_property enabled false $REPRESENTATION_PARAM_4
	}
}

proc validate_PARAM_VALUE.REPRESENTATION_PARAM_4 { PARAM_VALUE.REPRESENTATION_PARAM_4 } {
	# Procedure called to validate REPRESENTATION_PARAM_4
	return true
}

proc update_PARAM_VALUE.BRAM_DDR { PARAM_VALUE.BRAM_DDR } {
	# Procedure called to update BRAM_DDR when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BRAM_DDR { PARAM_VALUE.BRAM_DDR } {
	# Procedure called to validate BRAM_DDR
	return true
}

proc update_PARAM_VALUE.EXTENSION_MEMORY { PARAM_VALUE.EXTENSION_MEMORY } {
	# Procedure called to update EXTENSION_MEMORY when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.EXTENSION_MEMORY { PARAM_VALUE.EXTENSION_MEMORY } {
	# Procedure called to validate EXTENSION_MEMORY
	return true
}

proc update_PARAM_VALUE.INTERFACE_TYPE { PARAM_VALUE.INTERFACE_TYPE } {
	# Procedure called to update INTERFACE_TYPE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.INTERFACE_TYPE { PARAM_VALUE.INTERFACE_TYPE } {
	# Procedure called to validate INTERFACE_TYPE
	return true
}

proc update_PARAM_VALUE.NUMBER_OF_DEBUG_POINTS { PARAM_VALUE.NUMBER_OF_DEBUG_POINTS } {
	# Procedure called to update NUMBER_OF_DEBUG_POINTS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUMBER_OF_DEBUG_POINTS { PARAM_VALUE.NUMBER_OF_DEBUG_POINTS } {
	# Procedure called to validate NUMBER_OF_DEBUG_POINTS
	return true
}

proc update_PARAM_VALUE.NUMBER_OF_USER_DEFINES { PARAM_VALUE.NUMBER_OF_USER_DEFINES } {
	# Procedure called to update NUMBER_OF_USER_DEFINES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUMBER_OF_USER_DEFINES { PARAM_VALUE.NUMBER_OF_USER_DEFINES } {
	# Procedure called to validate NUMBER_OF_USER_DEFINES
	return true
}

proc update_PARAM_VALUE.REPRESENTATION_TYPE { PARAM_VALUE.REPRESENTATION_TYPE } {
	# Procedure called to update REPRESENTATION_TYPE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.REPRESENTATION_TYPE { PARAM_VALUE.REPRESENTATION_TYPE } {
	# Procedure called to validate REPRESENTATION_TYPE
	return true
}

proc update_PARAM_VALUE.SIU_SUPPORT { PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to update SIU_SUPPORT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SIU_SUPPORT { PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to validate SIU_SUPPORT
	return true
}


proc update_MODELPARAM_VALUE.INTERFACE_TYPE { MODELPARAM_VALUE.INTERFACE_TYPE PARAM_VALUE.INTERFACE_TYPE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.INTERFACE_TYPE}] ${MODELPARAM_VALUE.INTERFACE_TYPE}
}

proc update_MODELPARAM_VALUE.SIU_SUPPORT { MODELPARAM_VALUE.SIU_SUPPORT PARAM_VALUE.SIU_SUPPORT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SIU_SUPPORT}] ${MODELPARAM_VALUE.SIU_SUPPORT}
}

proc update_MODELPARAM_VALUE.EXMU_SUPPORT { MODELPARAM_VALUE.EXMU_SUPPORT PARAM_VALUE.EXMU_SUPPORT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.EXMU_SUPPORT}] ${MODELPARAM_VALUE.EXMU_SUPPORT}
}

proc update_MODELPARAM_VALUE.OFFSET { MODELPARAM_VALUE.OFFSET PARAM_VALUE.OFFSET } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OFFSET}] ${MODELPARAM_VALUE.OFFSET}
}

proc update_MODELPARAM_VALUE.BRAM_DDR { MODELPARAM_VALUE.BRAM_DDR PARAM_VALUE.BRAM_DDR } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BRAM_DDR}] ${MODELPARAM_VALUE.BRAM_DDR}
}

proc update_MODELPARAM_VALUE.REPRESENTATION_TYPE { MODELPARAM_VALUE.REPRESENTATION_TYPE PARAM_VALUE.REPRESENTATION_TYPE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.REPRESENTATION_TYPE}] ${MODELPARAM_VALUE.REPRESENTATION_TYPE}
}

proc update_MODELPARAM_VALUE.REPRESENTATION_PARAM_1 { MODELPARAM_VALUE.REPRESENTATION_PARAM_1 PARAM_VALUE.REPRESENTATION_PARAM_1 } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.REPRESENTATION_PARAM_1}] ${MODELPARAM_VALUE.REPRESENTATION_PARAM_1}
}

proc update_MODELPARAM_VALUE.REPRESENTATION_PARAM_2 { MODELPARAM_VALUE.REPRESENTATION_PARAM_2 PARAM_VALUE.REPRESENTATION_PARAM_2 } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.REPRESENTATION_PARAM_2}] ${MODELPARAM_VALUE.REPRESENTATION_PARAM_2}
}

proc update_MODELPARAM_VALUE.REPRESENTATION_PARAM_3 { MODELPARAM_VALUE.REPRESENTATION_PARAM_3 PARAM_VALUE.REPRESENTATION_PARAM_3 } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.REPRESENTATION_PARAM_3}] ${MODELPARAM_VALUE.REPRESENTATION_PARAM_3}
}

proc update_MODELPARAM_VALUE.REPRESENTATION_PARAM_4 { MODELPARAM_VALUE.REPRESENTATION_PARAM_4 PARAM_VALUE.REPRESENTATION_PARAM_4 } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.REPRESENTATION_PARAM_4}] ${MODELPARAM_VALUE.REPRESENTATION_PARAM_4}
}

proc update_MODELPARAM_VALUE.POINTCLOUD_ID { MODELPARAM_VALUE.POINTCLOUD_ID PARAM_VALUE.POINTCLOUD_ID } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.POINTCLOUD_ID}] ${MODELPARAM_VALUE.POINTCLOUD_ID}
}

proc update_MODELPARAM_VALUE.NUMBER_OF_DEBUG_POINTS { MODELPARAM_VALUE.NUMBER_OF_DEBUG_POINTS PARAM_VALUE.NUMBER_OF_DEBUG_POINTS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUMBER_OF_DEBUG_POINTS}] ${MODELPARAM_VALUE.NUMBER_OF_DEBUG_POINTS}
}

proc update_MODELPARAM_VALUE.NUMBER_OF_USER_DEFINES { MODELPARAM_VALUE.NUMBER_OF_USER_DEFINES PARAM_VALUE.NUMBER_OF_USER_DEFINES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUMBER_OF_USER_DEFINES}] ${MODELPARAM_VALUE.NUMBER_OF_USER_DEFINES}
}


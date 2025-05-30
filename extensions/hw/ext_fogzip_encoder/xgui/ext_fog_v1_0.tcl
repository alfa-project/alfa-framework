# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "NUMBER_NODES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MIN_X" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MIN_Y" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MIN_Z" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MAX_X" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MAX_Y" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BB_MAX_Z" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_DEPTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "COMPRESSION" -parent ${Page_0} -widget comboBox
  ipgui::add_param $IPINST -name "PARALLEL_PROCESSING" -parent ${Page_0} -widget comboBox


}

proc update_PARAM_VALUE.BB_MAX_X { PARAM_VALUE.BB_MAX_X } {
	# Procedure called to update BB_MAX_X when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MAX_X { PARAM_VALUE.BB_MAX_X } {
	# Procedure called to validate BB_MAX_X
	return true
}

proc update_PARAM_VALUE.BB_MAX_Y { PARAM_VALUE.BB_MAX_Y } {
	# Procedure called to update BB_MAX_Y when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MAX_Y { PARAM_VALUE.BB_MAX_Y } {
	# Procedure called to validate BB_MAX_Y
	return true
}

proc update_PARAM_VALUE.BB_MAX_Z { PARAM_VALUE.BB_MAX_Z } {
	# Procedure called to update BB_MAX_Z when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MAX_Z { PARAM_VALUE.BB_MAX_Z } {
	# Procedure called to validate BB_MAX_Z
	return true
}

proc update_PARAM_VALUE.BB_MIN_X { PARAM_VALUE.BB_MIN_X } {
	# Procedure called to update BB_MIN_X when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MIN_X { PARAM_VALUE.BB_MIN_X } {
	# Procedure called to validate BB_MIN_X
	return true
}

proc update_PARAM_VALUE.BB_MIN_Y { PARAM_VALUE.BB_MIN_Y } {
	# Procedure called to update BB_MIN_Y when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MIN_Y { PARAM_VALUE.BB_MIN_Y } {
	# Procedure called to validate BB_MIN_Y
	return true
}

proc update_PARAM_VALUE.BB_MIN_Z { PARAM_VALUE.BB_MIN_Z } {
	# Procedure called to update BB_MIN_Z when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BB_MIN_Z { PARAM_VALUE.BB_MIN_Z } {
	# Procedure called to validate BB_MIN_Z
	return true
}

proc update_PARAM_VALUE.COMPRESSION { PARAM_VALUE.COMPRESSION } {
	# Procedure called to update COMPRESSION when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.COMPRESSION { PARAM_VALUE.COMPRESSION } {
	# Procedure called to validate COMPRESSION
	return true
}

proc update_PARAM_VALUE.MAX_DEPTH { PARAM_VALUE.MAX_DEPTH } {
	# Procedure called to update MAX_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_DEPTH { PARAM_VALUE.MAX_DEPTH } {
	# Procedure called to validate MAX_DEPTH
	return true
}

proc update_PARAM_VALUE.NUMBER_NODES { PARAM_VALUE.NUMBER_NODES } {
	# Procedure called to update NUMBER_NODES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUMBER_NODES { PARAM_VALUE.NUMBER_NODES } {
	# Procedure called to validate NUMBER_NODES
	return true
}

proc update_PARAM_VALUE.PARALLEL_PROCESSING { PARAM_VALUE.PARALLEL_PROCESSING } {
	# Procedure called to update PARALLEL_PROCESSING when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PARALLEL_PROCESSING { PARAM_VALUE.PARALLEL_PROCESSING } {
	# Procedure called to validate PARALLEL_PROCESSING
	return true
}


proc update_MODELPARAM_VALUE.NUMBER_NODES { MODELPARAM_VALUE.NUMBER_NODES PARAM_VALUE.NUMBER_NODES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUMBER_NODES}] ${MODELPARAM_VALUE.NUMBER_NODES}
}

proc update_MODELPARAM_VALUE.BB_MIN_X { MODELPARAM_VALUE.BB_MIN_X PARAM_VALUE.BB_MIN_X } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MIN_X}] ${MODELPARAM_VALUE.BB_MIN_X}
}

proc update_MODELPARAM_VALUE.BB_MIN_Y { MODELPARAM_VALUE.BB_MIN_Y PARAM_VALUE.BB_MIN_Y } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MIN_Y}] ${MODELPARAM_VALUE.BB_MIN_Y}
}

proc update_MODELPARAM_VALUE.BB_MIN_Z { MODELPARAM_VALUE.BB_MIN_Z PARAM_VALUE.BB_MIN_Z } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MIN_Z}] ${MODELPARAM_VALUE.BB_MIN_Z}
}

proc update_MODELPARAM_VALUE.BB_MAX_X { MODELPARAM_VALUE.BB_MAX_X PARAM_VALUE.BB_MAX_X } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MAX_X}] ${MODELPARAM_VALUE.BB_MAX_X}
}

proc update_MODELPARAM_VALUE.BB_MAX_Y { MODELPARAM_VALUE.BB_MAX_Y PARAM_VALUE.BB_MAX_Y } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MAX_Y}] ${MODELPARAM_VALUE.BB_MAX_Y}
}

proc update_MODELPARAM_VALUE.BB_MAX_Z { MODELPARAM_VALUE.BB_MAX_Z PARAM_VALUE.BB_MAX_Z } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BB_MAX_Z}] ${MODELPARAM_VALUE.BB_MAX_Z}
}

proc update_MODELPARAM_VALUE.MAX_DEPTH { MODELPARAM_VALUE.MAX_DEPTH PARAM_VALUE.MAX_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_DEPTH}] ${MODELPARAM_VALUE.MAX_DEPTH}
}

proc update_MODELPARAM_VALUE.PARALLEL_PROCESSING { MODELPARAM_VALUE.PARALLEL_PROCESSING PARAM_VALUE.PARALLEL_PROCESSING } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PARALLEL_PROCESSING}] ${MODELPARAM_VALUE.PARALLEL_PROCESSING}
}

proc update_MODELPARAM_VALUE.COMPRESSION { MODELPARAM_VALUE.COMPRESSION PARAM_VALUE.COMPRESSION } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.COMPRESSION}] ${MODELPARAM_VALUE.COMPRESSION}
}


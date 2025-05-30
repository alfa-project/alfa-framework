# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "DEPTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "WIDTH_INPUT" -parent ${Page_0}
  ipgui::add_param $IPINST -name "WIDTH_INPUT_LENGTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "WIDTH_OUTPUT" -parent ${Page_0}


}

proc update_PARAM_VALUE.DEPTH { PARAM_VALUE.DEPTH } {
	# Procedure called to update DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DEPTH { PARAM_VALUE.DEPTH } {
	# Procedure called to validate DEPTH
	return true
}

proc update_PARAM_VALUE.WIDTH_INPUT { PARAM_VALUE.WIDTH_INPUT } {
	# Procedure called to update WIDTH_INPUT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.WIDTH_INPUT { PARAM_VALUE.WIDTH_INPUT } {
	# Procedure called to validate WIDTH_INPUT
	return true
}

proc update_PARAM_VALUE.WIDTH_INPUT_LENGTH { PARAM_VALUE.WIDTH_INPUT_LENGTH } {
	# Procedure called to update WIDTH_INPUT_LENGTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.WIDTH_INPUT_LENGTH { PARAM_VALUE.WIDTH_INPUT_LENGTH } {
	# Procedure called to validate WIDTH_INPUT_LENGTH
	return true
}

proc update_PARAM_VALUE.WIDTH_OUTPUT { PARAM_VALUE.WIDTH_OUTPUT } {
	# Procedure called to update WIDTH_OUTPUT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.WIDTH_OUTPUT { PARAM_VALUE.WIDTH_OUTPUT } {
	# Procedure called to validate WIDTH_OUTPUT
	return true
}


proc update_MODELPARAM_VALUE.DEPTH { MODELPARAM_VALUE.DEPTH PARAM_VALUE.DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DEPTH}] ${MODELPARAM_VALUE.DEPTH}
}

proc update_MODELPARAM_VALUE.WIDTH_INPUT { MODELPARAM_VALUE.WIDTH_INPUT PARAM_VALUE.WIDTH_INPUT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.WIDTH_INPUT}] ${MODELPARAM_VALUE.WIDTH_INPUT}
}

proc update_MODELPARAM_VALUE.WIDTH_INPUT_LENGTH { MODELPARAM_VALUE.WIDTH_INPUT_LENGTH PARAM_VALUE.WIDTH_INPUT_LENGTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.WIDTH_INPUT_LENGTH}] ${MODELPARAM_VALUE.WIDTH_INPUT_LENGTH}
}

proc update_MODELPARAM_VALUE.WIDTH_OUTPUT { MODELPARAM_VALUE.WIDTH_OUTPUT PARAM_VALUE.WIDTH_OUTPUT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.WIDTH_OUTPUT}] ${MODELPARAM_VALUE.WIDTH_OUTPUT}
}


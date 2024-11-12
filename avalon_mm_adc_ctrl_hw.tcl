# TCL File Generated by Component Editor 18.0
# Tue Nov 12 15:47:36 EST 2024
# DO NOT MODIFY


# 
# avalon_mm_adc_ctrl "avalon_mm_adc_ctrl" v1.0
#  2024.11.12.15:47:36
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module avalon_mm_adc_ctrl
# 
set_module_property DESCRIPTION ""
set_module_property NAME avalon_mm_adc_ctrl
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP CustomIP
set_module_property AUTHOR ""
set_module_property DISPLAY_NAME avalon_mm_adc_ctrl
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL adc_ctrl
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file adc_ctrl.sv SYSTEM_VERILOG PATH adc_ctrl.sv TOP_LEVEL_FILE

add_fileset SIM_VERILOG SIM_VERILOG "" ""
set_fileset_property SIM_VERILOG TOP_LEVEL adc_ctrl
set_fileset_property SIM_VERILOG ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VERILOG ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file adc_ctrl.sv SYSTEM_VERILOG PATH adc_ctrl.sv


# 
# parameters
# 
add_parameter SRAM_SIZE INTEGER 16384
set_parameter_property SRAM_SIZE DEFAULT_VALUE 16384
set_parameter_property SRAM_SIZE DISPLAY_NAME SRAM_SIZE
set_parameter_property SRAM_SIZE TYPE INTEGER
set_parameter_property SRAM_SIZE UNITS None
set_parameter_property SRAM_SIZE ALLOWED_RANGES -2147483648:2147483647
set_parameter_property SRAM_SIZE HDL_PARAMETER true


# 
# display items
# 


# 
# connection point avalon_slave_0
# 
add_interface avalon_slave_0 avalon end
set_interface_property avalon_slave_0 addressUnits WORDS
set_interface_property avalon_slave_0 associatedClock clock_sink
set_interface_property avalon_slave_0 associatedReset reset_sink
set_interface_property avalon_slave_0 bitsPerSymbol 8
set_interface_property avalon_slave_0 burstOnBurstBoundariesOnly false
set_interface_property avalon_slave_0 burstcountUnits WORDS
set_interface_property avalon_slave_0 explicitAddressSpan 0
set_interface_property avalon_slave_0 holdTime 0
set_interface_property avalon_slave_0 linewrapBursts false
set_interface_property avalon_slave_0 maximumPendingReadTransactions 0
set_interface_property avalon_slave_0 maximumPendingWriteTransactions 0
set_interface_property avalon_slave_0 readLatency 0
set_interface_property avalon_slave_0 readWaitStates 0
set_interface_property avalon_slave_0 readWaitTime 0
set_interface_property avalon_slave_0 setupTime 0
set_interface_property avalon_slave_0 timingUnits Cycles
set_interface_property avalon_slave_0 writeWaitTime 0
set_interface_property avalon_slave_0 ENABLED true
set_interface_property avalon_slave_0 EXPORT_OF ""
set_interface_property avalon_slave_0 PORT_NAME_MAP ""
set_interface_property avalon_slave_0 CMSIS_SVD_VARIABLES ""
set_interface_property avalon_slave_0 SVD_ADDRESS_GROUP ""

add_interface_port avalon_slave_0 address_i address Input 8
add_interface_port avalon_slave_0 byteenable_i byteenable Input 4
add_interface_port avalon_slave_0 read_i read Input 1
add_interface_port avalon_slave_0 write_i write Input 1
add_interface_port avalon_slave_0 writedata_i writedata Input 32
add_interface_port avalon_slave_0 readdata_o readdata Output 32
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isFlash 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment avalon_slave_0 embeddedsw.configuration.isPrintableDevice 0


# 
# connection point conduit_ad9226
# 
add_interface conduit_ad9226 conduit end
set_interface_property conduit_ad9226 associatedClock ""
set_interface_property conduit_ad9226 associatedReset ""
set_interface_property conduit_ad9226 ENABLED true
set_interface_property conduit_ad9226 EXPORT_OF ""
set_interface_property conduit_ad9226 PORT_NAME_MAP ""
set_interface_property conduit_ad9226 CMSIS_SVD_VARIABLES ""
set_interface_property conduit_ad9226 SVD_ADDRESS_GROUP ""

add_interface_port conduit_ad9226 ad9226_clk_o clk_o Output 1
add_interface_port conduit_ad9226 ad9226_d_i d_i Input 12
add_interface_port conduit_ad9226 ad9226_otr_i otr_i Input 1


# 
# connection point clock_sink
# 
add_interface clock_sink clock end
set_interface_property clock_sink clockRate 0
set_interface_property clock_sink ENABLED true
set_interface_property clock_sink EXPORT_OF ""
set_interface_property clock_sink PORT_NAME_MAP ""
set_interface_property clock_sink CMSIS_SVD_VARIABLES ""
set_interface_property clock_sink SVD_ADDRESS_GROUP ""

add_interface_port clock_sink clk_i clk Input 1


# 
# connection point reset_sink
# 
add_interface reset_sink reset end
set_interface_property reset_sink associatedClock clock_sink
set_interface_property reset_sink synchronousEdges DEASSERT
set_interface_property reset_sink ENABLED true
set_interface_property reset_sink EXPORT_OF ""
set_interface_property reset_sink PORT_NAME_MAP ""
set_interface_property reset_sink CMSIS_SVD_VARIABLES ""
set_interface_property reset_sink SVD_ADDRESS_GROUP ""

add_interface_port reset_sink rst_ni reset_n Input 1


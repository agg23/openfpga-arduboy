onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /rom_loader_tb/clk
add wave -noupdate /rom_loader_tb/clk2
add wave -noupdate -radix hexadecimal /rom_loader_tb/bridge_addr
add wave -noupdate -radix hexadecimal /rom_loader_tb/bridge_wr_data
add wave -noupdate /rom_loader_tb/bridge_wr
add wave -noupdate -radix hexadecimal /rom_loader_tb/address_b
add wave -noupdate -radix hexadecimal /rom_loader_tb/output_b
add wave -noupdate /rom_loader_tb/write_en
add wave -noupdate -radix hexadecimal /rom_loader_tb/write_addr
add wave -noupdate -radix hexadecimal /rom_loader_tb/write_data
add wave -noupdate -radix hexadecimal /rom_loader_tb/LOADER/line_byte_count
add wave -noupdate -radix hexadecimal /rom_loader_tb/LOADER/digit
add wave -noupdate /rom_loader_tb/LOADER/apf_write_en
add wave -noupdate -radix hexadecimal /rom_loader_tb/LOADER/apf_write_addr
add wave -noupdate -radix hexadecimal -childformat {{{/rom_loader_tb/LOADER/apf_write_data[7]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[6]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[5]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[4]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[3]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[2]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[1]} -radix hexadecimal} {{/rom_loader_tb/LOADER/apf_write_data[0]} -radix hexadecimal}} -subitemconfig {{/rom_loader_tb/LOADER/apf_write_data[7]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[6]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[5]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[4]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[3]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[2]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[1]} {-height 15 -radix hexadecimal} {/rom_loader_tb/LOADER/apf_write_data[0]} {-height 15 -radix hexadecimal}} /rom_loader_tb/LOADER/apf_write_data
add wave -noupdate -radix decimal /rom_loader_tb/LOADER/hex_nibble_index
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {116717464 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 246
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {7620448 ps}

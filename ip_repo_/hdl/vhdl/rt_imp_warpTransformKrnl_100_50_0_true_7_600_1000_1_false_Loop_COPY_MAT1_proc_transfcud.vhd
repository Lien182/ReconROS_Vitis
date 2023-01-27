-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
-- Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_transfcud is 
    generic(
             DataWidth     : integer := 32; 
             AddressWidth     : integer := 4; 
             AddressRange    : integer := 9
    ); 
    port (
          address0      : in std_logic_vector(AddressWidth-1 downto 0); 
          ce0       : in std_logic; 
          q0         : out std_logic_vector(DataWidth-1 downto 0);
          reset     : in std_logic;
          clk       : in std_logic
    ); 
end entity; 


architecture rtl of rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_transfcud is 

signal address0_tmp : std_logic_vector(AddressWidth-1 downto 0); 
type mem_array is array (0 to AddressRange-1) of std_logic_vector (DataWidth-1 downto 0); 
signal mem : mem_array := (
    0 => "11000000000110011110110010001111", 
    1 => "11000000011100110000100110111001", 
    2 => "01000100100111101011001111011001", 
    3 => "00000000000000000000000000000000", 
    4 => "11000001000001011010101110111111", 
    5 => "01000101000101110110110010001111", 
    6 => "00000000000000000000000000000000", 
    7 => "10111011111110001101111011110010", 
    8 => "00111111100000000000000000000000" );


begin 


memory_access_guard_0: process (address0) 
begin
      address0_tmp <= address0;
--synthesis translate_off
      if (CONV_INTEGER(address0) > AddressRange-1) then
           address0_tmp <= (others => '0');
      else 
           address0_tmp <= address0;
      end if;
--synthesis translate_on
end process;

p_rom_access: process (clk)  
begin 
    if (clk'event and clk = '1') then
        if (ce0 = '1') then 
            q0 <= mem(CONV_INTEGER(address0_tmp)); 
        end if;
    end if;
end process;

end rtl;


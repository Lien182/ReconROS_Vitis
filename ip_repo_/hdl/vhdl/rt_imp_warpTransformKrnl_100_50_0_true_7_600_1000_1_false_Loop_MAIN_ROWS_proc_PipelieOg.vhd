-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
-- Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelieOg is 
    generic(
             DataWidth     : integer := 24; 
             AddressWidth     : integer := 5; 
             AddressRange    : integer := 32
    ); 
    port (
          address0      : in std_logic_vector(AddressWidth-1 downto 0); 
          ce0       : in std_logic; 
          q0         : out std_logic_vector(DataWidth-1 downto 0);
          address1      : in std_logic_vector(AddressWidth-1 downto 0); 
          ce1       : in std_logic; 
          q1         : out std_logic_vector(DataWidth-1 downto 0);
          address2      : in std_logic_vector(AddressWidth-1 downto 0); 
          ce2       : in std_logic; 
          q2         : out std_logic_vector(DataWidth-1 downto 0);
          address3      : in std_logic_vector(AddressWidth-1 downto 0); 
          ce3       : in std_logic; 
          q3         : out std_logic_vector(DataWidth-1 downto 0);
          reset     : in std_logic;
          clk       : in std_logic
    ); 
end entity; 


architecture rtl of rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelieOg is 

signal address0_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address1_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address2_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address3_tmp : std_logic_vector(AddressWidth-1 downto 0); 
type mem_array is array (0 to AddressRange-1) of std_logic_vector (DataWidth-1 downto 0); 
signal mem0 : mem_array := (
    0 => "001000000000000000000000", 1 => "000100000000000000000000", 
    2 => "000010000000000000000000", 3 => "000001000000000000000000", 
    4 => "000000100000000000000000", 5 => "000000010000000000000000", 
    6 => "000000001000000000000000", 7 => "000000000100000000000000", 
    8 => "000000000010000000000000", 9 => "000000000001000000000000", 
    10 => "000000000000100000000000", 11 => "000000000000010000000000", 
    12 => "000000000000001000000000", 13 => "000000000000000100000000", 
    14 => "000000000000000010000000", 15 => "000000000000000001000000", 
    16 => "000000000000000000100000", 17 => "000000000000000000010000", 
    18 => "000000000000000000001000", 19 => "000000000000000000000100", 
    20 => "000000000000000000000010", 21 => "000000000000000000000001", 
    22 to 29=> "000000000000000000000000", 30 => "100000000000000000000000", 
    31 => "010000000000000000000000" );
signal mem1 : mem_array := (
    0 => "001000000000000000000000", 1 => "000100000000000000000000", 
    2 => "000010000000000000000000", 3 => "000001000000000000000000", 
    4 => "000000100000000000000000", 5 => "000000010000000000000000", 
    6 => "000000001000000000000000", 7 => "000000000100000000000000", 
    8 => "000000000010000000000000", 9 => "000000000001000000000000", 
    10 => "000000000000100000000000", 11 => "000000000000010000000000", 
    12 => "000000000000001000000000", 13 => "000000000000000100000000", 
    14 => "000000000000000010000000", 15 => "000000000000000001000000", 
    16 => "000000000000000000100000", 17 => "000000000000000000010000", 
    18 => "000000000000000000001000", 19 => "000000000000000000000100", 
    20 => "000000000000000000000010", 21 => "000000000000000000000001", 
    22 to 29=> "000000000000000000000000", 30 => "100000000000000000000000", 
    31 => "010000000000000000000000" );


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

memory_access_guard_1: process (address1) 
begin
      address1_tmp <= address1;
--synthesis translate_off
      if (CONV_INTEGER(address1) > AddressRange-1) then
           address1_tmp <= (others => '0');
      else 
           address1_tmp <= address1;
      end if;
--synthesis translate_on
end process;

memory_access_guard_2: process (address2) 
begin
      address2_tmp <= address2;
--synthesis translate_off
      if (CONV_INTEGER(address2) > AddressRange-1) then
           address2_tmp <= (others => '0');
      else 
           address2_tmp <= address2;
      end if;
--synthesis translate_on
end process;

memory_access_guard_3: process (address3) 
begin
      address3_tmp <= address3;
--synthesis translate_off
      if (CONV_INTEGER(address3) > AddressRange-1) then
           address3_tmp <= (others => '0');
      else 
           address3_tmp <= address3;
      end if;
--synthesis translate_on
end process;

p_rom_access: process (clk)  
begin 
    if (clk'event and clk = '1') then
        if (ce0 = '1') then 
            q0 <= mem0(CONV_INTEGER(address0_tmp)); 
        end if;
        if (ce1 = '1') then 
            q1 <= mem0(CONV_INTEGER(address1_tmp)); 
        end if;
        if (ce2 = '1') then 
            q2 <= mem1(CONV_INTEGER(address2_tmp)); 
        end if;
        if (ce3 = '1') then 
            q3 <= mem1(CONV_INTEGER(address3_tmp)); 
        end if;
    end if;
end process;

end rtl;


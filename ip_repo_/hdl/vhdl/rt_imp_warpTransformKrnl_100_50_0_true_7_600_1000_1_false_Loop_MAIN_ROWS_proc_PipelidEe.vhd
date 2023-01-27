-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
-- Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelidEe is 
    generic(
             DataWidth     : integer := 23; 
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


architecture rtl of rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelidEe is 

signal address0_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address1_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address2_tmp : std_logic_vector(AddressWidth-1 downto 0); 
signal address3_tmp : std_logic_vector(AddressWidth-1 downto 0); 
type mem_array is array (0 to AddressRange-1) of std_logic_vector (DataWidth-1 downto 0); 
signal mem0 : mem_array := (
    0 => "01111111111111111111111", 1 => "00111111111111111111111", 
    2 => "00011111111111111111111", 3 => "00001111111111111111111", 
    4 => "00000111111111111111111", 5 => "00000011111111111111111", 
    6 => "00000001111111111111111", 7 => "00000000111111111111111", 
    8 => "00000000011111111111111", 9 => "00000000001111111111111", 
    10 => "00000000000111111111111", 11 => "00000000000011111111111", 
    12 => "00000000000001111111111", 13 => "00000000000000111111111", 
    14 => "00000000000000011111111", 15 => "00000000000000001111111", 
    16 => "00000000000000000111111", 17 => "00000000000000000011111", 
    18 => "00000000000000000001111", 19 => "00000000000000000000111", 
    20 => "00000000000000000000011", 21 => "00000000000000000000001", 
    22 to 29=> "00000000000000000000000", 30 to 31=> "11111111111111111111111" );
signal mem1 : mem_array := (
    0 => "01111111111111111111111", 1 => "00111111111111111111111", 
    2 => "00011111111111111111111", 3 => "00001111111111111111111", 
    4 => "00000111111111111111111", 5 => "00000011111111111111111", 
    6 => "00000001111111111111111", 7 => "00000000111111111111111", 
    8 => "00000000011111111111111", 9 => "00000000001111111111111", 
    10 => "00000000000111111111111", 11 => "00000000000011111111111", 
    12 => "00000000000001111111111", 13 => "00000000000000111111111", 
    14 => "00000000000000011111111", 15 => "00000000000000001111111", 
    16 => "00000000000000000111111", 17 => "00000000000000000011111", 
    18 => "00000000000000000001111", 19 => "00000000000000000000111", 
    20 => "00000000000000000000011", 21 => "00000000000000000000001", 
    22 to 29=> "00000000000000000000000", 30 to 31=> "11111111111111111111111" );


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


-- alu.vhd
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity alu is
    generic (
        W : natural := 4
    );
    port (
        a    : in  std_logic_vector(W-1 downto 0);
        b    : in  std_logic_vector(W-1 downto 0);
        sel  : in  std_logic_vector(1 downto 0); -- 00:add,01:sub,10:and,11:or
        outp : out std_logic_vector(W-1 downto 0);
        zero : out std_logic
    );
end entity;

architecture rtl of alu is
    signal sa, sb : signed(W-1 downto 0);
    signal result : signed(W-1 downto 0);
begin
    sa <= signed(a);
    sb <= signed(b);

    process(sa, sb, sel)
    begin
        case sel is
            when "00" => result <= sa + sb;
            when "01" => result <= sa - sb;
            when "10" => result <= sa and sb; -- type mismatch compensated below
            when "11" => result <= sa or sb;
            when others => result <= (others => '0');
        end case;
    end process;

    outp <= std_logic_vector(result);
    zero <= '1' when result = 0 else '0';
end architecture;
